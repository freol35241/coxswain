//! Cyphal actuator backend: the reference-deployment output backend (D-027,
//! D-028, TASKS Phase 7). Commands each effector in physical units over
//! Cyphal/CAN and consumes the nodes' feedback and power reports for the
//! command-then-report comparison (D-010).
//!
//! Unlike the `$CXOUT` serial bridge, which renders microseconds for a dumb
//! far end, a Cyphal actuator node is commanded in the allocator's physical
//! units (newtons or radians) and owns its local servo calibration (D-027).
//! So this backend sends the per-effector value straight through, one
//! single-frame Cyphal message per effector per tick.
//!
//! ## Message contract (our nodes, D-028)
//!
//! Each effector has a command subject the conn node publishes its setpoint
//! on, and a feedback subject its node reports its achieved value on; the
//! power-monitoring node publishes bus voltage on a power subject. Every
//! payload is a little-endian `f32` (Cyphal serialization is little-endian),
//! one frame each, well inside the single-frame limit. Subjects and node ids
//! are per-vessel firmware contract carried in config, not fixed here.
//!
//! Byte- and frame-level only: this crate is `no_std` and owns no CAN socket.
//! The hosted profile reads and writes `(can_id, data)` pairs (SocketCAN) and
//! feeds received frames to `handle_frame`.

use coxswain_contract::{BoundedList, MAX_EFFECTORS, PowerStatus, Timestamp};
use coxswain_cyphal::{
    DecodeError, MessageId, NodeId, Priority, SubjectId, TRANSFER_ID_MAX, decode_single_frame,
    encode_single_frame,
};

use crate::output::{ActuatorSink, OutputBackend, OutputFrame};

/// One effector's Cyphal wiring: which allocator output value its setpoint
/// reads, the subject the conn node commands it on, and the subject its node
/// reports achieved on.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct CyphalEffector {
    pub effector_index: usize,
    pub command_subject: SubjectId,
    pub feedback_subject: SubjectId,
}

/// Errors decoding a received Cyphal frame into a report.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ReportError {
    /// The frame's transport framing did not decode (service frame, anonymous,
    /// empty, or multi-frame; see `coxswain_cyphal::DecodeError`).
    Transport(DecodeError),
    /// A message on one of our subjects carried fewer than the 4 bytes an
    /// `f32` payload needs.
    ShortPayload,
    /// The `f32` payload was NaN or infinite.
    NonFinite,
    /// A power report's voltage was negative, which is not a usable bus
    /// voltage (same guard as the `$CXPWR` path).
    NegativeVoltage,
}

/// A decoded report from an actuator or power node.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum CyphalReport {
    /// A node's achieved value against what it was last commanded (D-010
    /// command-then-report). `diverged` is set when they differ by more than
    /// the configured tolerance.
    Feedback {
        effector_index: usize,
        commanded: f32,
        achieved: f32,
        diverged: bool,
    },
    /// A power-monitoring node's bus voltage.
    Power(PowerStatus),
}

/// The Cyphal actuator output backend and command-then-report reader.
#[derive(Clone, Debug)]
pub struct CyphalActuatorBackend {
    conn_node_id: NodeId,
    priority: Priority,
    power_subject: SubjectId,
    effectors: BoundedList<CyphalEffector, MAX_EFFECTORS>,
    /// Divergence threshold in physical units: `|achieved - commanded|` beyond
    /// this flags a command-then-report mismatch. Provisional (D-028).
    report_tolerance: f64,
    transfer_id: [u8; MAX_EFFECTORS],
    last_commanded: [f32; MAX_EFFECTORS],
}

impl CyphalActuatorBackend {
    pub fn new(
        conn_node_id: NodeId,
        priority: Priority,
        power_subject: SubjectId,
        effectors: BoundedList<CyphalEffector, MAX_EFFECTORS>,
        report_tolerance: f64,
    ) -> Self {
        Self {
            conn_node_id,
            priority,
            power_subject,
            effectors,
            report_tolerance,
            transfer_id: [0; MAX_EFFECTORS],
            last_commanded: [0.0; MAX_EFFECTORS],
        }
    }

    /// Decode one received `(can_id, data)` frame. `Ok(Some(report))` for a
    /// frame on a subject we command-then-report or power-monitor; `Ok(None)`
    /// for a well-formed Cyphal message on some other subject (not ours to
    /// interpret); `Err` for malformed framing or payload. `acquired_at` is
    /// the caller-injected capture time (driver timestamping policy).
    pub fn handle_frame(
        &self,
        can_id: u32,
        data: &[u8],
        acquired_at: Timestamp,
    ) -> Result<Option<CyphalReport>, ReportError> {
        let frame = decode_single_frame(can_id, data).map_err(ReportError::Transport)?;
        let subject = frame.id.subject_id;

        if subject == self.power_subject {
            let voltage = read_f32(frame.payload)?;
            if voltage < 0.0 {
                return Err(ReportError::NegativeVoltage);
            }
            return Ok(Some(CyphalReport::Power(PowerStatus {
                t: acquired_at,
                voltage_v: voltage as f64,
            })));
        }

        for slot in 0..self.effectors.len() {
            let eff = self.effectors.as_slice()[slot];
            if subject == eff.feedback_subject {
                let achieved = read_f32(frame.payload)?;
                let commanded = self.last_commanded[slot];
                let diverged = abs_f32(achieved - commanded) as f64 > self.report_tolerance;
                return Ok(Some(CyphalReport::Feedback {
                    effector_index: eff.effector_index,
                    commanded,
                    achieved,
                    diverged,
                }));
            }
        }

        Ok(None)
    }
}

impl OutputBackend for CyphalActuatorBackend {
    /// One single-frame Cyphal message per effector, carrying its physical
    /// setpoint as a little-endian `f32`, on that effector's command subject
    /// with the conn node as source. Records each commanded value for the
    /// command-then-report comparison and advances that subject's transfer-id.
    fn write_outputs(&mut self, values: &[f64], sink: &mut dyn ActuatorSink) {
        for slot in 0..self.effectors.len() {
            let eff = self.effectors.as_slice()[slot];
            let value = values.get(eff.effector_index).copied().unwrap_or(0.0) as f32;
            self.last_commanded[slot] = value;
            let id = MessageId {
                priority: self.priority,
                subject_id: eff.command_subject,
                source_node_id: self.conn_node_id,
            };
            if let Some(frame) =
                encode_single_frame(id, self.transfer_id[slot], &value.to_le_bytes())
            {
                sink.emit(OutputFrame::Can {
                    can_id: frame.can_id,
                    data: frame.data(),
                });
            }
            self.transfer_id[slot] = (self.transfer_id[slot] + 1) & TRANSFER_ID_MAX;
        }
    }
}

/// Read a little-endian `f32` from the first 4 payload bytes, rejecting a
/// short payload and a non-finite value.
fn read_f32(payload: &[u8]) -> Result<f32, ReportError> {
    let bytes: [u8; 4] = payload
        .get(..4)
        .and_then(|s| s.try_into().ok())
        .ok_or(ReportError::ShortPayload)?;
    let value = f32::from_le_bytes(bytes);
    // `is_finite` is a core method (no libm); it rejects NaN and both
    // infinities, a broken node's garbage reading.
    if value.is_finite() {
        Ok(value)
    } else {
        Err(ReportError::NonFinite)
    }
}

/// `f32::abs` is not in `core` (it would pull in libm here), so the
/// command-then-report divergence magnitude is taken by hand.
fn abs_f32(x: f32) -> f32 {
    if x < 0.0 { -x } else { x }
}

#[cfg(test)]
mod tests {
    use super::*;
    use coxswain_cyphal::decode_single_frame;

    const CMD0: u16 = 100;
    const CMD1: u16 = 101;
    const FB0: u16 = 200;
    const FB1: u16 = 201;
    const POWER: u16 = 300;
    const CONN_NODE: u8 = 5;
    const NODE0: u8 = 11;
    const TOLERANCE: f64 = 1.0;

    fn subj(v: u16) -> SubjectId {
        SubjectId::new(v).unwrap()
    }

    /// Two-effector backend: effector 0 on cmd/fb 100/200, effector 1 on
    /// 101/201, power on 300, conn node 5.
    fn backend() -> CyphalActuatorBackend {
        let effectors = BoundedList::from_slice(&[
            CyphalEffector {
                effector_index: 0,
                command_subject: subj(CMD0),
                feedback_subject: subj(FB0),
            },
            CyphalEffector {
                effector_index: 1,
                command_subject: subj(CMD1),
                feedback_subject: subj(FB1),
            },
        ])
        .unwrap();
        CyphalActuatorBackend::new(
            NodeId::new(CONN_NODE).unwrap(),
            Priority::High,
            subj(POWER),
            effectors,
            TOLERANCE,
        )
    }

    /// Collects emitted CAN frames into a fixed buffer (no_std: no Vec).
    struct CanCollect {
        frames: [(u32, [u8; 8], usize); MAX_EFFECTORS],
        n: usize,
    }

    impl CanCollect {
        fn new() -> Self {
            Self {
                frames: [(0, [0; 8], 0); MAX_EFFECTORS],
                n: 0,
            }
        }
    }

    impl ActuatorSink for CanCollect {
        fn emit(&mut self, frame: OutputFrame) {
            match frame {
                OutputFrame::Can { can_id, data } => {
                    let mut buf = [0u8; 8];
                    buf[..data.len()].copy_from_slice(data);
                    self.frames[self.n] = (can_id, buf, data.len());
                    self.n += 1;
                }
                OutputFrame::Serial(_) => panic!("cyphal backend emitted a serial line"),
            }
        }
    }

    fn emit(driver: &mut CyphalActuatorBackend, values: &[f64]) -> CanCollect {
        let mut sink = CanCollect::new();
        driver.write_outputs(values, &mut sink);
        sink
    }

    /// Build a received feedback/power frame the way a node would.
    fn node_frame(subject: u16, source_node: u8, value: f32) -> ([u8; 8], usize, u32) {
        let id = MessageId {
            priority: Priority::Nominal,
            subject_id: subj(subject),
            source_node_id: NodeId::new(source_node).unwrap(),
        };
        let frame = encode_single_frame(id, 0, &value.to_le_bytes()).unwrap();
        let mut buf = [0u8; 8];
        buf[..frame.len()].copy_from_slice(frame.data());
        (buf, frame.len(), frame.can_id)
    }

    #[test]
    fn commands_one_frame_per_effector_with_le_f32_payload() {
        let mut b = backend();
        let out = emit(&mut b, &[10.0, -3.0]);
        assert_eq!(out.n, 2);

        let (id0, data0, len0) = out.frames[0];
        let f0 = decode_single_frame(id0, &data0[..len0]).unwrap();
        assert_eq!(f0.id.subject_id, subj(CMD0));
        assert_eq!(f0.id.source_node_id, NodeId::new(CONN_NODE).unwrap());
        assert_eq!(f0.transfer_id, 0);
        assert_eq!(f32::from_le_bytes(f0.payload.try_into().unwrap()), 10.0);

        let (id1, data1, len1) = out.frames[1];
        let f1 = decode_single_frame(id1, &data1[..len1]).unwrap();
        assert_eq!(f1.id.subject_id, subj(CMD1));
        assert_eq!(f32::from_le_bytes(f1.payload.try_into().unwrap()), -3.0);
    }

    #[test]
    fn transfer_id_advances_per_subject_across_ticks() {
        let mut b = backend();
        emit(&mut b, &[0.0, 0.0]);
        let out = emit(&mut b, &[0.0, 0.0]);
        let (id0, data0, len0) = out.frames[0];
        assert_eq!(
            decode_single_frame(id0, &data0[..len0])
                .unwrap()
                .transfer_id,
            1
        );
    }

    #[test]
    fn matching_feedback_within_tolerance_does_not_diverge() {
        let mut b = backend();
        emit(&mut b, &[10.0, -3.0]); // commanded
        let (data, len, id) = node_frame(FB0, NODE0, 10.4); // within 1.0
        let report = b
            .handle_frame(id, &data[..len], Timestamp::from_nanos(1))
            .unwrap()
            .unwrap();
        assert_eq!(
            report,
            CyphalReport::Feedback {
                effector_index: 0,
                commanded: 10.0,
                achieved: 10.4,
                diverged: false,
            }
        );
    }

    #[test]
    fn feedback_beyond_tolerance_diverges() {
        let mut b = backend();
        emit(&mut b, &[10.0, -3.0]);
        let (data, len, id) = node_frame(FB1, NODE0, 5.0); // commanded -3.0, off by 8
        let report = b
            .handle_frame(id, &data[..len], Timestamp::from_nanos(1))
            .unwrap()
            .unwrap();
        let CyphalReport::Feedback {
            effector_index,
            diverged,
            ..
        } = report
        else {
            panic!("expected Feedback, got {report:?}");
        };
        assert_eq!(effector_index, 1);
        assert!(diverged);
    }

    #[test]
    fn power_subject_yields_voltage() {
        let b = backend();
        let (data, len, id) = node_frame(POWER, 21, 12.6);
        let report = b
            .handle_frame(id, &data[..len], Timestamp::from_nanos(7))
            .unwrap()
            .unwrap();
        let CyphalReport::Power(status) = report else {
            panic!("expected Power, got {report:?}");
        };
        assert!((status.voltage_v - 12.6_f64).abs() < 1e-4);
        assert_eq!(status.t, Timestamp::from_nanos(7));
    }

    #[test]
    fn negative_voltage_is_rejected() {
        let b = backend();
        let (data, len, id) = node_frame(POWER, 21, -1.0);
        assert_eq!(
            b.handle_frame(id, &data[..len], Timestamp::from_nanos(1)),
            Err(ReportError::NegativeVoltage)
        );
    }

    #[test]
    fn non_finite_payload_is_rejected() {
        let b = backend();
        let (data, len, id) = node_frame(FB0, NODE0, f32::NAN);
        assert_eq!(
            b.handle_frame(id, &data[..len], Timestamp::from_nanos(1)),
            Err(ReportError::NonFinite)
        );
    }

    #[test]
    fn short_payload_is_rejected() {
        let b = backend();
        // A feedback frame with only 2 payload bytes.
        let id = MessageId {
            priority: Priority::Nominal,
            subject_id: subj(FB0),
            source_node_id: NodeId::new(NODE0).unwrap(),
        };
        let frame = encode_single_frame(id, 0, &[1, 2]).unwrap();
        assert_eq!(
            b.handle_frame(frame.can_id, frame.data(), Timestamp::from_nanos(1)),
            Err(ReportError::ShortPayload)
        );
    }

    #[test]
    fn unknown_subject_is_none() {
        let b = backend();
        let (data, len, id) = node_frame(999, NODE0, 1.0);
        assert_eq!(
            b.handle_frame(id, &data[..len], Timestamp::from_nanos(1)),
            Ok(None)
        );
    }
}
