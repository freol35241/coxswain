//! The output backend trait: the conn node's actuation output stage (D-027).
//!
//! Guidance's generalized tau has already been allocated (D-026) into
//! per-effector physical outputs (newtons for a thruster, radians for a
//! rudder), indexed parallel to the manifest effector table. Each manifest
//! bus kind has an output backend that takes those physical values and drives
//! the effectors on its bus. This trait is the shared contract across them,
//! crystallized when the second backend (Cyphal) landed alongside the first
//! (the `$CXOUT` serial bridge); with only one it would have been a single-use
//! abstraction (D-027).
//!
//! The physical-units boundary is deliberate. The serial bridge commands a
//! dumb far end, so it renders manifest PWM calibration into per-channel
//! microseconds at the conn node; a Cyphal node is commanded in physical units
//! and owns its local calibration (D-027). Both take the same `&[f64]` here;
//! each decides how its far end is addressed and calibrated.
//!
//! I/O is injected. This crate is `no_std` and owns no port; a backend renders
//! or encodes a tick's outputs and hands each transport write to the caller's
//! `ActuatorSink`, the same discipline as the caller-injected clock in the
//! `Driver` timestamping policy.

/// One transport write emitted by an output backend: a serial byte line, or a
/// CAN frame. A backend emits only the variant matching its bus, and the sink
/// the caller installs handles that variant; the two are paired at boot by bus
/// kind (D-027: an effector's output bus kind selects its backend).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum OutputFrame<'a> {
    /// A serial byte line, framed and ready for the wire (`$CXOUT`).
    Serial(&'a [u8]),
    /// A CAN frame: 29-bit identifier and up to 8 data bytes (Cyphal/CAN).
    Can { can_id: u32, data: &'a [u8] },
}

/// The transport an output backend writes to, injected by the caller. On the
/// hosted profile this wraps a serial port or a CAN socket; in tests it
/// collects the emitted frames.
pub trait ActuatorSink {
    fn emit(&mut self, frame: OutputFrame);
}

/// A manifest bus kind's actuation output stage. Takes the allocator's
/// per-effector physical outputs for one control tick and drives the effectors
/// through the injected sink.
pub trait OutputBackend {
    /// `values` are the per-effector physical outputs (newtons or radians),
    /// index-parallel to the manifest effector table. Called every control
    /// tick, including with the calibrated zero-demand values while disarmed,
    /// so the far end's dead-man watchdog always sees traffic on schedule.
    fn write_outputs(&mut self, values: &[f64], sink: &mut dyn ActuatorSink);
}
