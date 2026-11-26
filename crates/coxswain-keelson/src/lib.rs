//! Keelson adapter at the process boundary (D-003, D-007, D-008).
//!
//! Two interfaces, opened separately (D-035): [`TelemetryPublisher`]
//! publishes raw and fused streams, health, conn state, and guidance
//! references under Keelson keyspace conventions; [`CommandServer`] serves
//! the conn RPCs and receives setpoints. Their far sides are [`Observer`],
//! which can only watch, and [`ClaimantClient`], which can claim.
//!
//! Host-only. The core runs on injected monotonic time; wall-clock time
//! enters here at the edge, as `SystemTime` parameters on every publish.

mod bus;
mod claimant;
mod command;
mod convert;
mod error;
pub mod keys;
mod observer;
pub mod proto;
mod telemetry;

pub use bus::{
    BusEffector, BusEffectorKind, BusInput, BusSensor, BusSensorKind, KeelsonBusInput,
    KeelsonBusOutput, KeelsonBusPeer, StdDefaults, StdSource,
};
pub use claimant::ClaimantClient;
pub use command::{CommandServer, ConnEvent, Event, ReplyHandle};
pub use convert::{ned_cov_to_enu, setpoint_from_proto, setpoint_to_proto};
pub use error::Error;
pub use observer::{HealthUpdate, Observer, StateUpdate};
pub use proto::coxswain::conn_reply::Result as ConnReplyResult;
pub use telemetry::{IoHealth, IoSource, PowerHealth, TelemetryPublisher};
