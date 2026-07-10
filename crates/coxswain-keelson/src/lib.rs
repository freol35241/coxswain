//! Keelson adapter at the process boundary (D-003, D-007, D-008).
//!
//! Publishes raw and fused streams, health, and conn state under Keelson
//! keyspace conventions, and serves the conn RPC endpoints. The claimant
//! client is the other side of the same protocol: the teleoperation client
//! and the integration tests' tool.
//!
//! Host-only. The core runs on injected monotonic time; wall-clock time
//! enters here at the edge, as `SystemTime` parameters on every publish.

mod claimant;
mod convert;
mod error;
pub mod keys;
pub mod proto;
mod vessel;

pub use claimant::{ClaimantClient, StateUpdate};
pub use convert::{ned_cov_to_enu, setpoint_from_proto, setpoint_to_proto};
pub use error::Error;
pub use proto::coxswain::conn_reply::Result as ConnReplyResult;
pub use vessel::{ConnEvent, Event, ReplyHandle, VesselEndpoint};
