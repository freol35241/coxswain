use core::fmt;

#[derive(Debug)]
pub enum Error {
    Zenoh(zenoh::Error),
    Decode(prost::DecodeError),
    /// Structurally valid protobuf that violates the protocol: missing
    /// fields, out-of-range ids, over-capacity paths. Strict by default.
    Protocol(&'static str),
    /// A setpoint numeric field decoded as NaN or infinite. Caught here so a
    /// remote claimant streaming garbage never reaches guidance or the
    /// actuators as a contract `Setpoint`.
    NonFinite(&'static str),
    /// A setpoint geodetic or heading field decoded finite but outside its
    /// geometric bound. Caught here so a geometrically impossible position,
    /// or a heading far enough past a full turn to signal unit confusion,
    /// never reaches guidance as a contract `Setpoint`.
    OutOfRange(&'static str),
    /// An RPC produced no reply within the timeout.
    Timeout,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Zenoh(e) => write!(f, "zenoh: {e}"),
            Error::Decode(e) => write!(f, "protobuf decode: {e}"),
            Error::Protocol(msg) => write!(f, "protocol: {msg}"),
            Error::NonFinite(field) => write!(f, "setpoint field {field} is not finite"),
            Error::OutOfRange(field) => write!(f, "setpoint field {field} is out of range"),
            Error::Timeout => f.write_str("rpc timeout"),
        }
    }
}

impl std::error::Error for Error {}

impl From<zenoh::Error> for Error {
    fn from(e: zenoh::Error) -> Self {
        Error::Zenoh(e)
    }
}

impl From<prost::DecodeError> for Error {
    fn from(e: prost::DecodeError) -> Self {
        Error::Decode(e)
    }
}
