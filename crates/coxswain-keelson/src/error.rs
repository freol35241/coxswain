use core::fmt;

#[derive(Debug)]
pub enum Error {
    Zenoh(zenoh::Error),
    Decode(prost::DecodeError),
    /// Structurally valid protobuf that violates the protocol: missing
    /// fields, out-of-range ids, over-capacity paths. Strict by default.
    Protocol(&'static str),
    /// An RPC produced no reply within the timeout.
    Timeout,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Zenoh(e) => write!(f, "zenoh: {e}"),
            Error::Decode(e) => write!(f, "protobuf decode: {e}"),
            Error::Protocol(msg) => write!(f, "protocol: {msg}"),
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
