//! Shared error type for both log formats: an I/O failure or a malformed
//! line. One enum rather than two near-identical ones, since every caller
//! (the estimator harness, cxconvert, the hosted recorder's reader-side
//! tests) handles both the same way: report and stop, or count and move on.

use std::fmt;

#[derive(Debug)]
pub enum Error {
    Io(std::io::Error),
    Json(serde_json::Error),
    /// A raw-log line's JSON parsed but had neither a `line` nor a `b64`
    /// field, or its `b64` field was not valid base64.
    Malformed(String),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Io(e) => write!(f, "{e}"),
            Error::Json(e) => write!(f, "{e}"),
            Error::Malformed(msg) => write!(f, "{msg}"),
        }
    }
}

impl std::error::Error for Error {}

impl From<std::io::Error> for Error {
    fn from(e: std::io::Error) -> Self {
        Error::Io(e)
    }
}

impl From<serde_json::Error> for Error {
    fn from(e: serde_json::Error) -> Self {
        Error::Json(e)
    }
}
