//! Standard base64 (RFC 4648, padded), hand-rolled rather than a dependency.
//!
//! Used for exactly one case: a raw-log chunk that is not printable ASCII
//! (module doc comment). That is rare on a text-only NMEA wire (stray
//! garbage bytes, a mid-sentence resync), so pulling in a crate for it would
//! be the wrong trade against the "smallest approach that works" rule
//! (CLAUDE.md); the codec is ~40 lines either way.

const ALPHABET: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

/// `None` at index 64 (padding) and every byte outside the alphabet.
fn decode_table(byte: u8) -> Option<u8> {
    match byte {
        b'A'..=b'Z' => Some(byte - b'A'),
        b'a'..=b'z' => Some(byte - b'a' + 26),
        b'0'..=b'9' => Some(byte - b'0' + 52),
        b'+' => Some(62),
        b'/' => Some(63),
        _ => None,
    }
}

pub fn encode(bytes: &[u8]) -> String {
    let mut out = String::with_capacity(bytes.len().div_ceil(3) * 4);
    for chunk in bytes.chunks(3) {
        let b0 = chunk[0];
        let b1 = chunk.get(1).copied().unwrap_or(0);
        let b2 = chunk.get(2).copied().unwrap_or(0);
        out.push(ALPHABET[(b0 >> 2) as usize] as char);
        out.push(ALPHABET[(((b0 & 0x03) << 4) | (b1 >> 4)) as usize] as char);
        out.push(if chunk.len() > 1 {
            ALPHABET[(((b1 & 0x0F) << 2) | (b2 >> 6)) as usize] as char
        } else {
            '='
        });
        out.push(if chunk.len() > 2 {
            ALPHABET[(b2 & 0x3F) as usize] as char
        } else {
            '='
        });
    }
    out
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DecodeError;

impl core::fmt::Display for DecodeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "invalid base64")
    }
}

impl std::error::Error for DecodeError {}

pub fn decode(text: &str) -> Result<Vec<u8>, DecodeError> {
    let bytes = text.as_bytes();
    if !bytes.len().is_multiple_of(4) {
        return Err(DecodeError);
    }
    let mut out = Vec::with_capacity(bytes.len() / 4 * 3);
    for group in bytes.chunks(4) {
        let pad = group.iter().rev().take_while(|&&b| b == b'=').count();
        if pad > 2 {
            return Err(DecodeError);
        }
        let mut v = [0u8; 4];
        for (i, &b) in group.iter().enumerate() {
            v[i] = if b == b'=' {
                0
            } else {
                decode_table(b).ok_or(DecodeError)?
            };
        }
        out.push((v[0] << 2) | (v[1] >> 4));
        if pad < 2 {
            out.push((v[1] << 4) | (v[2] >> 2));
        }
        if pad < 1 {
            out.push((v[2] << 6) | v[3]);
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn known_vectors_match_rfc_4648() {
        assert_eq!(encode(b""), "");
        assert_eq!(encode(b"f"), "Zg==");
        assert_eq!(encode(b"fo"), "Zm8=");
        assert_eq!(encode(b"foo"), "Zm9v");
        assert_eq!(encode(b"foob"), "Zm9vYg==");
        assert_eq!(encode(b"fooba"), "Zm9vYmE=");
        assert_eq!(encode(b"foobar"), "Zm9vYmFy");
    }

    #[test]
    fn decode_inverts_encode_for_every_length_mod_3() {
        for text in ["", "f", "fo", "foo", "foob", "fooba", "foobar"] {
            assert_eq!(decode(&encode(text.as_bytes())).unwrap(), text.as_bytes());
        }
    }

    #[test]
    fn round_trips_non_ascii_bytes() {
        let bytes: Vec<u8> = (0u8..=255).collect();
        assert_eq!(decode(&encode(&bytes)).unwrap(), bytes);
    }

    #[test]
    fn decode_rejects_wrong_length() {
        assert_eq!(decode("A"), Err(DecodeError));
        assert_eq!(decode("AB"), Err(DecodeError));
    }

    #[test]
    fn decode_rejects_invalid_character() {
        assert_eq!(decode("A!=="), Err(DecodeError));
    }
}
