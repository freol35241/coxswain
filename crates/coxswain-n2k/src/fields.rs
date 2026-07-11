//! Little-endian field readers with the N2K "not available" sentinel rule
//! applied uniformly: all-ones for unsigned, max-positive for signed. A
//! sentinel decodes to `None`, never to a number, for every field in this
//! crate's PGN set including `SID` (docs/TASKS.md Phase 7 N2K item).
//! Reserved bits are read by the caller and discarded, never surfaced.

pub(crate) fn u16_le_at(data: &[u8], offset: usize) -> u16 {
    u16::from_le_bytes([data[offset], data[offset + 1]])
}

pub(crate) fn i16_le_at(data: &[u8], offset: usize) -> i16 {
    i16::from_le_bytes([data[offset], data[offset + 1]])
}

pub(crate) fn u32_le_at(data: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes([
        data[offset],
        data[offset + 1],
        data[offset + 2],
        data[offset + 3],
    ])
}

pub(crate) fn i32_le_at(data: &[u8], offset: usize) -> i32 {
    i32::from_le_bytes([
        data[offset],
        data[offset + 1],
        data[offset + 2],
        data[offset + 3],
    ])
}

/// `u8` sentinel (`0xFF`), no scaling: the raw code kept as-is (e.g. SID).
pub(crate) fn opt_u8_raw(raw: u8) -> Option<u8> {
    if raw == u8::MAX { None } else { Some(raw) }
}

/// `u8` sentinel (`0xFF`), `resolution` applied to a present value.
pub(crate) fn opt_u8_scaled(raw: u8, resolution: f64) -> Option<f64> {
    if raw == u8::MAX {
        None
    } else {
        Some(raw as f64 * resolution)
    }
}

pub(crate) fn opt_u16_scaled(raw: u16, resolution: f64) -> Option<f64> {
    if raw == u16::MAX {
        None
    } else {
        Some(raw as f64 * resolution)
    }
}

pub(crate) fn opt_i16_scaled(raw: i16, resolution: f64) -> Option<f64> {
    if raw == i16::MAX {
        None
    } else {
        Some(raw as f64 * resolution)
    }
}

pub(crate) fn opt_u32_scaled(raw: u32, resolution: f64) -> Option<f64> {
    if raw == u32::MAX {
        None
    } else {
        Some(raw as f64 * resolution)
    }
}

pub(crate) fn opt_i32_scaled(raw: i32, resolution: f64) -> Option<f64> {
    if raw == i32::MAX {
        None
    } else {
        Some(raw as f64 * resolution)
    }
}
