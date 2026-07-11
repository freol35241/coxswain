//! CRC8/DVB-S2 (poly `0xD5`, init `0x00`, no reflection): the checksum CRSF
//! frames carry over type+payload only, never address or length (the
//! length byte is what lets a receiver find the crc byte in the first
//! place; covering it in its own checksum would be circular).
//!
//! Bitwise, not a table: CRSF frames top out at 64 bytes, so table-lookup
//! speed buys nothing here, and a hand-copied table is one more place a
//! transcription error could hide. `matches_the_published_check_value`
//! below is the actual correctness evidence, not the choice of algorithm.

pub(crate) fn crc8_dvb_s2(bytes: &[u8]) -> u8 {
    let mut crc = 0u8;
    for &b in bytes {
        crc ^= b;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 {
                (crc << 1) ^ 0xD5
            } else {
                crc << 1
            };
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matches_the_published_check_value() {
        // CRC-8/DVB-S2 catalogue check value: CRC of ASCII "123456789" is
        // 0xBC (catalogue.compress.ru / reveng.sourceforge.io CRC-8/DVB-S2
        // entry). This is the independent evidence the polynomial and
        // bit order above are correct, not just self-consistent.
        assert_eq!(crc8_dvb_s2(b"123456789"), 0xBC);
    }
}
