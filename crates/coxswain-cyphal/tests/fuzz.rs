//! CI-runnable pseudo-fuzz: hand-rolled xorshift64* RNG (same construction as
//! the other parser crates), no `rand` dependency. The decoder must never
//! panic on any `(can_id, data)` pair, only ever return `Ok` or a typed
//! `Err`; and every encodable payload must round-trip.

use coxswain_cyphal::{
    MAX_SINGLE_FRAME_PAYLOAD, MessageId, NodeId, Priority, SubjectId, decode_single_frame,
    encode_single_frame,
};

const ITERATIONS: u64 = 20_000;

struct Rng(u64);

impl Rng {
    fn next_u64(&mut self) -> u64 {
        // xorshift64*, the same construction used across the repo's fuzz tests.
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    fn below(&mut self, n: usize) -> usize {
        (self.next_u64() % n as u64) as usize
    }
}

#[test]
fn decode_never_panics_on_arbitrary_input() {
    let mut rng = Rng(0x1234_5678_9ABC_DEF0);
    for _ in 0..ITERATIONS {
        let can_id = self::next_u32(&mut rng);
        let len = rng.below(9); // 0..=8 data bytes
        let mut data = [0u8; 8];
        for b in data.iter_mut().take(len) {
            *b = (rng.next_u64() & 0xFF) as u8;
        }
        // Sole requirement: total function, no panic.
        let _ = decode_single_frame(can_id, &data[..len]);
    }
}

#[test]
fn encode_then_decode_round_trips_for_valid_payloads() {
    let mut rng = Rng(0x0FED_CBA9_8765_4321);
    for _ in 0..ITERATIONS {
        let id = MessageId {
            priority: PRIORITIES[rng.below(PRIORITIES.len())],
            subject_id: SubjectId::new((rng.next_u64() % 8192) as u16).unwrap(),
            source_node_id: NodeId::new((rng.next_u64() % 128) as u8).unwrap(),
        };
        let transfer_id = (rng.next_u64() & 0x1F) as u8;
        let len = rng.below(MAX_SINGLE_FRAME_PAYLOAD + 1); // 0..=7
        let mut payload = [0u8; MAX_SINGLE_FRAME_PAYLOAD];
        for b in payload.iter_mut().take(len) {
            *b = (rng.next_u64() & 0xFF) as u8;
        }
        let payload = &payload[..len];

        let frame = encode_single_frame(id, transfer_id, payload).unwrap();
        let decoded = decode_single_frame(frame.can_id, frame.data()).unwrap();
        assert_eq!(decoded.id, id);
        assert_eq!(decoded.transfer_id, transfer_id);
        assert_eq!(decoded.payload, payload);
    }
}

const PRIORITIES: [Priority; 8] = [
    Priority::Exceptional,
    Priority::Immediate,
    Priority::Fast,
    Priority::High,
    Priority::Nominal,
    Priority::Low,
    Priority::Slow,
    Priority::Optional,
];

fn next_u32(rng: &mut Rng) -> u32 {
    (rng.next_u64() >> 16) as u32
}
