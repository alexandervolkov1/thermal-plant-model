pub(crate) struct XorShift32 {
    state: u32,
}

impl XorShift32 {
    pub(crate) fn new(seed: u32) -> Self {
        Self {
            state: if seed == 0 { 0xA341_316C } else { seed },
        }
    }

    fn next_u32(&mut self) -> u32 {
        let mut x = self.state;

        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;

        self.state = x;
        x
    }

    pub(crate) fn next_signed_unit(&mut self) -> f32 {
        let value = (self.next_u32() >> 8) as f32 / 16_777_216.0;

        2.0 * value - 1.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sequence_is_reproducible() {
        let mut rng = XorShift32::new(1);

        assert_eq!(rng.next_u32(), 270_369);
        assert_eq!(rng.next_u32(), 67_634_689);
        assert_eq!(rng.next_u32(), 2_647_435_461);
    }
}
