//! Small deterministic PRNG used so native and WebAssembly scans can be replayed.

#[derive(Clone, Debug)]
pub(crate) struct DeterministicRng {
    state: u64,
    spare_normal: Option<f64>,
}

impl DeterministicRng {
    pub(crate) fn new(seed: u64) -> Self {
        Self {
            // SplitMix64 accepts zero, but scrambling here also separates nearby IDs.
            state: mix64(seed ^ 0xa076_1d64_78bd_642f),
            spare_normal: None,
        }
    }

    pub(crate) fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9e37_79b9_7f4a_7c15);
        mix64(self.state)
    }

    /// Uniform on [0, 1), using 53 random mantissa bits.
    pub(crate) fn uniform(&mut self) -> f64 {
        ((self.next_u64() >> 11) as f64) * (1.0 / ((1_u64 << 53) as f64))
    }

    pub(crate) fn normal(&mut self) -> f64 {
        if let Some(value) = self.spare_normal.take() {
            return value;
        }
        // Box-Muller. Keep u1 away from ln(0).
        let u1 = (1.0 - self.uniform()).max(f64::MIN_POSITIVE);
        let u2 = self.uniform();
        let radius = (-2.0 * u1.ln()).sqrt();
        let angle = core::f64::consts::TAU * u2;
        let (s, c) = angle.sin_cos();
        self.spare_normal = Some(radius * s);
        radius * c
    }
}

pub(crate) fn keyed_seed(seed: u64, values: &[u64]) -> u64 {
    values.iter().fold(mix64(seed), |acc, value| {
        mix64(acc ^ mix64(value.wrapping_add(0x517c_c1b7_2722_0a95)))
    })
}

fn mix64(mut z: u64) -> u64 {
    z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    z ^ (z >> 31)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn seeded_sequence_is_repeatable() {
        let mut a = DeterministicRng::new(42);
        let mut b = DeterministicRng::new(42);
        for _ in 0..100 {
            assert_eq!(a.next_u64(), b.next_u64());
        }
    }
}
