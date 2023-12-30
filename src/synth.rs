use num_traits::clamp;

pub(crate) struct Synth {
    phase: f32,    // 0 to 128
    duration: u32, // in samples
}

impl Synth {
    const PWM_MAX: u16 = 4096;

    pub const fn new() -> Self {
        Self {
            phase: 0.0,
            duration: 0,
        }
    }

    pub fn next_sample(&mut self, wave_type: u16, attack: i32) -> u16 {
        match wave_type {
            0 => {
                self.duration = 0;
                Self::PWM_MAX / 2
            }
            _ => {
                self.phase += 128.0 / 22190.0 * wave_type as f32;
                while self.phase as usize >= 128 {
                    self.phase -= 128.0;
                }
                if self.phase < 0.0 {
                    self.phase = 0.0;
                }

                let scaled = ((SIN_12BIT_128[self.phase as usize]
                    + SIN_12BIT_128[(self.phase as usize + 12) / 2])
                    as f32
                    * Self::envelope(true, self.duration, clamp(attack, 0, 1024) as u32))
                    as i16;
                let scaled = (i32::from(clamp(scaled, -2047, 2047)) + 2047) as u16;
                self.duration += 1;
                scaled
            }
        }
    }

    fn envelope(_key_down: bool, duration: u32, attack: u32) -> f32 {
        // const ATTACK: u32 = 1000 * 22;
        // const DECAY: u32 = 400 * 22;
        // const SUSTAIN: f32 = 0.5;
        // const RELEASE: u32 = 200;

        if (0..(attack * 22)).contains(&duration) {
            (duration as f32) / (attack * 22) as f32
        } else {
            1.0
        }
    }
}

// -2,047 to 2,047
const SIN_12BIT_128: [i16; 128] = [
    0, 100, 201, 300, 399, 497, 594, 690, 783, 875, 965, 1052, 1137, 1219, 1299, 1375, 1447, 1517,
    1582, 1644, 1702, 1756, 1805, 1850, 1891, 1927, 1959, 1986, 2008, 2025, 2037, 2045, 2047, 2045,
    2037, 2025, 2008, 1986, 1959, 1927, 1891, 1850, 1805, 1756, 1702, 1644, 1582, 1517, 1447, 1375,
    1299, 1219, 1137, 1052, 965, 875, 783, 690, 594, 497, 399, 300, 201, 100, 0, -100, -201, -300,
    -399, -497, -594, -690, -783, -875, -965, -1052, -1137, -1219, -1299, -1375, -1447, -1517,
    -1582, -1644, -1702, -1756, -1805, -1850, -1891, -1927, -1959, -1986, -2008, -2025, -2037,
    -2045, -2047, -2045, -2037, -2025, -2008, -1986, -1959, -1927, -1891, -1850, -1805, -1756,
    -1702, -1644, -1582, -1517, -1447, -1375, -1299, -1219, -1137, -1052, -965, -875, -783, -690,
    -594, -497, -399, -300, -201, -100,
];
