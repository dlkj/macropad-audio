use fixed::{
    traits::{LosslessTryFrom, LossyFrom},
    types::{I16F16, U16F16},
};
use num_traits::clamp;

#[derive(Clone, Copy)]
enum Event {
    KeyUp(u16),
    KeyDown(u16),
}

pub(crate) struct Synth {
    phase: U16F16, // 0 to 128
    event: Event,
    duration: u32, // in samples
}

impl Synth {
    const PWM_MAX: u16 = 4096;

    pub const fn new() -> Self {
        Self {
            phase: U16F16::ZERO,
            duration: 0,
            event: Event::KeyUp(0),
        }
    }

    pub fn next_sample(
        &mut self,
        next_freq: u16,
        attack: u16,
        decay: u16,
        sustain: u8,
        release: u16,
    ) -> u16 {
        self.event = match self.event {
            Event::KeyUp(f) => {
                if next_freq == 0 {
                    self.duration = self.duration.saturating_add(1);
                    Event::KeyUp(f)
                } else {
                    self.duration = 0;
                    Event::KeyDown(next_freq)
                }
            }
            Event::KeyDown(f) => {
                if next_freq == 0 {
                    self.duration = 0;
                    Event::KeyUp(f)
                } else if next_freq == f {
                    self.duration = self.duration.saturating_add(1);
                    Event::KeyDown(next_freq)
                } else {
                    self.duration = 0;
                    Event::KeyDown(next_freq)
                }
            }
        };

        let (key_down, playing_freq) = match self.event {
            Event::KeyUp(f) => (false, f),
            Event::KeyDown(f) => (true, f),
        };

        self.phase += U16F16::from_num(128.0 / 22190.0) * U16F16::from_num(playing_freq);
        while self.phase >= 128 {
            self.phase -= U16F16::from_num(128);
        }

        let envelope = Self::envelope(key_down, self.duration, attack, decay, sustain, release);

        let scaled = I16F16::from_num::<i16>(SIN_12BIT_128[u16::lossy_from(self.phase) as usize])
            * I16F16::lossless_try_from(envelope).unwrap();

        (clamp(scaled.to_num::<i32>(), -2047, 2047) + 2047) as u16
    }

    fn envelope(
        key_down: bool,
        duration: u32,
        attack: u16,
        decay: u16,
        sustain: u8,
        release: u16,
    ) -> U16F16 {
        let attack = attack * 22;
        let decay = decay * 22;
        let sustain = U16F16::from(clamp(sustain, 0, 100)) / 100;
        let release = release * 22;

        let duration = duration.try_into().unwrap_or(u16::MAX);

        if key_down {
            if (0..attack).contains(&duration) {
                (U16F16::from_num(duration) / U16F16::from_num(attack))
                    .lerp(U16F16::ZERO, U16F16::ONE)
            } else if (attack..(attack + decay)).contains(&duration) {
                (U16F16::from_num(duration - attack) / U16F16::from_num(decay))
                    .lerp(U16F16::ONE, sustain)
            } else {
                sustain
            }
        } else if (0..release).contains(&duration) {
            (U16F16::from_num(duration) / U16F16::from_num(release)).lerp(sustain, U16F16::ZERO)
        } else {
            U16F16::ZERO
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
