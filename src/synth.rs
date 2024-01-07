use fixed::{
    traits::{LosslessTryFrom, LossyFrom},
    types::I16F16,
};
use num_traits::clamp;

#[derive(Clone, Copy)]
enum Event {
    KeyUp(u16),
    KeyDown(u16),
}

pub(crate) struct Synth {
    event: Event,
    duration: u32, // in samples
    osc: [Oscillator; 3],
}

impl Synth {
    const PWM_MAX: i32 = 2047;
    const PWM_MIN: i32 = -2047;

    pub const fn new() -> Self {
        Self {
            duration: 0,
            event: Event::KeyUp(0),
            osc: [Oscillator::new(); 3],
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

        let playing_freq = I16F16::from_num(playing_freq);

        const MOD2_RATIO: I16F16 = I16F16::lit("5.01");
        const MOD2_LEVEL: I16F16 = I16F16::lit("0.9");
        let mod2 = self.osc[2].get_sample(playing_freq * MOD2_RATIO);

        const MOD1_RATIO: I16F16 = I16F16::lit("3.05");
        const MOD1_LEVEL: I16F16 = I16F16::lit("0.9");
        let mod1 = self.osc[1]
            .get_sample(playing_freq * MOD1_RATIO + playing_freq * MOD1_RATIO * mod2 * MOD2_LEVEL);

        let sample = self.osc[0].get_sample(playing_freq + playing_freq * mod1 * MOD1_LEVEL);

        let envelope = Self::envelope(key_down, self.duration, attack, decay, sustain, release);

        let mixed = sample * I16F16::lossless_try_from(envelope).unwrap();

        (clamp(
            (mixed * Self::PWM_MAX).to_num::<i32>(),
            Self::PWM_MIN,
            Self::PWM_MAX,
        ) + Self::PWM_MAX) as u16
    }

    fn envelope(
        key_down: bool,
        duration: u32,
        attack: u16,
        decay: u16,
        sustain: u8,
        release: u16,
    ) -> I16F16 {
        let attack = attack * 22;
        let decay = decay * 22;
        let sustain = I16F16::from(clamp(sustain, 0, 100)) / 100;
        let release = release * 22;

        let duration = duration.try_into().unwrap_or(u16::MAX);

        if key_down {
            if (0..attack).contains(&duration) {
                (I16F16::from_num(duration) / I16F16::from_num(attack))
                    .lerp(I16F16::ZERO, I16F16::ONE)
            } else if (attack..(attack + decay)).contains(&duration) {
                (I16F16::from_num(duration - attack) / I16F16::from_num(decay))
                    .lerp(I16F16::ONE, sustain)
            } else {
                sustain
            }
        } else if (0..release).contains(&duration) {
            (I16F16::from_num(duration) / I16F16::from_num(release)).lerp(sustain, I16F16::ZERO)
        } else {
            I16F16::ZERO
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct Oscillator {
    phase: I16F16, // 0 to 128
}

impl Oscillator {
    const SAMPLES_PER_RATE: I16F16 = I16F16::lit("0.0057683641"); // 128.0 / 22190.0

    pub const fn new() -> Self {
        Self {
            phase: I16F16::ZERO,
        }
    }

    pub fn get_sample(&mut self, freq: I16F16) -> I16F16 {
        self.phase += Self::SAMPLES_PER_RATE * freq;
        while self.phase >= 128 {
            self.phase -= I16F16::from_num(128);
        }

        SIN_12BIT_128[i16::lossy_from(self.phase) as usize]
    }
}

// -2,047 to 2,047
const SIN_12BIT_128: [I16F16; 128] = [
    I16F16::lit("0.00000000000"),
    I16F16::lit("0.04906767433"),
    I16F16::lit("0.09801714033"),
    I16F16::lit("0.14673047446"),
    I16F16::lit("0.19509032202"),
    I16F16::lit("0.24298017990"),
    I16F16::lit("0.29028467725"),
    I16F16::lit("0.33688985339"),
    I16F16::lit("0.38268343237"),
    I16F16::lit("0.42755509343"),
    I16F16::lit("0.47139673683"),
    I16F16::lit("0.51410274419"),
    I16F16::lit("0.55557023302"),
    I16F16::lit("0.59569930449"),
    I16F16::lit("0.63439328416"),
    I16F16::lit("0.67155895485"),
    I16F16::lit("0.70710678119"),
    I16F16::lit("0.74095112535"),
    I16F16::lit("0.77301045336"),
    I16F16::lit("0.80320753148"),
    I16F16::lit("0.83146961230"),
    I16F16::lit("0.85772861000"),
    I16F16::lit("0.88192126435"),
    I16F16::lit("0.90398929312"),
    I16F16::lit("0.92387953251"),
    I16F16::lit("0.94154406518"),
    I16F16::lit("0.95694033573"),
    I16F16::lit("0.97003125319"),
    I16F16::lit("0.98078528040"),
    I16F16::lit("0.98917650996"),
    I16F16::lit("0.99518472667"),
    I16F16::lit("0.99879545621"),
    I16F16::lit("1.00000000000"),
    I16F16::lit("0.99879545621"),
    I16F16::lit("0.99518472667"),
    I16F16::lit("0.98917650996"),
    I16F16::lit("0.98078528040"),
    I16F16::lit("0.97003125319"),
    I16F16::lit("0.95694033573"),
    I16F16::lit("0.94154406518"),
    I16F16::lit("0.92387953251"),
    I16F16::lit("0.90398929312"),
    I16F16::lit("0.88192126435"),
    I16F16::lit("0.85772861000"),
    I16F16::lit("0.83146961230"),
    I16F16::lit("0.80320753148"),
    I16F16::lit("0.77301045336"),
    I16F16::lit("0.74095112535"),
    I16F16::lit("0.70710678119"),
    I16F16::lit("0.67155895485"),
    I16F16::lit("0.63439328416"),
    I16F16::lit("0.59569930449"),
    I16F16::lit("0.55557023302"),
    I16F16::lit("0.51410274419"),
    I16F16::lit("0.47139673683"),
    I16F16::lit("0.42755509343"),
    I16F16::lit("0.38268343237"),
    I16F16::lit("0.33688985339"),
    I16F16::lit("0.29028467725"),
    I16F16::lit("0.24298017990"),
    I16F16::lit("0.19509032202"),
    I16F16::lit("0.14673047446"),
    I16F16::lit("0.09801714033"),
    I16F16::lit("0.04906767433"),
    I16F16::lit("0.00000000000"),
    I16F16::lit("-0.049067433"),
    I16F16::lit("-0.098014033"),
    I16F16::lit("-0.146737446"),
    I16F16::lit("-0.195092202"),
    I16F16::lit("-0.242987990"),
    I16F16::lit("-0.290287725"),
    I16F16::lit("-0.336885339"),
    I16F16::lit("-0.382683237"),
    I16F16::lit("-0.427559343"),
    I16F16::lit("-0.471393683"),
    I16F16::lit("-0.514104419"),
    I16F16::lit("-0.555573302"),
    I16F16::lit("-0.595690449"),
    I16F16::lit("-0.634398416"),
    I16F16::lit("-0.671555485"),
    I16F16::lit("-0.707108119"),
    I16F16::lit("-0.740952535"),
    I16F16::lit("-0.773015336"),
    I16F16::lit("-0.803203148"),
    I16F16::lit("-0.831461230"),
    I16F16::lit("-0.857721000"),
    I16F16::lit("-0.881926435"),
    I16F16::lit("-0.903989312"),
    I16F16::lit("-0.923873251"),
    I16F16::lit("-0.941546518"),
    I16F16::lit("-0.956943573"),
    I16F16::lit("-0.970035319"),
    I16F16::lit("-0.980788040"),
    I16F16::lit("-0.989170996"),
    I16F16::lit("-0.995182667"),
    I16F16::lit("-0.998795621"),
    I16F16::lit("-1.000000000"),
    I16F16::lit("-0.998795621"),
    I16F16::lit("-0.995182667"),
    I16F16::lit("-0.989170996"),
    I16F16::lit("-0.980788040"),
    I16F16::lit("-0.970035319"),
    I16F16::lit("-0.956943573"),
    I16F16::lit("-0.941546518"),
    I16F16::lit("-0.923873251"),
    I16F16::lit("-0.903989312"),
    I16F16::lit("-0.881926435"),
    I16F16::lit("-0.857721000"),
    I16F16::lit("-0.831461230"),
    I16F16::lit("-0.803203148"),
    I16F16::lit("-0.773015336"),
    I16F16::lit("-0.740952535"),
    I16F16::lit("-0.707108119"),
    I16F16::lit("-0.671555485"),
    I16F16::lit("-0.634398416"),
    I16F16::lit("-0.595690449"),
    I16F16::lit("-0.555573302"),
    I16F16::lit("-0.514104419"),
    I16F16::lit("-0.471393683"),
    I16F16::lit("-0.427559343"),
    I16F16::lit("-0.382683237"),
    I16F16::lit("-0.336885339"),
    I16F16::lit("-0.290287725"),
    I16F16::lit("-0.242987990"),
    I16F16::lit("-0.195092202"),
    I16F16::lit("-0.146737446"),
    I16F16::lit("-0.098014033"),
    I16F16::lit("-0.049067433"),
];
