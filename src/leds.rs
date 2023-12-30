use core::fmt::Debug;
use smart_leds::{brightness, gamma, SmartLedsWrite, RGB8};

const STRIP_LEN: usize = 12;

pub(crate) struct LedController<S>
where
    S: SmartLedsWrite,
{
    ws: S,
    leds: [RGB8; STRIP_LEN],
    sin: extern "C" fn(f32) -> f32,
}

impl<S: SmartLedsWrite> LedController<S>
where
    <S as SmartLedsWrite>::Color: From<RGB8>,
    <S as SmartLedsWrite>::Error: Debug,
{
    const STRIP_BRIGHTNESS: u8 = 64u8; // Limit brightness to 64/256

    pub fn new(ws: S, sin: extern "C" fn(f32) -> f32) -> Self {
        Self {
            ws,
            leds: [(0, 0, 0).into(); STRIP_LEN],
            sin,
        }
    }

    pub fn next_frame(&mut self, t: f32, input: u32) {
        for (i, led) in self.leds.iter_mut().enumerate() {
            let rgb = if (input >> i) & 0b1 == 0 {
                Self::hsv2rgb_u8(0.0, 0.0, 1.0)
            } else {
                let hue_offs = i as f32 * 0.0208333;

                let sin_11 = (self.sin)((t + hue_offs) * 2.0 * core::f32::consts::PI);
                // Bring -1..1 sine range to 0..1 range:
                let sin_01 = (sin_11 + 1.0) * 0.5;

                let hue = 360.0 * sin_01;
                let sat = 1.0;
                let val = 1.0;

                Self::hsv2rgb_u8(hue, sat, val)
            };
            *led = rgb.into();
        }

        self.ws
            .write(brightness(
                gamma(self.leds.iter().copied()),
                Self::STRIP_BRIGHTNESS,
            ))
            .unwrap();
    }

    pub fn hsv2rgb(hue: f32, sat: f32, val: f32) -> (f32, f32, f32) {
        let c = val * sat;
        let v = (hue / 60.0) % 2.0 - 1.0;
        let v = if v < 0.0 { -v } else { v };
        let x = c * (1.0 - v);
        let m = val - c;
        let (r, g, b) = if hue < 60.0 {
            (c, x, 0.0)
        } else if hue < 120.0 {
            (x, c, 0.0)
        } else if hue < 180.0 {
            (0.0, c, x)
        } else if hue < 240.0 {
            (0.0, x, c)
        } else if hue < 300.0 {
            (x, 0.0, c)
        } else {
            (c, 0.0, x)
        };
        (r + m, g + m, b + m)
    }

    pub fn hsv2rgb_u8(h: f32, s: f32, v: f32) -> (u8, u8, u8) {
        let r = Self::hsv2rgb(h, s, v);

        (
            (r.0 * 255.0) as u8,
            (r.1 * 255.0) as u8,
            (r.2 * 255.0) as u8,
        )
    }
}
