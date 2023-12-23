#![no_std]
#![no_main]

use core::fmt::Write;

use adafruit_macropad as bsp;
use bsp::{
    entry,
    hal::{
        self,
        clocks::{init_clocks_and_plls, Clock},
        fugit::ExtU64,
        gpio::{FunctionSpi, PullNone, PullUp},
        pac,
        pio::PIOExt,
        sio::Sio,
        timer::Instant,
        watchdog::Watchdog,
        Timer,
    },
};
use display::Screen;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::{digital::v2::InputPin, PwmPin};
use hal::fugit::RateExtU32;
use heapless::String;
use leds::LedController;
use panic_halt as _;
use sh1106::{mode::GraphicsMode, Builder};
use ws2812_pio::Ws2812;

mod display;
mod leds;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let clocks = init_clocks_and_plls(
        bsp::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let sin = hal::rom_data::float_funcs::fsin::ptr();

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let ws = Ws2812::new(
        pins.neopixel.into_function(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    let spi = hal::spi::Spi::<_, _, _>::new(
        pac.SPI1,
        (
            pins.mosi.reconfigure::<FunctionSpi, PullNone>(),
            pins.miso.reconfigure::<FunctionSpi, PullUp>(),
            pins.sclk.reconfigure::<FunctionSpi, PullNone>(),
        ),
    );

    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        4.MHz(),
        embedded_hal::spi::MODE_0,
    );

    let display: GraphicsMode<_> = Builder::new()
        .connect_spi(
            spi,
            pins.oled_dc.into_push_pull_output(),
            pins.oled_cs.into_push_pull_output(),
        )
        .into();

    let pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);
    let mut pwm = pwm_slices.pwm0;
    pwm.channel_a.output_to(pins.speaker);
    // 12 bit 22KHz sample output
    const PWM_MAX: u16 = 4096;
    const PWM_DIV_FRAC: u8 = 6;
    let sample_rate = (clocks.system_clock.freq().to_Hz() as f32
        / (PWM_MAX as f32 * (1.0 + PWM_DIV_FRAC as f32 / 16.0))) as u16;
    pwm.set_top(PWM_MAX);
    pwm.set_div_frac(6);
    pwm.enable();

    pins.speaker_shutdown
        .into_push_pull_output()
        .set_high()
        .unwrap();

    pins.oled_reset.into_push_pull_output().set_high().unwrap();

    let key_1 = pins.key1.into_pull_up_input();
    let key_2 = pins.key2.into_pull_up_input();
    let key_3 = pins.key3.into_pull_up_input();
    let key_4 = pins.key4.into_pull_up_input();

    let mut screen_controller = Screen::new(display).unwrap();
    let mut led_controller = LedController::new(ws, sin);

    let mut s: String<128> = String::new();
    write!(s, "Rate: {}", sample_rate).unwrap();
    screen_controller.write(&s).unwrap();

    let mut t = 0.0;

    // Slow down timer by this factor (0.1 will result in 10 seconds):
    let animation_speed = 0.1;

    let mut wave_idx = 0;

    let mut led_next_frame = Instant::from_ticks(0);

    let encoder = pins.button.into_pull_up_input();
    loop {
        let now = timer.get_counter();

        if encoder.is_low().unwrap() && now > led_next_frame {
            led_controller.next_frame(t);
            t += (16.0 / 1000.0) * animation_speed;
            while t > 1.0 {
                t -= 1.0;
            }
            led_next_frame = now + 16u64.millis();
        }

        if pwm.has_overflown() {
            pwm.clear_interrupt();

            if key_1.is_low().unwrap() {
                pwm.channel_a.set_duty(SIN_128[wave_idx]);
            } else if key_2.is_low().unwrap() {
                pwm.channel_a.set_duty(SAW_128[wave_idx]);
            } else if key_3.is_low().unwrap() {
                pwm.channel_a.set_duty(TRIANGLE_128[wave_idx]);
            } else if key_4.is_low().unwrap() {
                pwm.channel_a.set_duty(if wave_idx < SIN_128.len() / 2 {
                    0
                } else {
                    PWM_MAX
                });
            } else {
                pwm.channel_a.set_duty(PWM_MAX / 2);
            }

            wave_idx += 4;
            if wave_idx >= SIN_128.len() {
                wave_idx = 0;
            }
        }
    }
}

const SIN_128: [u16; 128] = [
    2048, 2148, 2249, 2349, 2448, 2546, 2643, 2738, 2832, 2924, 3013, 3101, 3186, 3268, 3347, 3423,
    3496, 3565, 3631, 3693, 3751, 3805, 3854, 3899, 3940, 3976, 4008, 4035, 4057, 4074, 4086, 4094,
    4096, 4094, 4086, 4074, 4057, 4035, 4008, 3976, 3940, 3899, 3854, 3805, 3751, 3693, 3631, 3565,
    3496, 3423, 3347, 3268, 3186, 3101, 3013, 2924, 2832, 2738, 2643, 2546, 2448, 2349, 2249, 2148,
    2048, 1948, 1847, 1747, 1648, 1550, 1453, 1358, 1264, 1172, 1083, 995, 910, 828, 749, 673, 600,
    531, 465, 403, 345, 291, 242, 197, 156, 120, 88, 61, 39, 22, 10, 2, 0, 2, 10, 22, 39, 61, 88,
    120, 156, 197, 242, 291, 345, 403, 465, 531, 600, 673, 749, 828, 910, 995, 1083, 1172, 1264,
    1358, 1453, 1550, 1648, 1747, 1847, 1948,
];

const SAW_128: [u16; 128] = [
    0, 32, 64, 96, 128, 160, 192, 224, 256, 288, 320, 352, 384, 416, 448, 480, 512, 544, 576, 608,
    640, 672, 704, 736, 768, 800, 832, 864, 896, 928, 960, 992, 1024, 1056, 1088, 1120, 1152, 1184,
    1216, 1248, 1280, 1312, 1344, 1376, 1408, 1440, 1472, 1504, 1536, 1568, 1600, 1632, 1664, 1696,
    1728, 1760, 1792, 1824, 1856, 1888, 1920, 1952, 1984, 2016, 2048, 2080, 2112, 2144, 2176, 2208,
    2240, 2272, 2304, 2336, 2368, 2400, 2432, 2464, 2496, 2528, 2560, 2592, 2624, 2656, 2688, 2720,
    2752, 2784, 2816, 2848, 2880, 2912, 2944, 2976, 3008, 3040, 3072, 3104, 3136, 3168, 3200, 3232,
    3264, 3296, 3328, 3360, 3392, 3424, 3456, 3488, 3520, 3552, 3584, 3616, 3648, 3680, 3712, 3744,
    3776, 3808, 3840, 3872, 3904, 3936, 3968, 4000, 4032, 4064,
];
const TRIANGLE_128: [u16; 128] = [
    0, 64, 128, 192, 256, 320, 384, 448, 512, 576, 640, 704, 768, 832, 896, 960, 1024, 1088, 1152,
    1216, 1280, 1344, 1408, 1472, 1536, 1600, 1664, 1728, 1792, 1856, 1920, 1984, 2048, 2112, 2176,
    2240, 2304, 2368, 2432, 2496, 2560, 2624, 2688, 2752, 2816, 2880, 2944, 3008, 3072, 3136, 3200,
    3264, 3328, 3392, 3456, 3520, 3584, 3648, 3712, 3776, 3840, 3904, 3968, 4032, 4096, 4032, 3968,
    3904, 3840, 3776, 3712, 3648, 3584, 3520, 3456, 3392, 3328, 3264, 3200, 3136, 3072, 3008, 2944,
    2880, 2816, 2752, 2688, 2624, 2560, 2496, 2432, 2368, 2304, 2240, 2176, 2112, 2048, 1984, 1920,
    1856, 1792, 1728, 1664, 1600, 1536, 1472, 1408, 1344, 1280, 1216, 1152, 1088, 1024, 960, 896,
    832, 768, 704, 640, 576, 512, 448, 384, 320, 256, 192, 128, 64,
];
