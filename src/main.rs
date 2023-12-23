#![no_std]
#![no_main]

use adafruit_macropad as bsp;
use bsp::{
    entry,
    hal::{
        self,
        clocks::{init_clocks_and_plls, Clock},
        fugit::ExtU64,
        gpio::{FunctionSpi, PullNone, PullUp},
        pac::{self, interrupt},
        pio::PIOExt,
        sio::Sio,
        timer::Instant,
        watchdog::Watchdog,
        Timer,
    },
    pac::rosc::PHASE,
};
use core::{cell::Cell, fmt::Write, sync::atomic::AtomicU16};
use cortex_m::asm::wfe;
use critical_section::Mutex;
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

static WAVE_TYPE: AtomicU16 = AtomicU16::new(0);
static PWM: Mutex<Cell<Option<hal::pwm::Slice<hal::pwm::Pwm0, hal::pwm::FreeRunning>>>> =
    Mutex::new(Cell::new(None));

const PWM_MAX: u16 = 4096;

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
    const PWM_DIV_FRAC: u8 = 6;
    // let sample_rate = (clocks.system_clock.freq().to_Hz() as f32
    //     / (PWM_MAX as f32 * (1.0 + PWM_DIV_FRAC as f32 / 16.0))) as u16;
    pwm.set_top(PWM_MAX);
    pwm.set_div_frac(6);
    pwm.enable_interrupt();
    pwm.enable();

    critical_section::with(|cs| {
        PWM.borrow(cs).replace(Some(pwm));
    });

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::PWM_IRQ_WRAP);
    }

    pins.speaker_shutdown
        .into_push_pull_output()
        .set_high()
        .unwrap();

    pins.oled_reset.into_push_pull_output().set_high().unwrap();

    let key_1 = pins.key1.into_pull_up_input();
    let key_2 = pins.key2.into_pull_up_input();
    let key_3 = pins.key3.into_pull_up_input();
    let key_4 = pins.key4.into_pull_up_input();
    let key_5 = pins.key5.into_pull_up_input();
    let key_6 = pins.key6.into_pull_up_input();
    let key_7 = pins.key7.into_pull_up_input();
    let key_8 = pins.key8.into_pull_up_input();
    let key_9 = pins.key9.into_pull_up_input();
    let key_10 = pins.key10.into_pull_up_input();
    let key_11 = pins.key11.into_pull_up_input();
    let key_12 = pins.key12.into_pull_up_input();
    let encoder = pins.button.into_pull_up_input();

    let mut screen_controller = Screen::new(display).unwrap();
    let mut led_controller = LedController::new(ws, sin);

    // Slow down timer by this factor (0.1 will result in 10 seconds):
    let animation_speed = 0.1;
    let mut t = 0.0;

    let mut next_frame = Instant::from_ticks(0);
    loop {
        let now = timer.get_counter();

        if now > next_frame {
            if encoder.is_low().unwrap() {
                led_controller.next_frame(t);
                t += (16.0 / 1000.0) * animation_speed;
                while t > 1.0 {
                    t -= 1.0;
                }
                next_frame = now + 16u64.millis();
            }

            if key_1.is_low().unwrap() {
                WAVE_TYPE.store(262, core::sync::atomic::Ordering::Relaxed);
            } else if key_2.is_low().unwrap() {
                WAVE_TYPE.store(294, core::sync::atomic::Ordering::Relaxed);
            } else if key_3.is_low().unwrap() {
                WAVE_TYPE.store(330, core::sync::atomic::Ordering::Relaxed);
            } else if key_4.is_low().unwrap() {
                WAVE_TYPE.store(349, core::sync::atomic::Ordering::Relaxed);
            } else if key_5.is_low().unwrap() {
                WAVE_TYPE.store(392, core::sync::atomic::Ordering::Relaxed);
            } else if key_6.is_low().unwrap() {
                WAVE_TYPE.store(440, core::sync::atomic::Ordering::Relaxed);
            } else if key_7.is_low().unwrap() {
                WAVE_TYPE.store(494, core::sync::atomic::Ordering::Relaxed);
            } else if key_8.is_low().unwrap() {
                WAVE_TYPE.store(523, core::sync::atomic::Ordering::Relaxed);
            } else if key_9.is_low().unwrap() {
                WAVE_TYPE.store(587, core::sync::atomic::Ordering::Relaxed);
            } else if key_10.is_low().unwrap() {
                WAVE_TYPE.store(659, core::sync::atomic::Ordering::Relaxed);
            } else if key_11.is_low().unwrap() {
                WAVE_TYPE.store(699, core::sync::atomic::Ordering::Relaxed);
            } else if key_12.is_low().unwrap() {
                WAVE_TYPE.store(784, core::sync::atomic::Ordering::Relaxed);
            } else {
                WAVE_TYPE.store(0, core::sync::atomic::Ordering::Relaxed);
            }

            let mut s: String<128> = String::new();
            write!(
                s,
                "Frequency: {} Hz",
                WAVE_TYPE.load(core::sync::atomic::Ordering::Relaxed)
            )
            .unwrap();
            screen_controller.write(&s).unwrap();
        }

        // PWM will fire regularly, might as well sleep
        wfe()
    }
}

#[interrupt]
fn PWM_IRQ_WRAP() {
    static mut IRQ_PWM: Option<hal::pwm::Slice<hal::pwm::Pwm0, hal::pwm::FreeRunning>> = None;
    static mut PHASE: f32 = 0.0; // 0 to 128

    if IRQ_PWM.is_none() {
        critical_section::with(|cs| {
            *IRQ_PWM = PWM.borrow(cs).take();
        });
    }

    if let Some(pwm) = IRQ_PWM {
        pwm.clear_interrupt();

        let wave_type = WAVE_TYPE.load(core::sync::atomic::Ordering::Relaxed);

        match wave_type {
            0 => pwm.channel_a.set_duty(PWM_MAX / 2),
            _ => {
                *PHASE += 128.0 / 22000.0 * wave_type as f32;
                while *PHASE > 128.0 {
                    *PHASE -= 128.0;
                }
                pwm.channel_a.set_duty(SIN_128[*PHASE as usize])
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
