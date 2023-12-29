#![no_std]
#![no_main]

use adafruit_macropad as bsp;
use bsp::{
    entry,
    hal::{
        self,
        clocks::{init_clocks_and_plls, Clock},
        fugit::ExtU64,
        gpio::{
            bank0::{Gpio17, Gpio18, Gpio23},
            DynPinId, FunctionSio, FunctionSioOutput, FunctionSpi, Interrupt, Pin, PullDown,
            PullNone, PullUp, SioInput,
        },
        pac::{self, interrupt},
        pio::PIOExt,
        pwm::Pwm0,
        sio::Sio,
        timer::Instant,
        watchdog::Watchdog,
        Timer,
    },
};
use core::{
    cell::Cell,
    fmt::{Debug, Write},
};
use cortex_m::{
    asm::wfe,
    peripheral::{syst::SystClkSource, SYST},
};
use cortex_m_rt::exception;
use critical_section::Mutex;
use display::Screen;
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use hal::fugit::RateExtU32;
use heapless::String;
use leds::LedController;
use panic_persist as _;
use portable_atomic::{AtomicI32, AtomicU16};
use sh1106::{interface::DisplayInterface, mode::GraphicsMode, Builder};
use synth::Synth;
use ws2812_pio::Ws2812;

mod display;
mod leds;
mod synth;

type EncoderRotAPin = Pin<Gpio17, FunctionSio<SioInput>, PullUp>;
type EncoderRotBPin = Pin<Gpio18, FunctionSio<SioInput>, PullUp>;

static WAVE_TYPE: AtomicU16 = AtomicU16::new(0);
static ENCODER_VALUE: AtomicI32 = AtomicI32::new(0);
static IRQ_SYNTH: Mutex<Cell<Option<Synth<Pwm0>>>> = Mutex::new(Cell::new(None));
static IRQ_ENCODER_PINS: Mutex<Cell<Option<(EncoderRotAPin, EncoderRotBPin)>>> =
    Mutex::new(Cell::new(None));

const PWM_MAX: u16 = 4096;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);
    let mut core = pac::CorePeripherals::take().unwrap();
    core.SYST.set_clock_source(SystClkSource::Core);
    core.SYST.set_reload(SYST::get_ticks_per_10ms());
    core.SYST.clear_current();
    core.SYST.enable_counter();

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

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let spi = hal::spi::Spi::<_, _, _>::new(
        pac.SPI1,
        (
            pins.mosi.reconfigure::<FunctionSpi, PullNone>(),
            pins.miso.reconfigure::<FunctionSpi, PullUp>(),
            pins.sclk.reconfigure::<FunctionSpi, PullNone>(),
        ),
    )
    .init(
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

    let mut oled_reset = pins.oled_reset.into_push_pull_output();
    let display = check_for_panic(&mut oled_reset, display);

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let sin = hal::rom_data::float_funcs::fsin::ptr();

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let ws = Ws2812::new(
        pins.neopixel.into_function(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    let pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);
    let mut pwm = pwm_slices.pwm0;
    pwm.channel_a.output_to(pins.speaker);
    // 12 bit 22KHz sample output
    const PWM_DIV_FRAC: u8 = 6;
    // let sample_rate = (clocks.system_clock.freq().to_Hz() as f32
    //     / (PWM_MAX as f32 * (1.0 + PWM_DIV_FRAC as f32 / 16.0))) as u16;
    pwm.set_top(PWM_MAX);
    pwm.set_div_frac(PWM_DIV_FRAC);
    pwm.enable_interrupt();
    pwm.enable();

    critical_section::with(|cs| {
        IRQ_SYNTH.borrow(cs).set(Some(Synth::new(pwm)));
    });

    pins.speaker_shutdown
        .into_push_pull_output()
        .set_high()
        .unwrap();

    oled_reset.set_high().unwrap();

    let key1 = pins.key1.into_pull_up_input();
    let key2 = pins.key2.into_pull_up_input();
    let key3 = pins.key3.into_pull_up_input();
    let key4 = pins.key4.into_pull_up_input();
    let key5 = pins.key5.into_pull_up_input();
    let key6 = pins.key6.into_pull_up_input();
    let key7 = pins.key7.into_pull_up_input();
    let key8 = pins.key8.into_pull_up_input();
    let key9 = pins.key9.into_pull_up_input();
    let key10 = pins.key10.into_pull_up_input();
    let key11 = pins.key11.into_pull_up_input();
    let key12 = pins.key12.into_pull_up_input();
    let button = pins.button.into_pull_up_input();
    let encoder_rota = pins.encoder_rota.into_pull_up_input();
    let encoder_rotb = pins.encoder_rotb.into_pull_up_input();

    critical_section::with(|cs| {
        IRQ_ENCODER_PINS
            .borrow(cs)
            .set(Some((encoder_rota, encoder_rotb)));
    });

    let mut screen_controller = Screen::new(display).unwrap();
    let mut led_controller = LedController::new(ws, sin);

    // Enable interrupts
    core.SYST.enable_interrupt();
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::PWM_IRQ_WRAP);
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }

    // Slow down timer by this factor (0.1 will result in 10 seconds):
    let animation_speed = 0.1;
    let mut t = 0.0;

    let mut next_frame = Instant::from_ticks(0);
    loop {
        let now = timer.get_counter();
        if now > next_frame {
            if button.is_low().unwrap() {
                led_controller.next_frame(t);
                t += (16.0 / 1000.0) * animation_speed;
                while t > 1.0 {
                    t -= 1.0;
                }
                next_frame = now + 16u64.millis();
            }

            if key1.is_low().unwrap() {
                WAVE_TYPE.store(262, core::sync::atomic::Ordering::Relaxed);
            } else if key2.is_low().unwrap() {
                WAVE_TYPE.store(294, core::sync::atomic::Ordering::Relaxed);
            } else if key3.is_low().unwrap() {
                WAVE_TYPE.store(330, core::sync::atomic::Ordering::Relaxed);
            } else if key4.is_low().unwrap() {
                WAVE_TYPE.store(349, core::sync::atomic::Ordering::Relaxed);
            } else if key5.is_low().unwrap() {
                WAVE_TYPE.store(392, core::sync::atomic::Ordering::Relaxed);
            } else if key6.is_low().unwrap() {
                WAVE_TYPE.store(440, core::sync::atomic::Ordering::Relaxed);
            } else if key7.is_low().unwrap() {
                WAVE_TYPE.store(494, core::sync::atomic::Ordering::Relaxed);
            } else if key8.is_low().unwrap() {
                WAVE_TYPE.store(523, core::sync::atomic::Ordering::Relaxed);
            } else if key9.is_low().unwrap() {
                WAVE_TYPE.store(587, core::sync::atomic::Ordering::Relaxed);
            } else if key10.is_low().unwrap() {
                WAVE_TYPE.store(659, core::sync::atomic::Ordering::Relaxed);
            } else if key11.is_low().unwrap() {
                WAVE_TYPE.store(699, core::sync::atomic::Ordering::Relaxed);
            } else if key12.is_low().unwrap() {
                WAVE_TYPE.store(784, core::sync::atomic::Ordering::Relaxed);
            } else {
                WAVE_TYPE.store(0, core::sync::atomic::Ordering::Relaxed);
            }

            let mut s: String<128> = String::new();
            write!(
                s,
                "Frequency: {} Hz\nEncoder: {}",
                WAVE_TYPE.load(core::sync::atomic::Ordering::Relaxed),
                ENCODER_VALUE.load(core::sync::atomic::Ordering::Relaxed),
            )
            .unwrap();
            screen_controller.write(&s).unwrap();
        }

        // PWM will fire regularly, might as well sleep
        wfe()
    }
}

fn check_for_panic<DI>(
    pin: &mut Pin<Gpio23, FunctionSioOutput, PullDown>,
    display: GraphicsMode<DI>,
) -> GraphicsMode<DI>
where
    <DI as DisplayInterface>::Error: Debug,
    DI: DisplayInterface,
{
    if let Some(panic) = panic_persist::get_panic_message_utf8() {
        pin.set_high().unwrap();
        let mut screen_controller = Screen::new(display).unwrap();

        screen_controller.write(panic).unwrap();
        loop {
            wfe();
        }
    } else {
        display
    }
}

#[interrupt]
fn PWM_IRQ_WRAP() {
    static mut SYNTH: Option<Synth<Pwm0>> = None;

    if SYNTH.is_none() {
        critical_section::with(|cs| {
            *SYNTH = IRQ_SYNTH.borrow(cs).take();
        });
    }

    if let Some(synth) = SYNTH {
        synth.next_sample(
            WAVE_TYPE.load(core::sync::atomic::Ordering::Relaxed),
            ENCODER_VALUE.load(core::sync::atomic::Ordering::Relaxed),
        );
    }

    /*
     * # TODO
     *
     * - Feed pwm via DMA
     * - Feed oled? and leds via DMA
     * - Read encoder knob
     * - Move synth code to fixed point arithmetic rather than float
     * - Synth features
     *   - Envelopes
     *   - Filters
     *   - Multiple oscillators
     *   - Mix sources
     *   - Amplitude modulation
     *   - Frequency modulation
     *   - Multiple channels
     * - Nicer led visualization and key feedback
     * - Mute speaker when not sounding
     * - Basic sequencer
     * - Edit sound parameters with GUI, encoder and buttons
     */
}

#[interrupt]
fn IO_IRQ_BANK0() {
    static mut ENCODER_PINS: Option<(EncoderRotAPin, EncoderRotBPin)> = None;
    static mut ENCODER_LAST: (bool, bool) = (false, false);

    if ENCODER_PINS.is_none() {
        critical_section::with(|cs| {
            *ENCODER_PINS = IRQ_ENCODER_PINS.borrow(cs).take();
        });
    }

    if let Some((enc_rot_a, enc_rot_b)) = ENCODER_PINS {
        let (a_last, b_last) = *ENCODER_LAST;
        let a_now = enc_rot_a.is_low().unwrap();
        let b_now = enc_rot_b.is_low().unwrap();
        let change = match (a_last, b_last, a_now, b_now) {
            (false, false, false, true)
            | (false, true, true, true)
            | (true, false, false, false)
            | (true, true, true, false) => 1,
            (false, false, true, false)
            | (false, true, false, false)
            | (true, false, true, true)
            | (true, true, false, true) => -1,
            _ => 0,
        };
        ENCODER_VALUE.add(change, core::sync::atomic::Ordering::SeqCst);

        *ENCODER_LAST = (a_now, b_now);

        enc_rot_a.clear_interrupt(Interrupt::EdgeHigh);
        enc_rot_a.clear_interrupt(Interrupt::EdgeLow);
        enc_rot_b.clear_interrupt(Interrupt::EdgeHigh);
        enc_rot_b.clear_interrupt(Interrupt::EdgeLow);
    }
}

#[exception]
fn SysTick() {
    // Update buttons

    // Update encoder
}

struct Buttons<const N: usize> {
    button_pins: [Pin<DynPinId, FunctionSioOutput, PullUp>; N],
}

impl<const N: usize> Buttons<N> {
    fn get_buttons(&self) -> [bool; N] {
        todo!()
    }
}

struct Encoder {
    pin_a: Pin<DynPinId, FunctionSioOutput, PullUp>,
    pin_b: Pin<DynPinId, FunctionSioOutput, PullUp>,
}

impl Encoder {
    fn get_encoder(&self) -> EncoderValue {
        todo!()
    }
}
enum EncoderValue {
    Clockwise,
    Anticlockwise,
    None,
}
