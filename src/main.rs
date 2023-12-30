#![no_std]
#![no_main]

use adafruit_macropad as bsp;
use bsp::{
    entry,
    hal::{
        self,
        clocks::{init_clocks_and_plls, Clock},
        dma::{double_buffer, DMAExt, SingleChannel},
        fugit::{ExtU64, MicrosDurationU64},
        gpio::{
            bank0::{
                Gpio0, Gpio1, Gpio10, Gpio11, Gpio12, Gpio17, Gpio18, Gpio2, Gpio23, Gpio3, Gpio4,
                Gpio5, Gpio6, Gpio7, Gpio8, Gpio9,
            },
            FunctionSio, FunctionSioOutput, FunctionSpi, Pin, PinGroup, PullDown, PullNone, PullUp,
            SioInput,
        },
        pac::{self},
        pio::PIOExt,
        pwm::{CcFormat, SliceDmaWrite},
        sio::Sio,
        timer::Instant,
        watchdog::Watchdog,
        Timer,
    },
    pac::interrupt,
};
use core::{
    cell::RefCell,
    fmt::{Debug, Write},
};
use cortex_m::{
    asm::wfi,
    peripheral::{syst::SystClkSource, SYST},
    singleton,
};
use cortex_m_rt::exception;
use critical_section::Mutex;
use display::Screen;
use embedded_hal::digital::v2::OutputPin;
use frunk_core::hlist::{HCons, HNil};
use hal::fugit::RateExtU32;
use heapless::{spsc::Queue, String};
use leds::LedController;
use panic_persist as _;
use portable_atomic::{AtomicU16, AtomicU32, Ordering};
use sh1106::{interface::DisplayInterface, mode::GraphicsMode, Builder};
use synth::Synth;
use ws2812_pio::Ws2812;

mod display;
mod leds;
mod synth;

type InputPinGroup = PinGroup<
    HCons<
        Pin<Gpio18, FunctionSio<SioInput>, PullUp>,
        HCons<
            Pin<Gpio17, FunctionSio<SioInput>, PullUp>,
            HCons<
                Pin<Gpio0, FunctionSio<SioInput>, PullUp>,
                HCons<
                    Pin<Gpio12, FunctionSio<SioInput>, PullUp>,
                    HCons<
                        Pin<Gpio11, FunctionSio<SioInput>, PullUp>,
                        HCons<
                            Pin<Gpio10, FunctionSio<SioInput>, PullUp>,
                            HCons<
                                Pin<Gpio9, FunctionSio<SioInput>, PullUp>,
                                HCons<
                                    Pin<Gpio8, FunctionSio<SioInput>, PullUp>,
                                    HCons<
                                        Pin<Gpio7, FunctionSio<SioInput>, PullUp>,
                                        HCons<
                                            Pin<Gpio6, FunctionSio<SioInput>, PullUp>,
                                            HCons<
                                                Pin<Gpio5, FunctionSio<SioInput>, PullUp>,
                                                HCons<
                                                    Pin<Gpio4, FunctionSio<SioInput>, PullUp>,
                                                    HCons<
                                                        Pin<Gpio3, FunctionSio<SioInput>, PullUp>,
                                                        HCons<
                                                            Pin<
                                                                Gpio2,
                                                                FunctionSio<SioInput>,
                                                                PullUp,
                                                            >,
                                                            HCons<
                                                                Pin<
                                                                    Gpio1,
                                                                    FunctionSio<SioInput>,
                                                                    PullUp,
                                                                >,
                                                                HNil,
                                                            >,
                                                        >,
                                                    >,
                                                >,
                                            >,
                                        >,
                                    >,
                                >,
                            >,
                        >,
                    >,
                >,
            >,
        >,
    >,
>;

const PWM_SAMPLE_LEN: usize = 220;

type PwmDmaTransfer = double_buffer::Transfer<
    hal::dma::Channel<hal::dma::CH0>,
    hal::dma::Channel<hal::dma::CH1>,
    &'static mut [CcFormat; PWM_SAMPLE_LEN],
    hal::pwm::SliceDmaWriteCc<hal::pwm::Pwm0, hal::pwm::FreeRunning>,
    double_buffer::ReadNext<&'static mut [CcFormat; PWM_SAMPLE_LEN]>,
>;

static SHARED_STATE: Mutex<RefCell<Option<SharedState>>> = Mutex::new(RefCell::new(None));
static ATOMIC_STATE: AtomicState = AtomicState::new();

const PWM_MAX: u16 = 4096;

struct SharedState {
    input_pin_group: Option<InputPinGroup>,
    pwm_dma_transfer: Option<PwmDmaTransfer>,
}

impl SharedState {
    fn new(input_pin_group: InputPinGroup, pwm_dma_transfer: PwmDmaTransfer) -> Self {
        Self {
            input_pin_group: Some(input_pin_group),
            pwm_dma_transfer: Some(pwm_dma_transfer),
        }
    }
}
struct AtomicState {
    input_values: AtomicU32,
    wave_type: AtomicU16,
}

impl AtomicState {
    pub const fn new() -> Self {
        Self {
            input_values: AtomicU32::new(u32::MAX),
            wave_type: AtomicU16::new(0),
        }
    }
}

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);
    let mut core = pac::CorePeripherals::take().unwrap();
    core.SYST.set_clock_source(SystClkSource::Core);
    let get_ticks_per_1ms = (SYST::get_ticks_per_10ms() + 1) - 1;
    core.SYST.set_reload(get_ticks_per_1ms);
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
    pwm.enable();

    let pwm_buf0 = singleton!(: [CcFormat; PWM_SAMPLE_LEN] = [CcFormat{a: PWM_MAX/2, b: PWM_MAX/2}; PWM_SAMPLE_LEN]).unwrap();
    let pwm_buf1 = singleton!(: [CcFormat; PWM_SAMPLE_LEN] = [CcFormat{a: PWM_MAX/2, b: PWM_MAX/2}; PWM_SAMPLE_LEN]).unwrap();

    let mut dma = pac.DMA.split(&mut pac.RESETS);
    let dma_pwm = SliceDmaWrite::from(pwm);
    dma.ch0.enable_irq0();
    dma.ch1.enable_irq0();
    let dma_pwm_conf = double_buffer::Config::new((dma.ch0, dma.ch1), pwm_buf0, dma_pwm.cc);

    pins.speaker_shutdown
        .into_push_pull_output()
        .set_high()
        .unwrap();

    oled_reset.set_high().unwrap();

    let input_pins = PinGroup::new();
    let input_pins = input_pins.add_pin(pins.key1.into_pull_up_input());
    let input_pins = input_pins.add_pin(pins.key2.into_pull_up_input());
    let input_pins = input_pins.add_pin(pins.key3.into_pull_up_input());
    let input_pins = input_pins.add_pin(pins.key4.into_pull_up_input());
    let input_pins = input_pins.add_pin(pins.key5.into_pull_up_input());
    let input_pins = input_pins.add_pin(pins.key6.into_pull_up_input());
    let input_pins = input_pins.add_pin(pins.key7.into_pull_up_input());
    let input_pins = input_pins.add_pin(pins.key8.into_pull_up_input());
    let input_pins = input_pins.add_pin(pins.key9.into_pull_up_input());
    let input_pins = input_pins.add_pin(pins.key10.into_pull_up_input());
    let input_pins = input_pins.add_pin(pins.key11.into_pull_up_input());
    let input_pins = input_pins.add_pin(pins.key12.into_pull_up_input());
    let input_pins = input_pins.add_pin(pins.button.into_pull_up_input());
    let input_pins = input_pins.add_pin(pins.encoder_rota.into_pull_up_input());
    let input_pins = input_pins.add_pin(pins.encoder_rotb.into_pull_up_input());

    let mut screen_controller = Screen::new(display).unwrap();
    let mut led_controller = LedController::new(ws, sin);

    // Start DMA
    let pwm_transfer = dma_pwm_conf.start().read_next(pwm_buf1);

    // Set shared state
    let shared_state = SharedState::new(input_pins, pwm_transfer);
    critical_section::with(|cs| {
        *SHARED_STATE.borrow(cs).borrow_mut() = Some(shared_state);
    });
    // Enable interrupts
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::DMA_IRQ_0);
    }

    core.SYST.enable_interrupt();

    let animation_speed = 0.1;
    let mut t = 0.0;

    let mut next_frame = Instant::from_ticks(0);

    let mut screen = MicrosDurationU64::from_ticks(0);

    loop {
        let now = timer.get_counter();
        let input = ATOMIC_STATE
            .input_values
            .load(portable_atomic::Ordering::Relaxed);

        if now > next_frame {
            led_controller.next_frame(t);
            t += (16.0 / 1000.0) * animation_speed;
            while t > 1.0 {
                t -= 1.0;
            }
            let led = timer.get_counter();

            let mut s: String<128> = String::new();
            write!(s, "Input: {:019b}\nScreen: {}", input, screen,).unwrap();
            screen_controller.write(&s).unwrap();
            next_frame = now + 16u64.millis();

            screen = timer.get_counter() - led;
        }

        // wait for the next timer tick
        wfi()
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
            wfi();
        }
    } else {
        display
    }
}

/*
 * # TODO
 *
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
 * - Move synth to 2nd core?
 */

// #[interrupt]
// fn IO_IRQ_BANK0() {
//     static mut ENCODER_LAST: (bool, bool) = (false, false);

//     if let Some((enc_rot_a, enc_rot_b)) = ENCODER_PINS {
//         let (a_last, b_last) = *ENCODER_LAST;
//         let a_now = enc_rot_a.is_low().unwrap();
//         let b_now = enc_rot_b.is_low().unwrap();
//         let change = match (a_last, b_last, a_now, b_now) {
//             (false, false, false, true)
//             | (false, true, true, true)
//             | (true, false, false, false)
//             | (true, true, true, false) => 1,
//             (false, false, true, false)
//             | (false, true, false, false)
//             | (true, false, true, true)
//             | (true, true, false, true) => -1,
//             _ => 0,
//         };
//         ENCODER_VALUE.add(change, core::sync::atomic::Ordering::SeqCst);

//         *ENCODER_LAST = (a_now, b_now);

//         enc_rot_a.clear_interrupt(Interrupt::EdgeHigh);
//         enc_rot_a.clear_interrupt(Interrupt::EdgeLow);
//         enc_rot_b.clear_interrupt(Interrupt::EdgeHigh);
//         enc_rot_b.clear_interrupt(Interrupt::EdgeLow);
//     }
// }

#[exception]
fn SysTick() {
    static mut INPUT_PIN_GROUP: Option<InputPinGroup> = None;
    static mut INPUT_HISTORY: Queue<u32, 8> = Queue::new();

    if INPUT_PIN_GROUP.is_none() {
        critical_section::with(|cs| {
            *INPUT_PIN_GROUP = SHARED_STATE
                .borrow(cs)
                .borrow_mut()
                .as_mut()
                .map(|s| s.input_pin_group.take())
                .unwrap_or_default();
        });
    }

    if let Some(input_pin_group) = INPUT_PIN_GROUP {
        let input = input_pin_group.read();
        while INPUT_HISTORY.is_full() {
            INPUT_HISTORY.dequeue();
        }
        INPUT_HISTORY.enqueue(input).unwrap();

        ATOMIC_STATE.input_values.store(
            INPUT_HISTORY.iter().fold(u32::MAX, |a, &i| a & i),
            Ordering::Relaxed,
        );
    }
}

#[interrupt]
fn DMA_IRQ_0() {
    static mut PWM_DMA_TRANSFER: Option<PwmDmaTransfer> = None;
    static mut SYNTH: Synth = Synth::new();

    if PWM_DMA_TRANSFER.is_none() {
        *PWM_DMA_TRANSFER = critical_section::with(|cs| {
            SHARED_STATE
                .borrow(cs)
                .borrow_mut()
                .as_mut()
                .map(|s| s.pwm_dma_transfer.take())
                .unwrap_or_default()
        });
    }

    if let Some(mut transfer) = PWM_DMA_TRANSFER.take() {
        transfer.check_irq0();
        let (buf, transfer) = transfer.wait();
        let input = ATOMIC_STATE.input_values.load(Ordering::Relaxed);
        let wave_type = match input {
            i if (i >> 1 & 1) == 0 => 262,
            i if (i >> 2 & 1) == 0 => 294,
            i if (i >> 3 & 1) == 0 => 330,
            i if (i >> 4 & 1) == 0 => 349,
            i if (i >> 5 & 1) == 0 => 392,
            i if (i >> 6 & 1) == 0 => 440,
            i if (i >> 7 & 1) == 0 => 494,
            i if (i >> 8 & 1) == 0 => 523,
            i if (i >> 9 & 1) == 0 => 587,
            i if (i >> 10 & 1) == 0 => 659,
            i if (i >> 11 & 1) == 0 => 699,
            i if (i >> 12 & 1) == 0 => 784,
            _ => 0,
        };

        for s in buf.iter_mut() {
            s.a = SYNTH.next_sample(wave_type, 0)
        }

        ATOMIC_STATE.wave_type.store(wave_type, Ordering::Relaxed);
        *PWM_DMA_TRANSFER = Some(transfer.read_next(buf));
    }
}
