#![no_std]
#![no_main]

use core::{cell::RefCell, fmt::{self, Write}, f64::consts::PI};
use cortex_m::{
    interrupt::Mutex,
    peripheral::NVIC,
};
use embedded_hal::{delay::DelayNs, digital::OutputPin, spi::MODE_0};
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::{FONT_10X20, FONT_6X10}},
    pixelcolor::Rgb565, prelude::*, text::Text,
    primitives::{Circle, Line, PrimitiveStyle, Rectangle},
};
use fugit::RateExtU32;
use libm as math;
use mipidsi::{interface::SpiInterface, models::ST7789,
    options::{ColorInversion, ColorOrder, Orientation, Rotation}, Builder};
use rp235x_hal::{self as hal, clocks::{self, Clock},
    gpio::{bank0::Gpio32, FunctionSio, FunctionSpi, Interrupt as GpioInterrupt,
        Pin, PinState, PullUp, SioInput},
    pac, spi::Spi, timer::CopyableTimer0};
use hal::pac::interrupt;
use panic_halt as _;

/// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

const XTAL_FREQ_HZ: u32 = 12_000_000u32; // 12.0 Mhz

// CPU overclock
// python3 vcocalc.py 200
// Requested: 200.0 MHz
// Achieved:  200.0 MHz
// REFDIV:    1
// FBDIV:     100 (VCO = 1200.0 MHz)
// PD1:       6
// PD2:       1
const REFDIV: u8 = 1;
const POST_DIVIDER_1: u8 = 6;
const POST_DIVIDER_2: u8 = 1;
const VCO_FREQ: u32 = 1200;

// This is output for the usb clock pll settings
// $ ./vcocalc.py 48
// Requested: 48.0 MHz
// Achieved:  48.0 MHz
// REFDIV:    1
// FBDIV:     120 (VCO = 1440.0 MHz)
// PD1:       6
// PD2:       5
const USB_REFDIV: u8 = 1;
const USB_POST_DIVIDER_1: u8 = 6;
const USB_POST_DIVIDER_2: u8 = 5;
const USB_VCO_FREQ: u32 = 1440;

// speed should be mapped in the range 0-161 KPH. I don't think faster is required on this bike...
const V_MIN: f64 = 0.0;
const V_MAX: f64 = 161.0;
// the speed gague should be set with theta -225 to 0
const THETA_MIN: f64 = 3.0 * PI / 4.0 - 2.0 * PI;
const THETA_MAX: f64 = 0.0;
// rpm should be mapped in the range 0-10,000 RPM. It SHOULDN'T go higher than this or else I have real problems >_>
const RPM_MIN: f64 = 0.0;
const RPM_MAX: f64 = 10000.0;

// Dial circle position is the top-left of its bounding box.
const RADIUS: u32 = 64;
const X_CENTER: i32 = 110;
const Y_CENTER: i32 = 114;
// Leave space between the needle tip and the dial outline.
const NEEDLE_LENGTH: f64 = RADIUS as f64 - 12.0; // Clearance from tick strokes.
const RPM_BAR_MAX_WIDTH: u32 = 280;
const RPM_INNER_WIDTH: u32 = RPM_BAR_MAX_WIDTH - 2;
const RPM_OUTER_X: u32 = 30;
const RPM_INNER_X: u32 = RPM_OUTER_X + 1;
const RPM_INNER_Y: u32 = 230;
const RPM_INNER_SIZE_HEIGHT: u32 = 8;
const SPEED_X_OFFSET: i32 = 80;
const SPEED_Y_OFFSET: i32 = 0;
const SPEED_TEXT_X: i32 = X_CENTER + SPEED_X_OFFSET;
const SPEED_TEXT_Y: i32 = Y_CENTER + SPEED_Y_OFFSET;

const WHEEL_CIRCUMFERENCE_M: f64 = 2.16; // Calibrate with a loaded wheel rollout.
const WHEEL_PULSES_PER_REV: f64 = 4.0; // Use 1.0 for one magnet per revolution.
// Explicit minimum speed used to derive a pulse timeout (~1.944 s).
const MIN_SPEED_KMH: f64 = 1.0;
const SPEED_TIMEOUT_US: u64 = (3_600_000.0 * WHEEL_CIRCUMFERENCE_M
    / (WHEEL_PULSES_PER_REV * MIN_SPEED_KMH)) as u64;
const DISPLAY_INTERVAL_US: u64 = 25_000;
const DEMO_RPM: f64 = 4500.0;

fn fraction(value: f64, min: f64, max: f64) -> f64 {
    ((value - min) / (max - min)).clamp(0.0, 1.0)
}

fn rpm_mapping(rpm: f64) -> f64 { fraction(rpm, RPM_MIN, RPM_MAX) }

fn theta_needle(speed: f64) -> f64 {
    THETA_MIN + fraction(speed, V_MIN, V_MAX) * (THETA_MAX - THETA_MIN)
}

fn radial_point(radius: f64, theta: f64) -> Point {
    Point::new(
        X_CENTER + math::round(radius * math::cos(theta)) as i32,
        Y_CENTER + math::round(radius * math::sin(theta)) as i32,
    )
}

// No allocation. This writer is only used from main, never from the ISR.
struct BufWriter<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> BufWriter<'a> {
    fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, pos: 0 }
    }
    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[..self.pos]).unwrap()
    }
}

impl<'a> fmt::Write for BufWriter<'a> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let bytes = s.as_bytes();
        let remaining = &mut self.buf[self.pos..];
        if remaining.len() < bytes.len() {
            return Err(fmt::Error);
        }
        remaining[..bytes.len()].copy_from_slice(bytes);
        self.pos += bytes.len();
        Ok(())
    }
}

// Measurement state is independent of rendering and can be reused for RPM.
#[derive(Clone, Copy)]
struct PulseState {
    count: u64,
    last_us: Option<u64>,
    period_us: Option<u64>,
}

impl PulseState {
    const fn new() -> Self {
        Self { count: 0, last_us: None, period_us: None }
    }

    fn record(&mut self, now_us: u64, timeout_us: u64) {
        self.period_us = self.last_us.and_then(|last| {
            let dt = now_us - last;
            if dt > 0 && dt < timeout_us { Some(dt) } else { None }
        });
        self.last_us = Some(now_us);
        self.count = self.count.wrapping_add(1);
    }

    fn fresh_period(&self, now_us: u64, timeout_us: u64) -> Option<u64> {
        match (self.last_us, self.period_us) {
            (Some(last), Some(dt)) if dt > 0 && now_us - last < timeout_us => Some(dt),
            _ => None,
        }
    }
}

fn wheel_speed(state: PulseState, now_us: u64) -> f64 {
    state.fresh_period(now_us, SPEED_TIMEOUT_US).map_or(0.0, |dt| {
        3_600_000.0 * WHEEL_CIRCUMFERENCE_M / (WHEEL_PULSES_PER_REV * dt as f64)
    })
}

type HallPin = Pin<Gpio32, FunctionSio<SioInput>, PullUp>;
type CaptureTimer = hal::Timer<CopyableTimer0>;

// Own pin and timer together so the handler never sees partial initialization.
struct Capture {
    hall: HallPin,
    timer: CaptureTimer,
    wheel: PulseState,
}
static CAPTURE: Mutex<RefCell<Option<Capture>>> = Mutex::new(RefCell::new(None));

fn start_wheel_capture(mut hall: HallPin, timer: CaptureTimer) {
    cortex_m::interrupt::free(|cs| {
        hall.clear_interrupt(GpioInterrupt::EdgeLow);
        hall.set_interrupt_enabled(GpioInterrupt::EdgeLow, true);
        CAPTURE.borrow(cs).replace(Some(Capture { hall, timer, wheel: PulseState::new() }));
    });
    // SAFETY: state is initialized; only core 0 owns this interrupt and pin.
    unsafe { NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0); }
}

#[interrupt]
fn IO_IRQ_BANK0() {
    cortex_m::interrupt::free(|cs| {
        let mut slot = CAPTURE.borrow(cs).borrow_mut();
        if let Some(capture) = slot.as_mut() {
            if capture.hall.interrupt_status(GpioInterrupt::EdgeLow) {
                capture.hall.clear_interrupt(GpioInterrupt::EdgeLow);
                let now_us = capture.timer.get_counter().ticks();
                capture.wheel.record(now_us, SPEED_TIMEOUT_US);
            }
        }
    });
}

fn wheel_snapshot() -> PulseState {
    cortex_m::interrupt::free(|cs| {
        CAPTURE.borrow(cs).borrow().as_ref()
            .map_or(PulseState::new(), |capture| capture.wheel)
    })
}

fn draw_static<D: DrawTarget<Color = Rgb565>>(display: &mut D) -> Result<(), D::Error> {
    let text_style_small = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);
    let top_left_point = Point::new(X_CENTER - RADIUS as i32, Y_CENTER - RADIUS as i32);
    Circle::new(top_left_point, 2*RADIUS)
    .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 2))
    .draw(display)?;
    
    // A separate horizontal level indicator.
    Rectangle::new(Point::new(RPM_OUTER_X as i32, RPM_INNER_Y as i32 - 1), Size::new(RPM_BAR_MAX_WIDTH, 10))
    .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
    .draw(display)?;

    // create text buffer for write macro
    let mut text_buffer = [0u8; 32];
    
    // One loop for both labelled 1000-RPM ticks and shorter 500-RPM ticks.
    for step in 0..=20 {
        let rpm = step as f64 * 500.0;
        let x = RPM_INNER_X as i32
            + math::round(rpm_mapping(rpm) * RPM_INNER_WIDTH as f64) as i32;
        let major = step % 2 == 0;
        let height = if major { 10 } else { 5 };
        Line::new(
            Point::new(x, RPM_INNER_Y as i32 - 1),
            Point::new(x, RPM_INNER_Y as i32 - 1 - height),
        )
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
        .draw(display)?;

        if major {
            let mut text = BufWriter::new(&mut text_buffer);
            write!(text, "{}", step / 2).unwrap();
            let label = Text::new(text.as_str(), Point::zero(), text_style_small);
            let bounds = label.bounding_box();
            label.translate(Point::new(
                x - bounds.size.width as i32 / 2,
                RPM_INNER_Y as i32 - 14,
            )).draw(display)?;
        }
    }

    let tick_inner_radius = RADIUS as f64 - 6.0;
    let tick_outer_radius = RADIUS as f64 + 10.0;
    let label_radius = RADIUS as f64 + 23.0;

    for speed in (0..=160).step_by(10) {
        let theta = theta_needle(speed as f64);
        // Draw the tick.
        Line::new(
            radial_point(tick_outer_radius, theta),
            radial_point(tick_inner_radius, theta),
        )
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
        .draw(display)?;

        // Format the speed label.
        let mut w = BufWriter::new(&mut text_buffer);
        write!(w, "{speed}").unwrap();

        // Centre the text's bounding box on its radial position.
        let label_position = radial_point(label_radius, theta);
        let label = Text::new(w.as_str(), Point::zero(), text_style_small);
        let bounds = label.bounding_box();

        let offset = label_position
            - bounds.top_left
            - Point::new(
                bounds.size.width as i32 / 2,
                bounds.size.height as i32 / 2,
            );

        label
            .translate(offset)
            .draw(display)?;
    }

    Text::new("km/h", Point::new(SPEED_TEXT_X + 20, SPEED_TEXT_Y + 57), text_style_small)
        .draw(display)?;
    Text::new("RPM x1000 (DEMO)", Point::new(30, 199), text_style_small).draw(display)?;
    Ok(())
}

struct Dashboard {
    previous_tip: Option<Point>,
    previous_speed_hundredths: Option<u32>,
    previous_rpm_width: u32,
}

impl Dashboard {
    const fn new() -> Self {
        Self { previous_tip: None, previous_speed_hundredths: None, previous_rpm_width: 0 }
    }

    fn update<D: DrawTarget<Color = Rgb565>>(
        &mut self, display: &mut D, speed_kph: f64, rpm: f64,
    ) -> Result<(), D::Error> {
        let tip = radial_point(NEEDLE_LENGTH, theta_needle(speed_kph));
        if self.previous_tip != Some(tip) {
            if let Some(old) = self.previous_tip {
                Line::new(Point::new(X_CENTER, Y_CENTER), old)
                    .into_styled(PrimitiveStyle::with_stroke(Rgb565::BLACK, 3)).draw(display)?;
            }
            Line::new(Point::new(X_CENTER, Y_CENTER), tip)
                .into_styled(PrimitiveStyle::with_stroke(Rgb565::RED, 3)).draw(display)?;
            self.previous_tip = Some(tip);
        }

        // Compare exactly what will be displayed, separately from needle pixels.
        let hundredths = math::round(speed_kph.max(0.0) * 100.0) as u32;
        if self.previous_speed_hundredths != Some(hundredths) {
            Rectangle::new(Point::new(SPEED_TEXT_X + 18, SPEED_TEXT_Y + 20), Size::new(110, 24))
                .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK)).draw(display)?;
            let mut bytes = [0u8; 32];
            let mut text = BufWriter::new(&mut bytes);
            // Fit at most nine glyphs in the available screen area.
            if hundredths <= 99_999_999 {
                write!(text, "{}.{:02}", hundredths / 100, hundredths % 100).unwrap();
            } else {
                write!(text, "OVR").unwrap();
            }
            Text::new(text.as_str(), Point::new(SPEED_TEXT_X + 20, SPEED_TEXT_Y + 40),
                MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE)).draw(display)?;
            self.previous_speed_hundredths = Some(hundredths);
        }

        let width = (rpm_mapping(rpm) * RPM_INNER_WIDTH as f64) as u32;
        if width != self.previous_rpm_width {
            let start = width.min(self.previous_rpm_width);
            let delta = width.abs_diff(self.previous_rpm_width);
            let colour = if width > self.previous_rpm_width { Rgb565::GREEN } else { Rgb565::BLACK };
            Rectangle::new(Point::new(RPM_INNER_X as i32 + start as i32, RPM_INNER_Y as i32),
                Size::new(delta, RPM_INNER_SIZE_HEIGHT))
                .into_styled(PrimitiveStyle::with_fill(colour)).draw(display)?;
            self.previous_rpm_width = width;
        }
        Ok(())
    }
}

#[rp235x_hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let mut clocks = clocks::ClocksManager::new(pac.CLOCKS);

    watchdog.enable_tick_generation((XTAL_FREQ_HZ / 1_000_000) as u16);

    let xosc = hal::xosc::setup_xosc_blocking(pac.XOSC, XTAL_FREQ_HZ.Hz())
        .map_err(clocks::InitError::XoscErr)
        .unwrap();

    pub const PLL_SYS_200MHZ: hal::pll::PLLConfig = hal::pll::PLLConfig {
        vco_freq: fugit::HertzU32::MHz(VCO_FREQ),
        refdiv: REFDIV,
        post_div1: POST_DIVIDER_1,
        post_div2: POST_DIVIDER_2,
    };

    pub const PLL_USB_48MHZ: hal::pll::PLLConfig = hal::pll::PLLConfig {
        vco_freq: fugit::HertzU32::MHz(USB_VCO_FREQ),
        refdiv: USB_REFDIV,
        post_div1: USB_POST_DIVIDER_1,
        post_div2: USB_POST_DIVIDER_2,
    };

    let pll_sys = hal::pll::setup_pll_blocking(
        pac.PLL_SYS,
        xosc.operating_frequency().into(),
        PLL_SYS_200MHZ,
        &mut clocks,
        &mut pac.RESETS,
    )
    .unwrap();

    let pll_usb = hal::pll::setup_pll_blocking(
        pac.PLL_USB,
        xosc.operating_frequency().into(),
        PLL_USB_48MHZ,
        &mut clocks,
        &mut pac.RESETS,
    )
    .unwrap();

    // Initialize the system clock to 200 MHz and USB to 48 MHz.
    clocks.init_default(&xosc, &pll_sys, &pll_usb).unwrap();

    let sio = rp235x_hal::Sio::new(pac.SIO);

    let pins = rp235x_hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Timer also provides the capture timestamps.
    let mut delay =
        hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    // Hardware SPI signals.
    let mosi = pins.gpio19.into_function::<FunctionSpi>();
    let sclk = pins.gpio18.into_function::<FunctionSpi>();

    let hall = pins.gpio32.into_pull_up_input();

    // Software-controlled display signals.
    let cs = pins
        .gpio17
        .into_push_pull_output_in_state(PinState::High);

    let dc = pins
        .gpio16
        .into_push_pull_output_in_state(PinState::Low);

    let mut backlight = pins
        .gpio20
        .into_push_pull_output_in_state(PinState::Low);

    // Rust can infer the pin tuple type.
    let spi = Spi::<_, _, _, 8>::new(pac.SPI0, (mosi, sclk)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16_000_000u32.Hz(),
        MODE_0,
    );

    // Convert the SPI bus into a device that controls CS.
    // Timer implements Copy: this copies the timer handle.
    let spi_device = ExclusiveDevice::new(spi, cs, delay).unwrap();

    // Temporary transfer buffer, not a full framebuffer.
    let mut buffer = [0_u8; 512];
    let interface = SpiInterface::new(spi_device, dc, &mut buffer);

    let mut display = Builder::new(ST7789, interface)
        // Native controller dimensions before rotation.
        .display_size(240, 320)
        // Landscape drawing area: 320 × 240.
        .orientation(Orientation::new().rotate(Rotation::Deg90))
        .color_order(ColorOrder::Rgb)
        .invert_colors(ColorInversion::Inverted)
        // Reset is connected to RUN on the Display Pack.
        .init(&mut delay)
        .unwrap();

    // Clear any old contents before illuminating the screen.
    display.clear(Rgb565::BLACK).unwrap();

    draw_static(&mut display).unwrap();
    let mut dashboard = Dashboard::new();
    dashboard.update(&mut display, 0.0, DEMO_RPM).unwrap();
    backlight.set_high().unwrap();

    start_wheel_capture(hall, delay);
    let mut next_frame_us = delay.get_counter().ticks();

    loop {
        let now_us = delay.get_counter().ticks();
        if now_us >= next_frame_us {
            let wheel = wheel_snapshot();
            // Read time AFTER the snapshot to avoid a timestamp newer than now.
            let sample_now_us = delay.get_counter().ticks();
            let speed_kmh = wheel_speed(wheel, sample_now_us);
            dashboard.update(&mut display, speed_kmh, DEMO_RPM).unwrap();

            let finished_us = delay.get_counter().ticks();
            next_frame_us += DISPLAY_INTERVAL_US;
            if next_frame_us <= finished_us {
                next_frame_us += ((finished_us - next_frame_us) / DISPLAY_INTERVAL_US + 1)
                    * DISPLAY_INTERVAL_US;
            }
        }
        // This delay leaves interrupts enabled
        delay.delay_us(100);
    }
}
