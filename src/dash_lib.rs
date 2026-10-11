use core::{cell::RefCell, fmt::{self, Write}, f64::consts::PI};
use cortex_m::{
    interrupt::Mutex,
    peripheral::NVIC,
};
use libm as math;
use rp235x_hal::{
    self as hal,
    gpio::{bank0::Gpio32, FunctionSio, Interrupt as GpioInterrupt,
        Pin, PullUp, SioInput},
    pac,
    timer::CopyableTimer0,
};

use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::{FONT_10X20, FONT_6X10}},
    pixelcolor::Rgb565, prelude::*, text::Text,
    primitives::{Circle, Line, PrimitiveStyle, Rectangle},
};
use hal::pac::interrupt;
use panic_halt as _;

pub(crate) const WHEEL_CIRCUMFERENCE_M: f64 = 2.16; // Calibrate with a loaded wheel rollout.
pub(crate) const WHEEL_PULSES_PER_REV: f64 = 4.0; // Use 1.0 for one magnet per revolution.
pub(crate) const MIN_SPEED_KMH: f64 = 1.0;
pub(crate) const SPEED_TIMEOUT_US: u64 = (3_600_000.0 * WHEEL_CIRCUMFERENCE_M
    / (WHEEL_PULSES_PER_REV * MIN_SPEED_KMH)) as u64;

// speed should be mapped in the range 0-161 KPH. I don't think faster is required on this bike...
pub(crate) const V_MIN: f64 = 0.0;
pub(crate) const V_MAX: f64 = 161.0;
// the speed gague should be set with theta -225 to 0
pub(crate) const THETA_MIN: f64 = 3.0 * PI / 4.0 - 2.0 * PI;
pub(crate) const THETA_MAX: f64 = 0.0;
// rpm should be mapped in the range 0-10,000 RPM. It SHOULDN'T go higher than this or else I have real problems >_>
pub(crate) const RPM_MIN: f64 = 0.0;
pub(crate) const RPM_MAX: f64 = 10000.0;
// Dial circle position is the top-left of its bounding box.
pub(crate) const RADIUS: u32 = 64;
pub(crate) const X_CENTER: i32 = 110;
pub(crate) const Y_CENTER: i32 = 114;
// Leave space between the needle tip and the dial outline.
pub(crate) const NEEDLE_LENGTH: f64 = RADIUS as f64 - 12.0; // Clearance from tick strokes.
pub(crate) const RPM_BAR_MAX_WIDTH: u32 = 280;
pub(crate) const RPM_INNER_WIDTH: u32 = RPM_BAR_MAX_WIDTH - 2;
pub(crate) const RPM_OUTER_X: u32 = 30;
pub(crate) const RPM_INNER_X: u32 = RPM_OUTER_X + 1;
pub(crate) const RPM_INNER_Y: u32 = 230;
pub(crate) const RPM_INNER_SIZE_HEIGHT: u32 = 8;
pub(crate) const SPEED_X_OFFSET: i32 = 80;
pub(crate) const SPEED_Y_OFFSET: i32 = 0;
pub(crate) const SPEED_TEXT_X: i32 = X_CENTER + SPEED_X_OFFSET;
pub(crate) const SPEED_TEXT_Y: i32 = Y_CENTER + SPEED_Y_OFFSET;

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
pub(crate) struct BufWriter<'a> {
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
pub(crate) struct PulseState {
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

pub(crate) fn wheel_speed(state: PulseState, now_us: u64) -> f64 {
    state.fresh_period(now_us, SPEED_TIMEOUT_US).map_or(0.0, |dt| {
        3_600_000.0 * WHEEL_CIRCUMFERENCE_M / (WHEEL_PULSES_PER_REV * dt as f64)
    })
}

type HallPin = Pin<Gpio32, FunctionSio<SioInput>, PullUp>;
type CaptureTimer = hal::Timer<CopyableTimer0>;

// Own pin and timer together so the handler never sees partial initialization.
pub(crate) struct Capture {
    hall: HallPin,
    timer: CaptureTimer,
    wheel: PulseState,
}
static CAPTURE: Mutex<RefCell<Option<Capture>>> = Mutex::new(RefCell::new(None));

pub(crate) fn start_wheel_capture(mut hall: HallPin, timer: CaptureTimer) {
    cortex_m::interrupt::free(|cs| {
        hall.clear_interrupt(GpioInterrupt::EdgeLow);
        hall.set_interrupt_enabled(GpioInterrupt::EdgeLow, true);
        CAPTURE.borrow(cs).replace(Some(Capture { hall, timer, wheel: PulseState::new() }));
    });
    // SAFETY: state is initialized; only core 0 owns this interrupt and pin.
    unsafe { NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0); }}

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

pub(crate) fn wheel_snapshot() -> PulseState {
    cortex_m::interrupt::free(|cs| {
        CAPTURE.borrow(cs).borrow().as_ref()
            .map_or(PulseState::new(), |capture| capture.wheel)
    })
}

pub(crate) fn draw_static<D: DrawTarget<Color = Rgb565>>(display: &mut D) -> Result<(), D::Error> {
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

pub(crate) struct PulseDetector {
    armed: bool,
    last_event: Option<u64>,
    low: u16,
    high: u16,
    min_gap_samples: u64,
}

impl PulseDetector {
    /// creates a pulse detector that operates on a hysterisis principal where aevents are triggered above 
    /// high, then blocked until an event below low is tirggered, allowing another high event.
    pub(crate) fn new(low: u16, high: u16, min_gap_samples: u64) -> Self {
        assert!(low < high);

        Self {
            armed: false, // First observe the signal below `low`.
            last_event: None,
            low,
            high,
            min_gap_samples,
        }
    }

    // Returns the sample index of an accepted pulse.
    pub(crate) fn feed(&mut self, sample: u16, index: u64) -> Option<u64> {
        if sample <= self.low {
            self.armed = true;
            return None;
        }

        if !self.armed || sample < self.high {
            return None;
        }

        self.armed = false;

        if let Some(previous) = self.last_event {
            if index - previous < self.min_gap_samples {
                return None;
            }
        }

        self.last_event = Some(index);
        Some(index)
    }
}

pub(crate) struct Dashboard {
    previous_tip: Option<Point>,
    previous_speed_hundredths: Option<u32>,
    previous_rpm_width: u32,
}

impl Dashboard {
    pub(crate) const fn new() -> Self {
        Self { previous_tip: None, previous_speed_hundredths: None, previous_rpm_width: 0 }
    }

    pub(crate) fn update<D: DrawTarget<Color = Rgb565>>(
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