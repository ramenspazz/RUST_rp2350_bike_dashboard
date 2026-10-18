use core::{
    cell::RefCell,
    f64::consts::PI,
    fmt::{self, Write},
};
use cortex_m::{interrupt::Mutex, peripheral::NVIC};
use libm as math;
use rp235x_hal::{
    self as hal,
    gpio::{bank0::Gpio22, FunctionSio, Interrupt as GpioInterrupt, Pin, PullUp, SioInput},
    pac,
    timer::CopyableTimer0,
};

use embedded_graphics::{
    mono_font::{
        ascii::{FONT_10X20, FONT_6X10},
        MonoTextStyle,
    },
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Circle, Line, PrimitiveStyle, Rectangle},
    text::Text,
};
use hal::pac::interrupt;
use panic_halt as _;

pub(crate) const WHEEL_CIRCUMFERENCE_M: f64 = 2.16; // Calibrate with a loaded wheel rollout.
pub(crate) const WHEEL_PULSES_PER_REV: f64 = 4.0; // Use 1.0 for one magnet per revolution.
pub(crate) const MIN_SPEED_KMH: f64 = 1.0;
pub(crate) const SPEED_TIMEOUT_US: u64 =
    (3_600_000.0 * WHEEL_CIRCUMFERENCE_M / (WHEEL_PULSES_PER_REV * MIN_SPEED_KMH)) as u64;

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

fn rpm_mapping(rpm: f64) -> f64 {
    fraction(rpm, RPM_MIN, RPM_MAX)
}

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

#[derive(Clone, Copy)]
pub(crate) struct PulseState {
    count: u64,
    last_us: Option<u64>,
    period_us: Option<u64>,
}

impl PulseState {
    const fn new() -> Self {
        Self {
            count: 0,
            last_us: None,
            period_us: None,
        }
    }

    fn record(&mut self, now_us: u64, timeout_us: u64) {
        self.period_us = self.last_us.and_then(|last| {
            let dt = now_us - last;
            if dt > 0 && dt < timeout_us {
                Some(dt)
            } else {
                None
            }
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
    state
        .fresh_period(now_us, SPEED_TIMEOUT_US)
        .map_or(0.0, |dt| {
            3_600_000.0 * WHEEL_CIRCUMFERENCE_M / (WHEEL_PULSES_PER_REV * dt as f64)
        })
}

type HallPin = Pin<Gpio22, FunctionSio<SioInput>, PullUp>;
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
        CAPTURE.borrow(cs).replace(Some(Capture {
            hall,
            timer,
            wheel: PulseState::new(),
        }));
    });
    // SAFETY: state is initialized; only core 0 owns this interrupt and pin.
    unsafe {
        NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }
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

pub(crate) fn wheel_snapshot() -> PulseState {
    cortex_m::interrupt::free(|cs| {
        CAPTURE
            .borrow(cs)
            .borrow()
            .as_ref()
            .map_or(PulseState::new(), |capture| capture.wheel)
    })
}

pub(crate) fn _draw_static<D: DrawTarget<Color = Rgb565>>(display: &mut D) -> Result<(), D::Error> {
    let text_style_small = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);
    let top_left_point = Point::new(X_CENTER - RADIUS as i32, Y_CENTER - RADIUS as i32);
    Circle::new(top_left_point, 2 * RADIUS)
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 2))
        .draw_visible(display)?;

    // A separate horizontal level indicator.
    Rectangle::new(
        Point::new(RPM_OUTER_X as i32, RPM_INNER_Y as i32 - 1),
        Size::new(RPM_BAR_MAX_WIDTH, 10),
    )
    .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
    .draw_visible(display)?;

    // create text buffer for write macro
    let mut text_buffer = [0u8; 32];

    // One loop for both labelled 1000-RPM ticks and shorter 500-RPM ticks.
    for step in 0..=20 {
        let rpm = step as f64 * 500.0;
        let x = RPM_INNER_X as i32 + math::round(rpm_mapping(rpm) * RPM_INNER_WIDTH as f64) as i32;
        let major = step % 2 == 0;
        let height = if major { 10 } else { 5 };
        Line::new(
            Point::new(x, RPM_INNER_Y as i32 - 1),
            Point::new(x, RPM_INNER_Y as i32 - 1 - height),
        )
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
        .draw_visible(display)?;

        if major {
            let mut text = BufWriter::new(&mut text_buffer);
            write!(text, "{}", step / 2).unwrap();
            let label = Text::new(text.as_str(), Point::zero(), text_style_small);
            let bounds = label.bounding_box();
            label
                .translate(Point::new(
                    x - bounds.size.width as i32 / 2,
                    RPM_INNER_Y as i32 - 14,
                ))
                .draw_visible(display)?;
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
        .draw_visible(display)?;

        // Format the speed label.
        let mut w = BufWriter::new(&mut text_buffer);
        write!(w, "{speed}").unwrap();

        // Centre the text's bounding box on its radial position.
        let label_position = radial_point(label_radius, theta);
        let label = Text::new(w.as_str(), Point::zero(), text_style_small);
        let bounds = label.bounding_box();

        let offset = label_position
            - bounds.top_left
            - Point::new(bounds.size.width as i32 / 2, bounds.size.height as i32 / 2);

        label.translate(offset).draw_visible(display)?;
    }

    Text::new(
        "km/h",
        Point::new(SPEED_TEXT_X + 20, SPEED_TEXT_Y + 57),
        text_style_small,
    )
    .draw_visible(display)?;
    Text::new("RPM x1000", Point::new(30, 199), text_style_small).draw_visible(display)?;
    Ok(())
}

// Cached screen geometry. Built once; no allocation and no per-tile trigonometry.
#[derive(Clone, Copy)]
struct CachedLabel {
    text: &'static str,
    position: Point,
}
struct StaticGeometry {
    ticks: [Line; 38],
    labels: [CachedLabel; 28],
}
impl StaticGeometry {
    fn new() -> Self {
        let mut ticks = [Line::new(Point::zero(), Point::zero()); 38];
        let mut labels = [CachedLabel {
            text: "",
            position: Point::zero(),
        }; 28];
        let style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);
        const RPM_LABELS: [&str; 11] = ["0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10"];
        const SPEED_LABELS: [&str; 17] = [
            "0", "10", "20", "30", "40", "50", "60", "70", "80", "90", "100", "110", "120", "130",
            "140", "150", "160",
        ];
        for step in 0..=20 {
            let x = RPM_INNER_X as i32
                + math::round(rpm_mapping(step as f64 * 500.0) * RPM_INNER_WIDTH as f64) as i32;
            let height = if step % 2 == 0 { 10 } else { 5 };
            ticks[step] = Line::new(
                Point::new(x, RPM_INNER_Y as i32 - 1),
                Point::new(x, RPM_INNER_Y as i32 - 1 - height),
            );
            if step % 2 == 0 {
                let text = RPM_LABELS[step / 2];
                let bounds = Text::new(text, Point::zero(), style).bounding_box();
                labels[step / 2] = CachedLabel {
                    text,
                    position: Point::new(x - bounds.size.width as i32 / 2, RPM_INNER_Y as i32 - 14),
                };
            }
        }
        for i in 0..17 {
            let theta = theta_needle(i as f64 * 10.0);
            // Compute sin/cos once per tick, not once per endpoint.
            let (cos, sin) = (math::cos(theta), math::sin(theta));
            let point = |radius: f64| {
                Point::new(
                    X_CENTER + math::round(radius * cos) as i32,
                    Y_CENTER + math::round(radius * sin) as i32,
                )
            };
            ticks[21 + i] = Line::new(point(RADIUS as f64 + 10.0), point(RADIUS as f64 - 6.0));
            let text = SPEED_LABELS[i];
            let bounds = Text::new(text, Point::zero(), style).bounding_box();
            let position = point(RADIUS as f64 + 23.0)
                - bounds.top_left
                - Point::new(bounds.size.width as i32 / 2, bounds.size.height as i32 / 2);
            labels[11 + i] = CachedLabel { text, position };
        }
        Self { ticks, labels }
    }
    fn draw<D: DrawTarget<Color = Rgb565>>(&self, target: &mut D) -> Result<(), D::Error> {
        Circle::new(
            Point::new(X_CENTER - RADIUS as i32, Y_CENTER - RADIUS as i32),
            2 * RADIUS,
        )
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 2))
        .draw_visible(target)?;
        Rectangle::new(
            Point::new(RPM_OUTER_X as i32, RPM_INNER_Y as i32 - 1),
            Size::new(RPM_BAR_MAX_WIDTH, 10),
        )
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
        .draw_visible(target)?;
        for tick in &self.ticks {
            tick.into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
                .draw_visible(target)?;
        }
        let style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);
        for label in &self.labels {
            Text::new(label.text, label.position, style).draw_visible(target)?;
        }
        Text::new(
            "km/h",
            Point::new(SPEED_TEXT_X + 20, SPEED_TEXT_Y + 57),
            style,
        )
        .draw_visible(target)?;
        Text::new("RPM x1000", Point::new(30, 199), style).draw_visible(target)?;
        Ok(())
    }
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

// ---- Tiled dashboard renderer ------------------------------------------------
// Coordinates are screen coordinates, even when the target is a small RAM tile.
use core::convert::Infallible;

pub(crate) const SCREEN_WIDTH: u32 = 320;
pub(crate) const SCREEN_HEIGHT: u32 = 240;
pub(crate) const TILE_SIZE: u32 = 32;
pub(crate) const TILE_BYTES: usize = 32 * 32 * 2;
const TILE_COLS: usize = 10;
const TILE_ROWS: usize = 8;
const TILE_COUNT: usize = TILE_COLS * TILE_ROWS;

// Avoid rasterizing primitives that cannot contribute to this tile.
trait DrawVisible: Drawable<Color = Rgb565> + Dimensions {
    fn draw_visible<D: DrawTarget<Color = Rgb565>>(&self, target: &mut D) -> Result<(), D::Error> {
        let overlap = self.bounding_box().intersection(&target.bounding_box());
        if overlap.size.width != 0 && overlap.size.height != 0 {
            self.draw(target).map(|_| ())?;
        }
        Ok(())
    }
}
impl<T: Drawable<Color = Rgb565> + Dimensions> DrawVisible for T {}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct TileRect {
    pub(crate) x: u16,
    pub(crate) y: u16,
    pub(crate) width: u16,
    pub(crate) height: u16,
}
impl TileRect {
    fn from_index(index: usize) -> Self {
        let x = (index % TILE_COLS) as u32 * TILE_SIZE;
        let y = (index / TILE_COLS) as u32 * TILE_SIZE;
        Self {
            x: x as u16,
            y: y as u16,
            width: TILE_SIZE.min(SCREEN_WIDTH - x) as u16,
            height: TILE_SIZE.min(SCREEN_HEIGHT - y) as u16,
        }
    }
    pub(crate) fn byte_len(self) -> usize {
        self.width as usize * self.height as usize * 2
    }
    fn bounds(self) -> Rectangle {
        Rectangle::new(
            Point::new(self.x as i32, self.y as i32),
            Size::new(self.width as u32, self.height as u32),
        )
    }
}

pub(crate) struct TileBuffer {
    rect: TileRect,
    bytes: [u8; TILE_BYTES],
}
impl TileBuffer {
    pub(crate) const fn new() -> Self {
        Self {
            rect: TileRect {
                x: 0,
                y: 0,
                width: 32,
                height: 32,
            },
            bytes: [0; TILE_BYTES],
        }
    }
    // pub(crate) fn rect(&self) -> TileRect {
    //     self.rect
    // }
    // pub(crate) fn pixels(&self) -> &[u8] {
    //     &self.bytes[..self.rect.byte_len()]
    // }
    fn prepare(&mut self, rect: TileRect) {
        self.rect = rect;
        self.bytes[..rect.byte_len()].fill(0); // Final tile starts with black background.
    }
}
// Implement Dimensions rather than OriginDimensions: the tile has a nonzero origin.
impl Dimensions for TileBuffer {
    fn bounding_box(&self) -> Rectangle {
        self.rect.bounds()
    }
}
impl DrawTarget for TileBuffer {
    type Color = Rgb565;
    type Error = Infallible;
    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Rgb565>>,
    {
        for Pixel(point, colour) in pixels {
            let x = point.x - self.rect.x as i32;
            let y = point.y - self.rect.y as i32;
            if x >= 0 && y >= 0 && x < self.rect.width as i32 && y < self.rect.height as i32 {
                let offset = 2 * (y as usize * self.rect.width as usize + x as usize);
                self.bytes[offset..offset + 2]
                    .copy_from_slice(&colour.into_storage().to_be_bytes());
            }
        }
        Ok(())
    }
    fn fill_solid(&mut self, area: &Rectangle, colour: Rgb565) -> Result<(), Self::Error> {
        let area = area.intersection(&self.bounding_box());
        self.draw_iter(area.points().map(|point| Pixel(point, colour)))
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Screen {
    Gauges,
    Menu,
    Detail,
}
#[derive(Clone, Copy, Debug)]
pub(crate) enum MenuInput {
    Open,
    Up,
    Down,
    Select,
    Back,
}
const MENU_ITEMS: [&str; 5] = [
    "Return to gauges",
    "Trip / odometer",
    "Calibration",
    "Display settings",
    "About",
];

pub(crate) struct Dashboard {
    screen: Screen,
    selection: usize,
    geometry: StaticGeometry,
    tip: Option<Point>,
    speed_hundredths: Option<u32>,
    rpm_width: u32,
    dirty: [bool; TILE_COUNT],
    in_flight: Option<usize>,
    next_tile: usize,
}
impl Dashboard {
    pub(crate) fn new() -> Self {
        // First pass draws the entire scene, including static markings.
        Self {
            geometry: StaticGeometry::new(),
            screen: Screen::Gauges,
            selection: 0,
            tip: None,
            speed_hundredths: None,
            rpm_width: 0,
            dirty: [true; TILE_COUNT],
            in_flight: None,
            next_tile: 0,
        }
    }
    /// UI only: measurement capture and update() continue on every screen.
    pub(crate) fn menu_input(&mut self, input: MenuInput) {
        match (self.screen, input) {
            (Screen::Gauges, MenuInput::Open) => self.change_screen(Screen::Menu),
            (Screen::Menu, MenuInput::Back) => self.change_screen(Screen::Gauges),
            (Screen::Detail, MenuInput::Back) => self.change_screen(Screen::Menu),
            (Screen::Menu, MenuInput::Up | MenuInput::Down) => {
                let old = self.selection;
                self.selection = match input {
                    MenuInput::Up => (old + MENU_ITEMS.len() - 1) % MENU_ITEMS.len(),
                    _ => (old + 1) % MENU_ITEMS.len(),
                };
                self.mark_rect(Self::menu_row(old));
                self.mark_rect(Self::menu_row(self.selection));
            }
            (Screen::Menu, MenuInput::Select) => {
                self.change_screen(if self.selection == 0 {
                    Screen::Gauges
                } else {
                    Screen::Detail
                });
            }
            _ => {}
        }
    }
    fn change_screen(&mut self, screen: Screen) {
        self.screen = screen;
        // Safe even during DMA: in-flight old-screen tile will be redrawn.
        self.invalidate_all();
    }
    fn menu_row(index: usize) -> Rectangle {
        Rectangle::new(Point::new(16, 48 + index as i32 * 30), Size::new(288, 26))
    }
    fn mark_gauge_rect(&mut self, rect: Rectangle) {
        if self.screen == Screen::Gauges {
            self.mark_rect(rect);
        }
    }
    fn draw_menu<D: DrawTarget<Color = Rgb565>>(&self, target: &mut D) -> Result<(), D::Error> {
        let normal = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);
        let small = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);
        if self.screen == Screen::Menu {
            Text::new("MENU", Point::new(20, 30), normal).draw_visible(target)?;
            for (i, label) in MENU_ITEMS.iter().enumerate() {
                let row = Self::menu_row(i);
                let selected = i == self.selection;
                if selected {
                    row.into_styled(PrimitiveStyle::with_fill(Rgb565::WHITE))
                        .draw_visible(target)?;
                }
                let style = MonoTextStyle::new(
                    &FONT_10X20,
                    if selected {
                        Rgb565::BLACK
                    } else {
                        Rgb565::WHITE
                    },
                );
                if selected {
                    Text::new(">", row.top_left + Point::new(6, 20), style).draw_visible(target)?;
                }
                Text::new(label, row.top_left + Point::new(26, 20), style).draw_visible(target)?;
            }
            Text::new(
                "Up/Down: move   A: select   B: back",
                Point::new(20, 220),
                small,
            )
            .draw_visible(target)?;
        } else {
            Text::new(MENU_ITEMS[self.selection], Point::new(20, 30), normal)
                .draw_visible(target)?;
            let lines: [&str; 3] = match self.selection {
                1 => [
                    "Odometer and trip distance",
                    "Not implemented yet",
                    "No distance values stored yet",
                ],
                2 => [
                    "Wheel and ignition calibration",
                    "Not implemented yet",
                    "Current code constants remain active",
                ],
                3 => [
                    "Display preferences",
                    "Not implemented yet",
                    "Current display settings remain active",
                ],
                _ => [
                    "RP2350 motorcycle dashboard",
                    "ST7789 / SPI DMA tiles",
                    "Qw/ST Pad menu",
                ],
            };
            for (i, line) in lines.iter().enumerate() {
                Text::new(line, Point::new(20, 75 + i as i32 * 22), small).draw_visible(target)?;
            }
            Text::new("B: back to menu", Point::new(20, 220), small).draw_visible(target)?;
        }
        Ok(())
    }

    /// Queue all tiles, preserving updates even if a transfer is in progress.
    pub(crate) fn invalidate_all(&mut self) {
        self.dirty.fill(true);
    }

    /// Blocking full-screen redraw through any RGB565 DrawTarget.
    /// Only call while LCD DMA is stopped and you own the target/interface.
    /// This does not change the dirty queue; queued tiles may be resent afterward.
    pub(crate) fn _draw_full<D: DrawTarget<Color = Rgb565>>(
        &self,
        target: &mut D,
    ) -> Result<(), D::Error> {
        target.clear(Rgb565::BLACK)?;
        self.draw_scene(target)
    }

    fn mark_rect(&mut self, area: Rectangle) {
        let area = area.intersection(&Rectangle::new(
            Point::zero(),
            Size::new(SCREEN_WIDTH, SCREEN_HEIGHT),
        ));
        if area.size.width == 0 || area.size.height == 0 {
            return;
        }
        let x0 = area.top_left.x as usize / TILE_SIZE as usize;
        let y0 = area.top_left.y as usize / TILE_SIZE as usize;
        let x1 = (area.top_left.x as usize + area.size.width as usize - 1) / TILE_SIZE as usize;
        let y1 = (area.top_left.y as usize + area.size.height as usize - 1) / TILE_SIZE as usize;
        for y in y0..=y1 {
            for x in x0..=x1 {
                self.dirty[y * TILE_COLS + x] = true;
            }
        }
    }
    fn needle_bounds(tip: Point) -> Rectangle {
        // Conservative 3-pixel margin around endpoints for the 3-pixel stroke.
        let left = X_CENTER.min(tip.x) - 3;
        let top = Y_CENTER.min(tip.y) - 3;
        Rectangle::new(
            Point::new(left, top),
            Size::new(
                (X_CENTER.max(tip.x) + 3 - left + 1) as u32,
                (Y_CENTER.max(tip.y) + 3 - top + 1) as u32,
            ),
        )
    }
    /// Store new values and invalidate affected tiles; does not access SPI.
    pub(crate) fn update(&mut self, speed_kph: f64, rpm: f64) {
        let speed_kph = if speed_kph.is_finite() {
            speed_kph.max(0.0)
        } else {
            0.0
        };
        let rpm = if rpm.is_finite() { rpm.max(0.0) } else { 0.0 };
        let tip = radial_point(NEEDLE_LENGTH, theta_needle(speed_kph));
        if self.tip != Some(tip) {
            if let Some(old) = self.tip {
                self.mark_gauge_rect(Self::needle_bounds(old));
            }
            self.mark_gauge_rect(Self::needle_bounds(tip));
            self.tip = Some(tip);
        }
        let hundredths = math::round(speed_kph * 100.0) as u32;
        if self.speed_hundredths != Some(hundredths) {
            self.mark_gauge_rect(Rectangle::new(
                Point::new(SPEED_TEXT_X + 18, SPEED_TEXT_Y + 20),
                Size::new(110, 24),
            ));
            self.speed_hundredths = Some(hundredths);
        }
        let width = (rpm_mapping(rpm) * RPM_INNER_WIDTH as f64) as u32;
        if width != self.rpm_width {
            let start = width.min(self.rpm_width);
            self.mark_gauge_rect(Rectangle::new(
                Point::new(RPM_INNER_X as i32 + start as i32, RPM_INNER_Y as i32),
                Size::new(width.abs_diff(self.rpm_width), RPM_INNER_SIZE_HEIGHT),
            ));
            self.rpm_width = width;
        }
    }
    /// Render and reserve ONE tile. Call finish_tile() only after SPI has drained.
    /// Main/core 0 owns this object; do not update it concurrently from an ISR.
    pub(crate) fn render_next_tile(&mut self, buffer: &mut TileBuffer) -> Option<TileRect> {
        if self.in_flight.is_some() {
            return None;
        }
        let index = (0..TILE_COUNT)
            .map(|n| (self.next_tile + n) % TILE_COUNT)
            .find(|&n| self.dirty[n])?;
        // Clear at reservation, NOT at completion. Later update() calls may set it again.
        self.dirty[index] = false;
        self.in_flight = Some(index);
        self.next_tile = (index + 1) % TILE_COUNT;
        let rect = TileRect::from_index(index);
        buffer.prepare(rect);
        self.draw_scene(buffer).unwrap();
        Some(rect)
    }
    pub(crate) fn finish_tile(&mut self) {
        assert!(self.in_flight.take().is_some());
        // Deliberately leave dirty[] untouched: a newer value may have arrived.
    }
    /// Requeue a reserved tile if transmission was cancelled/failed.
    /// Recover/stop the DMA hardware separately before reusing its memory.
    pub(crate) fn _retry_tile(&mut self) {
        if let Some(index) = self.in_flight.take() {
            self.dirty[index] = true;
        }
    }
    fn draw_scene<D: DrawTarget<Color = Rgb565>>(&self, target: &mut D) -> Result<(), D::Error> {
        if self.screen != Screen::Gauges {
            return self.draw_menu(target);
        }
        self.geometry.draw(target)?;
        if self.rpm_width != 0 {
            Rectangle::new(
                Point::new(RPM_INNER_X as i32, RPM_INNER_Y as i32),
                Size::new(self.rpm_width, RPM_INNER_SIZE_HEIGHT),
            )
            .into_styled(PrimitiveStyle::with_fill(Rgb565::GREEN))
            .draw_visible(target)?;
        }
        if let Some(tip) = self.tip {
            Line::new(Point::new(X_CENTER, Y_CENTER), tip)
                .into_styled(PrimitiveStyle::with_stroke(Rgb565::RED, 3))
                .draw_visible(target)?;
        }
        if let Some(hundredths) = self.speed_hundredths {
            let mut bytes = [0u8; 32];
            let mut text = BufWriter::new(&mut bytes);
            // Fit at most nine glyphs in the available screen area.
            if hundredths <= 99_999_999 {
                write!(text, "{}.{:02}", hundredths / 100, hundredths % 100).unwrap();
            } else {
                write!(text, "OVR").unwrap();
            }
            Text::new(
                text.as_str(),
                Point::new(SPEED_TEXT_X + 20, SPEED_TEXT_Y + 40),
                MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE),
            )
            .draw_visible(target)?;
        }
        Ok(())
    }
}

// Owning DMA adapters retain the FULL buffer while exposing only valid tile bytes.
// This is essential for the bottom 32x16 tiles: RX and TX must both count 1024,
// not 2048, bytes. No extra embedded-dma dependency is required.
pub(crate) struct TileTx {
    buffer: &'static mut TileBuffer,
}
impl TileTx {
    pub(crate) fn new(buffer: &'static mut TileBuffer) -> Self {
        Self { buffer }
    }
    pub(crate) fn into_buffer(self) -> &'static mut TileBuffer {
        self.buffer
    }
}
// SAFETY: the unique static borrow owns a stable allocation throughout the transfer;
// no safe method permits mutation of its bytes until this adapter is consumed.
unsafe impl hal::dma::ReadTarget for TileTx {
    type ReceivedWord = u8;
    fn rx_treq() -> Option<u8> {
        None
    }
    fn rx_address_count(&self) -> (u32, u32) {
        (
            self.buffer.bytes.as_ptr() as u32,
            self.buffer.rect.byte_len() as u32,
        )
    }
    fn rx_increment(&self) -> bool {
        true
    }
}

// SPI receives one byte for each byte sent. Repeatedly overwrite ONE scratch byte.
// No CPU reads it while DMA owns this adapter.
pub(crate) struct TileRx {
    scratch: &'static mut u8,
    count: u32,
}
impl TileRx {
    pub(crate) fn new(scratch: &'static mut u8, rect: TileRect) -> Self {
        Self {
            scratch,
            count: rect.byte_len() as u32,
        }
    }
    pub(crate) fn into_scratch(self) -> &'static mut u8 {
        self.scratch
    }
}
// SAFETY: a unique static byte remains valid; a nonincrementing DMA destination
// writes that same byte for the finite transfer count. Count is at most TILE_BYTES.
unsafe impl hal::dma::WriteTarget for TileRx {
    type TransmittedWord = u8;
    fn tx_treq() -> Option<u8> {
        None
    }
    fn tx_address_count(&mut self) -> (u32, u32) {
        (self.scratch as *mut u8 as u32, self.count)
    }
    fn tx_increment(&self) -> bool {
        false
    }
}

/// Call with CS already low. Leaves DC high, ready for RGB565 bytes.
/// Assumes the existing 320x240 landscape ST7789 setup with zero display offset.
pub(crate) fn begin_tile_write<SPI, DC>(
    spi: &mut SPI,
    dc: &mut DC,
    rect: TileRect,
) -> Result<(), SPI::Error>
where
    SPI: embedded_hal::spi::SpiBus<u8>,
    DC: embedded_hal::digital::OutputPin<Error = Infallible>,
{
    fn command<SPI, DC>(spi: &mut SPI, dc: &mut DC, cmd: u8, args: &[u8]) -> Result<(), SPI::Error>
    where
        SPI: embedded_hal::spi::SpiBus<u8>,
        DC: embedded_hal::digital::OutputPin<Error = Infallible>,
    {
        spi.flush()?;
        dc.set_low().unwrap();
        spi.write(&[cmd])?;
        spi.flush()?;
        dc.set_high().unwrap();
        if !args.is_empty() {
            spi.write(args)?;
            spi.flush()?;
        }
        Ok(())
    }
    let x0 = rect.x.to_be_bytes();
    let x1 = (rect.x + rect.width - 1).to_be_bytes();
    let y0 = rect.y.to_be_bytes();
    let y1 = (rect.y + rect.height - 1).to_be_bytes();
    command(spi, dc, 0x2a, &[x0[0], x0[1], x1[0], x1[1]])?;
    command(spi, dc, 0x2b, &[y0[0], y0[1], y1[0], y1[1]])?;
    command(spi, dc, 0x2c, &[])
}
