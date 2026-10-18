#![no_std]
#![no_main]

use rp235x_hal::{
    self as hal,
    adc::{Adc, AdcPin},
    clocks::{self, Clock},
    dma::{bidirectional, double_buffer, DMAExt},
    gpio::{FunctionI2c,FunctionSpi, PinState,},
    I2C,
    pac,
    singleton,
    spi::Spi,
};
use embedded_hal::{digital::OutputPin, spi::MODE_0};
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_graphics::{
    pixelcolor::Rgb565, prelude::*,
};
use fugit::RateExtU32;
// use libm as math;
use mipidsi::{interface::SpiInterface, models::ST7789,
    options::{ColorInversion, ColorOrder, Orientation, Rotation}, Builder};
use panic_halt as _;

mod dash_lib;
mod qwst_pad;

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
const DISPLAY_INTERVAL_US: u64 = 25_000;
// const DEMO_RPM: f64 = 4500.0;
const IGNITION_EVENTS_PER_REV: f64 = 1.0;

const BUFFER_SAMPLES: usize = 8192;

const LOW_THRESHOLD: u16 = 248;  // Approximately 0.2 V
const HIGH_THRESHOLD: u16 = 496; // Approximately 0.4 V
const SAMPLE_RATE_HZ: u64 = 500_000;
const MIN_GAP_SAMPLES: u64 = SAMPLE_RATE_HZ / 1_000;

// Example stopped-engine timeout: 0.5 seconds.
const RPM_TIMEOUT_SAMPLES: u64 = SAMPLE_RATE_HZ / 2;

// const PATCH_WIDTH: usize = 32;
// const PATCH_HEIGHT: usize = 32;
// const PATCH_BYTES: usize = PATCH_WIDTH * PATCH_HEIGHT * 2;

// SPI helper functions
use core::convert::Infallible;
use embedded_hal::{
    spi::SpiBus,
};

fn _lcd_command<SPI, DC>(
    spi: &mut SPI,
    dc: &mut DC,
    command: u8,
    arguments: &[u8],
) -> Result<(), SPI::Error>
where
    SPI: SpiBus<u8>,
    DC: OutputPin<Error = Infallible>,
{
    spi.flush()?;
    dc.set_low().unwrap();

    spi.write(&[command])?;
    spi.flush()?;

    dc.set_high().unwrap();

    if !arguments.is_empty() {
        spi.write(arguments)?;
        spi.flush()?;
    }

    Ok(())
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

    let hall = pins.gpio22.into_pull_up_input();

    // Software-controlled display signals.
    let mut cs = pins
        .gpio17
        .into_push_pull_output_in_state(PinState::High);

    let mut dc = pins
        .gpio16
        .into_push_pull_output_in_state(PinState::Low);

    let mut backlight = pins
        .gpio20
        .into_push_pull_output_in_state(PinState::Low);

    // Rust can infer the pin tuple type.
    let mut spi = Spi::<_, _, _, 8>::new(pac.SPI0, (mosi, sclk)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16_000_000u32.Hz(),
        MODE_0,
    );

    // the initialization is inside a scope to free the spi, cs and dc for use with DMA
    {    
        // Convert the SPI bus into a device that controls CS.
        // Timer implements Copy: this copies the timer handle.
        let spi_device = ExclusiveDevice::new(&mut spi, &mut cs, delay).unwrap();

        // Temporary transfer buffer, not a full framebuffer.
        let mut buffer = [0_u8; 512];
        let interface = SpiInterface::new(spi_device, &mut dc, &mut buffer);

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
    }
    // display now falls out of scope and no longer exists, but the physical
    // display is still initialized 

    const SAMPLE_RATE_HZ: u64 = 500_000;

    // dash_lib::draw_static(&mut display).unwrap();
    // let mut dashboard = dash_lib::Dashboard::new();
    // dashboard.update(&mut display, 0.0, DEMO_RPM).unwrap();
    backlight.set_high().unwrap();

    dash_lib::start_wheel_capture(hall, delay);
    let mut next_frame_us = delay.get_counter().ticks();

    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);

    let _a0_digital = pins.gpio26.into_floating_input();
    let mut tach_input =
        AdcPin::new(pins.gpio40.into_floating_input()).unwrap();

    // If DMA is already split elsewhere, reuse its unused channels.
    let dma = pac.DMA.split(&mut pac.RESETS);

    // Static storage: these arrays do not occupy the main stack.
    let buffer_a =
        singleton!(: [u16; BUFFER_SAMPLES] = [0; BUFFER_SAMPLES])
            .unwrap();

    let buffer_b =
        singleton!(: [u16; BUFFER_SAMPLES] = [0; BUFFER_SAMPLES])
            .unwrap();

    // Configure the FIFO but leave sampling paused until DMA is ready.
    let mut fifo = adc
        .build_fifo()
        .clock_divider(0, 0) // 500 ksps with a 48 MHz ADC clock
        .set_channel(&mut tach_input)
        .enable_dma()
        .start_paused();

    // Prepare A, then queue B before starting the ADC.
    let mut transfer = double_buffer::Config::new(
        (dma.ch0, dma.ch1),
        fifo.dma_read_target(),
        buffer_a,
    )
    .start()
    .write_next(buffer_b);

    // Persistent detector and measurement state.
    let mut detector = dash_lib::PulseDetector::new(
        LOW_THRESHOLD,
        HIGH_THRESHOLD,
        MIN_GAP_SAMPLES,
    );

    let mut sample_index = 0u64;
    let mut previous_event: Option<u64> = None;
    let mut pulse_count = 0u64;
    let mut rpm = 0.0f64;

    let tile = singleton!(: dash_lib::TileBuffer = dash_lib::TileBuffer::new())
        .unwrap();
    let rx_scratch = singleton!(: u8 = 0).unwrap();

    let mut dashboard = dash_lib::Dashboard::new();
    dashboard.update(0.0, 0.0);

    let first_rect = dashboard.render_next_tile(tile).unwrap();
    cs.set_low().unwrap();
    dash_lib::begin_tile_write(&mut spi, &mut dc, first_rect).unwrap();

    let mut lcd_transfer = Some(
        bidirectional::Config::new(
            (dma.ch2, dma.ch3),
            dash_lib::TileTx::new(tile),
            spi,
            dash_lib::TileRx::new(rx_scratch, first_rect),
        )
        .start(),
    );
    let mut lcd_idle = None;

    let sda = pins.gpio4.into_pull_up_input().into_function::<FunctionI2c>();
    let scl = pins.gpio5.into_pull_up_input().into_function::<FunctionI2c>();
    let mut pad_bus = I2C::new_controller(
        pac.I2C0,
        sda,
        scl,
        400_000u32.Hz(),
        &mut pac.RESETS,
        clocks.system_clock.freq(),
    );
    let mut pad = qwst_pad::QwstPad::new(qwst_pad::DEFAULT_ADDRESS);
    let pad_ready = pad.init(&mut pad_bus).is_ok();
    let mut pad_errors = 0u32;
    let mut next_pad_us = delay.get_counter().ticks();

    fifo.resume();

    loop {
        // Process ADC data only when a buffer is ready.
        if transfer.is_done() {
            // Because is_done() returned true, wait() returns immediately.
            // DMA is already filling the other buffer.
            let (completed, ongoing) = transfer.wait();

            assert!(!fifo.is_over(), "ADC FIFO overflow");

            for &raw in completed.iter() {
                assert_eq!(raw & 0x8000, 0, "ADC conversion error");

                let sample = raw & 0x0fff;

                if let Some(event_index) =
                    detector.feed(sample, sample_index)
                {
                    pulse_count = pulse_count.wrapping_add(1);

                    if let Some(previous) = previous_event {
                        let interval = event_index - previous;

                        rpm = 60.0 * SAMPLE_RATE_HZ as f64
                            / (interval as f64
                                * IGNITION_EVENTS_PER_REV);
                    }

                    previous_event = Some(event_index);
                }

                sample_index += 1;

                if let Some(previous) = previous_event {
                    if sample_index - previous >= RPM_TIMEOUT_SAMPLES {
                        rpm = 0.0;
                        previous_event = None;
                    }
                }
            }

            // Return the processed buffer before doing display work.
            transfer = ongoing.write_next(completed);

            // Catch an overrun that happened during processing.
            assert!(!fifo.is_over(), "ADC FIFO overflow");
        }

        let now_us = delay.get_counter().ticks();
        if pad_ready && now_us >= next_pad_us {
            match pad.poll(&mut pad_bus, now_us) {
                Ok(pressed) => {
                    use dash_lib::MenuInput;
                    let event = if pressed & (qwst_pad::B | qwst_pad::MINUS) != 0 {
                        Some(MenuInput::Back)
                    } else if pressed & qwst_pad::PLUS != 0 {
                        Some(MenuInput::Open)
                    } else if pressed & qwst_pad::A != 0 {
                        Some(MenuInput::Select)
                    } else {
                        match pressed & (qwst_pad::UP | qwst_pad::DOWN) {
                            qwst_pad::UP => Some(MenuInput::Up),
                            qwst_pad::DOWN => Some(MenuInput::Down),
                            _ => None,
                        }
                    };
                    if let Some(event) = event {
                        dashboard.menu_input(event);
                    }
                }
                Err(_) => {
                    pad_errors = pad_errors.wrapping_add(1);
                    // No navigation event is generated by a failed read.
                }
            }
            // No catch-up polling burst if a loop iteration ran late.
            next_pad_us = delay.get_counter().ticks() + 10_000;
        }
        
        // Recover LCD resources only when BOTH DMA channels have completed.
        if lcd_transfer.as_ref().is_some_and(|t| t.is_done()) {
            let (channels, tx, mut spi, rx) =
                lcd_transfer.take().unwrap().wait();

            // DMA completion and the last bit leaving SPI are distinct events.
            spi.flush().unwrap();
            cs.set_high().unwrap();
            dashboard.finish_tile();

            lcd_idle = Some((
                channels,
                spi,
                tx.into_buffer(),
                rx.into_scratch(),
            ));
        }

        // Keep the existing display schedule, but only update dashboard state.
        let now_us = delay.get_counter().ticks();
        if now_us >= next_frame_us {
            let wheel = dash_lib::wheel_snapshot();
            let sample_now_us = delay.get_counter().ticks();
            let speed_kmh = dash_lib::wheel_speed(wheel, sample_now_us);

            dashboard.update(speed_kmh, rpm);

            let finished_us = delay.get_counter().ticks();
            next_frame_us += DISPLAY_INTERVAL_US;
            if next_frame_us <= finished_us {
                next_frame_us +=
                    ((finished_us - next_frame_us) / DISPLAY_INTERVAL_US + 1)
                    * DISPLAY_INTERVAL_US;
            }
        }

        // Render and launch at most ONE tile, then return to ADC servicing.
        if let Some((channels, mut spi, tile, rx_scratch)) = lcd_idle.take() {
            if let Some(rect) = dashboard.render_next_tile(tile) {
                cs.set_low().unwrap();
                dash_lib::begin_tile_write(&mut spi, &mut dc, rect).unwrap();

                lcd_transfer = Some(
                    bidirectional::Config::new(
                        channels,
                        dash_lib::TileTx::new(tile),
                        spi,
                        dash_lib::TileRx::new(rx_scratch, rect),
                    )
                    .start(),
                );
            } else {
                lcd_idle = Some((channels, spi, tile, rx_scratch));
            }
        }

        // No delay_ms and no loop that drains every dirty tile at once.
    }
}
