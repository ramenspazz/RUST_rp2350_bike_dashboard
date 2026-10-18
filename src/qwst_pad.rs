//! Minimal Qw/ST Pad driver using embedded-hal 1.0; no heap allocation.
//! Register/pin mapping: https://github.com/pimoroni/qwstpad-micropython/blob/main/src/qwstpad.py
use embedded_hal::i2c::I2c;

pub const DEFAULT_ADDRESS: u8 = 0x21;
pub const UP: u16 = 1 << 1;
pub const DOWN: u16 = 1 << 4;
pub const LEFT: u16 = 1 << 2;
pub const RIGHT: u16 = 1 << 3;
pub const A: u16 = 1 << 14;
pub const B: u16 = 1 << 12;
pub const X: u16 = 1 << 15;
pub const Y: u16 = 1 << 13;
pub const PLUS: u16 = 1 << 11;
pub const MINUS: u16 = 1 << 5;
const BUTTON_MASK: u16 = UP | DOWN | LEFT | RIGHT | A | B | X | Y | PLUS | MINUS;
const DEBOUNCE_US: u64 = 20_000;

pub struct QwstPad {
    address: u8,
    debounce: Debouncer,
}
impl QwstPad {
    pub const fn new(address: u8) -> Self {
        Self {
            address,
            debounce: Debouncer::new(),
        }
    }
    /// Configure once before ADC capture starts. Errors are returned, not panicked on.
    pub fn init<I: I2c>(&mut self, i2c: &mut I) -> Result<(), I::Error> {
        // Preload LEDs off before enabling their output drivers.
        i2c.write(self.address, &[0x02, 0xc0, 0x06])?;
        // Invert active-low buttons so a pressed button reads as 1.
        i2c.write(self.address, &[0x04, 0x3f, 0xf8])?;
        // LED pins are outputs, all other pins inputs.
        i2c.write(self.address, &[0x06, 0x3f, 0xf9])?;
        self.debounce = Debouncer::new();
        Ok(())
    }
    pub fn read_buttons<I: I2c>(&self, i2c: &mut I) -> Result<u16, I::Error> {
        let mut bytes = [0; 2];
        i2c.write_read(self.address, &[0x00], &mut bytes)?;
        Ok(u16::from_le_bytes(bytes) & BUTTON_MASK)
    }
    /// Return only newly pressed, debounced buttons. Holding a key does not repeat.
    /// First successful read establishes a baseline, suppressing startup-held keys.
    pub fn poll<I: I2c>(&mut self, i2c: &mut I, now_us: u64) -> Result<u16, I::Error> {
        match self.read_buttons(i2c) {
            Ok(raw) => Ok(self.debounce.feed(raw, now_us)),
            Err(e) => {
                self.debounce = Debouncer::new();
                Err(e)
            }
        }
    }
}
struct Debouncer {
    initialized: bool,
    candidate: u16,
    stable: u16,
    since: [u64; 16],
}
impl Debouncer {
    const fn new() -> Self {
        Self {
            initialized: false,
            candidate: 0,
            stable: 0,
            since: [0; 16],
        }
    }
    fn feed(&mut self, raw: u16, now: u64) -> u16 {
        if !self.initialized {
            self.initialized = true;
            self.candidate = raw;
            self.stable = raw;
            self.since.fill(now);
            return 0;
        }
        let changed = self.candidate ^ raw;
        self.candidate = raw;
        let mut presses = 0;
        for bit in 0..16 {
            let mask = 1u16 << bit;
            if changed & mask != 0 {
                self.since[bit] = now;
            }
            if (self.stable ^ raw) & mask != 0 && now.saturating_sub(self.since[bit]) >= DEBOUNCE_US
            {
                self.stable = (self.stable & !mask) | (raw & mask);
                presses |= raw & mask;
            }
        }
        presses
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn bounce_hold_release_and_startup() {
        let mut d = Debouncer::new();
        assert_eq!(d.feed(A, 0), 0); // held at startup
        assert_eq!(d.feed(0, 10_000), 0);
        assert_eq!(d.feed(0, 30_000), 0);
        assert_eq!(d.feed(UP, 40_000), 0);
        assert_eq!(d.feed(0, 45_000), 0);
        assert_eq!(d.feed(UP, 50_000), 0);
        assert_eq!(d.feed(UP, 69_999), 0);
        assert_eq!(d.feed(UP, 70_000), UP);
        assert_eq!(d.feed(UP, 900_000), 0);
        assert_eq!(d.feed(0, 910_000), 0);
        assert_eq!(d.feed(0, 930_000), 0);
        assert_eq!(d.feed(UP, 940_000), 0);
        assert_eq!(d.feed(UP, 960_000), UP);
    }
}
