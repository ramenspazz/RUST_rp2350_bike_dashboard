# USB audio playback prototype

Based on upstream commit **447e9ad** (the working audio version).
The default build receives stereo PCM from USB and plays it through Pico Audio
Pack. The previous 300 Hz tone is retained behind the `test-tone` feature.
This is a bring-up prototype; physical USB enumeration/playback has not been
verified here. It is not a USB compliance certification or a tested release.

## What the computer sees

- Product name: **RP2350 USB Audio**.
- USB Audio Class **1.0**, playback only (no microphone).
- Full-speed USB, fixed **48,000 Hz**, two channels, signed **32-bit little-endian** PCM.
- An asynchronous isochronous OUT endpoint, plus an explicit feedback IN endpoint.
- No hardware USB volume/mute control; use the computer's software mixer.

UAC version, USB bus speed, and PCM word size are separate choices. UAC 1.0
supports this PCM format; UAC 2.0 is not required just to send 32-bit samples.
The 32-bit transport does not imply 32 bits of effective analog DAC precision.
The 48 kHz sample rate is a selected operating point, not the maximum waveform
frequency and not a claim about the hardware's maximum supported sample rate.

At 48 kHz, each millisecond normally contains 48 stereo frames:
48 * 2 channels * 4 bytes = **384 bytes**. Endpoint capacity is 392 bytes,
allowing a 49-frame packet when feedback asks the host to catch up. Smaller
whole-frame packets are accepted too. USB OUT means computer-to-device; USB IN
means device-to-computer, so the feedback endpoint is IN.

## Apply and build

Apply `usb-audio-447e9ad.patch` to commit 447e9ad or a compatible working tree.
The patch replaces the misleading mute comments as well as adding USB playback.
Local amplitude-only changes in `i2s_lib.rs` may require merging a patch hunk.

```sh
git apply --check /path/to/usb-audio-447e9ad.patch
git apply /path/to/usb-audio-447e9ad.patch
cargo build --release --locked
```

Hold BOOTSEL while reconnecting USB, release it, and flash with the configured
picotool runner:

```sh
cargo run --release --locked
```

The application then re-enumerates as **RP2350 USB Audio**. It is silent until
selected as an output device and sent audio. Use a USB data cable.
There is no automatic BOOTSEL-reset service in this application; use BOOTSEL
again before flashing subsequent builds.

To return to the known test tone:

```sh
cargo run --release --locked --features test-tone
```

That build intentionally does not enumerate as a USB audio device. Reflash
without the feature to restore USB playback.

## First playback test (Linux)

Start at low listening volume. The included test-file generator writes a 5%
peak-amplitude, 48 kHz S32_LE WAV: two seconds in the left channel, then two in
the right, with fades at each boundary.

```sh
aplay -l
python3 tools/make_usb_test.py /tmp/pico-audio-test.wav
aplay -D hw:CARD=N,DEV=0 /tmp/pico-audio-test.wav
```

Replace **N** with the numeric ALSA card index shown for RP2350 USB Audio.
`hw:` bypasses the desktop mixer; the generated WAV itself has a low amplitude.
If ALSA reports the device is busy, stop desktop playback or select it in the
desktop sound settings and play the WAV through the desktop instead.

For normal desktop use, select RP2350 USB Audio as the output device. A desktop
audio server can convert source music to the device's fixed 48 kHz format.
On other operating systems, inspect audio-device settings for the new output;
compatibility remains to be tested on each host.

If it does not appear, collect:

```sh
lsusb -d 1209:0001
lsusb -v -d 1209:0001
aplay -l
```

If present but silent, first check the selected output and headphone jack, then
reflash the tone build to separate the analog/I2S path from USB problems. The
LCD shows static configuration only, not measured packet reception or DMA health.
The existing LCD initialization must succeed before USB starts.

## How the firmware works

1. `usb_audio.rs` supplies the descriptors that identify AudioControl and
   AudioStreaming interfaces, stereo PCM format, 48 kHz rate, and endpoints.
2. The host selects alternate setting 1 to start streaming (setting 0 is idle).
3. `main.rs` polls USB continuously and receives packets without blocking.
4. `audio_buffer.rs` converts little-endian channel words into a stereo ring.
5. Two 48-frame DMA buffers feed PIO. One plays while software prepares the next.
6. PIO keeps the existing 48 kHz/3.072 MHz LRCLK/BCLK and signed 32-bit I2S slots.

USB packets arrive in bursts, whereas the DAC requires evenly spaced samples.
The ring prebuffers four milliseconds; two DMA buffers add about two milliseconds.
It outputs zeros while idle or rebuffering and resets the queue on stream changes,
USB reset, and suspend/resume state changes. On stop, up to two milliseconds of
already queued audio can still play. Error recovery can cause audible gaps.

### Clock feedback

The USB host and Pico have independent clocks. Always sending exactly 48 frames
per USB millisecond would eventually empty or overfill the queue if those clocks
differ, even though both sides call the rate 48 kHz.

The feedback endpoint sends a **3-byte 10.14 fixed-point** value in stereo frames
per USB millisecond. Nominal 48 is `0x0c0000`, transmitted `00 00 0c`.
A smoothed queue-fill controller asks for slightly more samples when the queue
is low and slightly fewer when it is high. It targets 192 frames remaining after
each refill, with correction bounded to +/-0.125 frame/ms. It does not resample
or alter the DAC clock. This is an occupancy-based controller, not direct SOF
clock measurement; tuning and real-host feedback behavior need hardware testing.

The main loop must run promptly: USB is polled, and each DMA completion must be
serviced within roughly one millisecond. Do not insert blocking LCD writes,
delays, or lengthy synthesis into it. Larger UI/DSP tasks will need a different
scheduling design (for example, interrupts or a second core).

No TinyUSB C integration is required. This uses `rp235x-hal` for the USB hardware,
`usb-device` for enumeration/control transfers, and a focused Rust UAC1 class.
The inspected `usbd-audio` implementation did not offer the required S32 format
and explicit feedback path, so it is not added as a dependency.

## Validation and limitations

- Both default USB and `test-tone` ARM release builds pass.
- Host tests exercise the actual `usb-device` control-transfer stack with a mock
  bus: descriptor lengths/endpoint association, alternate settings, fixed-rate
  requests, reset, and feedback byte encoding.
- Ring tests cover stereo order, negative samples, wraparound, 47/49-frame packets,
  malformed packets, overflow/underflow, and 60-second drift simulations at
  +/-100 and +/-1000 ppm with 16 ms of host feedback delay.
- These tests do not replace enumeration, timing, and sustained playback tests on
  a physical Pico. USB scheduling, electrical behavior, hotplug, suspend power,
  and host-specific driver behavior are not certified by the mock tests.

Run the host suite on an x86-64 Linux development machine:

```sh
cargo test --manifest-path tools/usb-tests/Cargo.toml --target x86_64-unknown-linux-gnu --locked
```

Use your Rust host target on other architectures. The explicit target overrides
this repository's embedded build default. The ring includes wrapping counters
for underruns, overruns, and malformed packets; they are not yet exposed on the LCD.

USB identity **1209:0001** is pid.codes' shared private-test ID, not a unique product
allocation. Do not ship or redistribute devices using it. The prototype serial
string is fixed; multiple boards need distinct serials in a later revision.

## References

- USB Audio 1.0: https://www.usb.org/sites/default/files/audio10.pdf
- Audio data formats 1.0: https://www.usb.org/sites/default/files/frmts10.pdf
- USB stack: https://docs.rs/usb-device/0.3.2/usb_device/
- TinyUSB UAC1 descriptor reference: https://github.com/hathach/tinyusb/blob/master/src/device/usbd.h
- Private-test USB ID: https://pid.codes/1209/0001/
