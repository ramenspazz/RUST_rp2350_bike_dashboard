<<<<<<< HEAD
# RP2350 USB audio playback prototype

Default firmware: UAC 1.0, stereo 48 kHz S32_LE USB playback through Pico Audio Pack.
See [USB_AUDIO.md](USB_AUDIO.md) for flashing, first playback, architecture, and validation limits.
Use `cargo run --release --features test-tone` to flash the standalone 300 Hz test.

The new USB path builds and passes host-side tests, but awaits physical enumeration
and playback testing. USB VID/PID 1209:0001 is for private testing only.

---

# Tired and lazy
uses rp-hal to create a pio based i2s audio output device. Double surprise, I have reused [code](https://github.com/electronjoe/noise-generator-rust-on-pi-pico/tree/main) that [electronjoe](https://github.com/electronjoe/) reused of [mine](https://github.com/ramenspazz/i2s-audio-rust-rp2040) to make this repo!

# Looking for help!
I am not sure if my [pico audio pack](https://shop.pimoroni.com/products/pico-audio-pack?variant=32369490853971) is not working, or if my code is not working! Any help is appreciated!
=======
# working sample achieved!
uses rp-hal to create a pio based i2s audio output device. Audio output now works and currently, code outputs a 300hz sine wave.
>>>>>>> cd778ce03dfb577d8afb748d07edd9d98ac58106
