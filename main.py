from machine import ADC
from array import array
import time
import gc

ADC_PIN = 26
SAMPLE_COUNT = 10_000
ADC_REFERENCE_V = 3.3  # Nominal; not a calibrated measurement.
CSV_FILE = "/adc_capture.csv"

adc = ADC(ADC_PIN)

# Allocate all sample storage before acquisition.
samples = array("H", [0] * SAMPLE_COUNT)
read_adc = adc.read_u16

gc.collect()
gc.disable()

try:
    start_us = time.ticks_us()

    for i in range(SAMPLE_COUNT):
        samples[i] = read_adc()

    elapsed_us = time.ticks_diff(time.ticks_us(), start_us)
finally:
    gc.enable()

# Acquisition has finished. File I/O cannot interrupt this burst.
with open(CSV_FILE, "w") as f:
    f.write("# elapsed_us={}\n".format(elapsed_us))
    f.write("# sample_count={}\n".format(SAMPLE_COUNT))
    f.write("# nominal_reference_v={}\n".format(ADC_REFERENCE_V))
    f.write("sample,raw_u16\n")

    for i in range(SAMPLE_COUNT):
        f.write("{},{}\n".format(i, samples[i]))

minimum_raw = min(samples)
maximum_raw = max(samples)

print("Saved:", CSV_FILE)
print("Capture duration: {} us".format(elapsed_us))

if elapsed_us > 0:
    print("Average throughput: {:.0f} samples/s".format(
        SAMPLE_COUNT * 1_000_000 / elapsed_us
    ))

print("Lowest captured sample: {:.4f} V".format(
    minimum_raw * ADC_REFERENCE_V / 65535
))
print("Highest captured sample: {:.4f} V".format(
    maximum_raw * ADC_REFERENCE_V / 65535
))