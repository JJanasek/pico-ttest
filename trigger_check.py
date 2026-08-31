## trigger_check.py
# Diagnostic for "scope did not trigger": watches the trigger line while the target performs one
# signature, and reports what the capture hardware actually saw.
#
# On a Husky the trigger is a logic input, so the check is direct: poll the pin's state during a
# signature (does it go high, for how long), then arm for real and see whether the edge fires.
#
# On a PicoScope the trigger is an analog channel, so it captures that channel untriggered and
# answers, in order:
#   1. does a pulse reach the scope at all,
#   2. how high does it get (is it above the trigger level),
#   3. how long is it, and where in the window does it start.
#
# Husky:  run it with the trigger wire on --trigger-pin (default tio4 = 20-pin header pin 16),
#         and the ESP32 ground on pin 17 or 19.
# Pico:   run it with the trigger wire on the channel you pass as --channel (default B, matching
#         tvla_capture.py --trigger-source B). Ground the probe to the ESP32 ground.
import argparse
import threading
import time

import numpy as np

from husky import DEFAULT_TRIGGER_PIN as HUSKY_DEFAULT_TRIGGER_PIN
from husky import MAX_BLOCK_SAMPLES as HUSKY_MAX_BLOCK_SAMPLES
from husky import TRIGGER_PINS as HUSKY_TRIGGER_PINS
from husky import HuskyScope
from tropic_target import DEFAULT_BAUD, DEFAULT_PORT, TropicTarget

PAYLOAD_LEN = 32


def parse_args():
    p = argparse.ArgumentParser(
        description="Check whether the ESP32 trigger pulse reaches the capture hardware")
    p.add_argument("--scope", choices=("husky", "pico"), default="husky",
                   help="capture hardware (default: husky)")
    p.add_argument("--trigger-pin", default=HUSKY_DEFAULT_TRIGGER_PIN, choices=HUSKY_TRIGGER_PINS,
                   help="Husky input the trigger wire is on; tio4 is 20-pin header pin 16 "
                        "(default: %(default)s)")
    p.add_argument("--channel", default="B", choices=("A", "B", "C", "D"),
                   help="PicoScope channel the trigger wire is on (default: B)")
    p.add_argument("--curve", choices=("ed", "ec"), default="ed", help="curve to sign with")
    p.add_argument("--port", default=DEFAULT_PORT, help="serial port of the ESP32")
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="serial baud rate")
    p.add_argument("--level", type=float, default=1.5, help="trigger level to compare against")
    p.add_argument("--window-ms", type=float, default=500.0,
                   help="capture window in ms; must comfortably exceed one signature")
    p.add_argument("--sample-rate", type=float, default=1e6,
                   help="PicoScope sample rate in S/s (1 MS/s is plenty to see a ms-scale "
                        "pulse). Husky derives its rate from --window-ms instead, spending its "
                        "whole 131,070 sample buffer on the window")
    p.add_argument("-v", "--verbose", action="store_true", help="echo the target's '#' lines")
    return p.parse_args()


def main():
    args = parse_args()
    target = TropicTarget(args.port, args.baud, verbose=args.verbose).open()
    print("[*] Target version: " + target.version())
    target.keygen(args.curve)
    try:
        if args.scope == "husky":
            check_husky(args, target)
        else:
            check_pico(args, target)
    finally:
        target.close()


def check_husky(args, target):
    """Two passes: read the pin's level during a signature, then arm and see if the edge fires."""
    scope = HuskyScope(trigger_pin=args.trigger_pin, timeout=args.window_ms * 2e-3 + 5.0)
    scope.connect()
    # The window only has to outlast one signature; the samples themselves are not looked at, so
    # spend the whole buffer on --window-ms rather than honouring --sample-rate (which at its
    # 1 MS/s default would ask for four times more samples than Husky can hold).
    n_samples = HUSKY_MAX_BLOCK_SAMPLES
    scope.setChannel(n_samples / (args.window_ms * 1e-3), n_samples)
    scope.setTriggerChannel()

    try:
        # Pass 1: is the line actually moving? Only the tio pins have a readable state; nrst and
        # the AUX MCX do not, so for those the arm-and-fire pass below is the whole answer.
        if args.trigger_pin.startswith("tio"):
            index = int(args.trigger_pin[-1]) - 1
            samples = []
            timing = {}

            def sign_once():
                started = time.time()
                try:
                    target.sign(bytes(PAYLOAD_LEN))
                except Exception as ex:  # noqa: BLE001 - reported below, on the main thread
                    # A target-side failure must not take the polling results down with it: what
                    # the trigger line did is still worth reporting, and the error itself would
                    # otherwise only surface as a thread traceback.
                    timing["error"] = ex
                timing["ms"] = (time.time() - started) * 1e3

            print(f"[*] Polling {args.trigger_pin} while the target signs")
            worker = threading.Thread(target=sign_once)
            worker.start()
            start = time.time()
            while worker.is_alive():
                samples.append((time.time() - start, scope.scope.io.tio_states[index]))
            worker.join()

            high = [t for t, state in samples if state]
            if "error" in timing:
                print(f"\n[!] The target failed this signature: {timing['error']}")
                print("    The trigger reading below still describes what the line did.")
            print(f"\n    signature took        {timing['ms']:.1f} ms (host measured)")
            print(f"    {args.trigger_pin} polled          {len(samples)} times "
                  f"({len(samples) / max(timing['ms'] * 1e-3, 1e-9):.0f} reads/s)")
            if not high:
                print(f"    -> {args.trigger_pin} never went high.")
                print("       The GPIO is not driving, is not wired to that pin, the grounds are "
                      "not common, or the pin is contended (TROPIC01's INT line sits on GPIO4).")
            else:
                print(f"    -> High from {high[0] * 1e3:.1f} ms to {high[-1] * 1e3:.1f} ms into "
                      f"the signature ({(high[-1] - high[0]) * 1e3:.1f} ms wide).")

        # Pass 2: the real thing - arm, sign, and see whether the rising edge started a capture.
        print("\n[*] Arming and signing once")
        scope.arm()
        started = time.time()
        target.sign(bytes(PAYLOAD_LEN))
        sign_ms = (time.time() - started) * 1e3
        try:
            scope.getNativeSignalBytes()
            triggered = True
        except TimeoutError:
            triggered = False

        print(f"\n    signature took        {sign_ms:.1f} ms (host measured)")
        print(f"    window captured       {1e3 * n_samples / scope.sampleRate:.1f} ms "
              f"({n_samples:,} samples at {scope.sampleRate / 1e6:.3f} MS/s)")
        # trig_count ticks in the ADC clock domain, not in decimated samples: measured against
        # the poll above, 2,024,296 counts at a 20 MHz ADC clock came to 101.2 ms against a
        # polled width of 101.0 ms.
        trig_cycles = scope.scope.adc.trig_count
        trig_ms = 1e3 * trig_cycles / scope.scope.clock.adc_freq
        print(f"    trigger high for      {trig_ms:.1f} ms ({trig_cycles:,} ADC clocks) - this, "
              f"not the host-measured time, is what the window must cover")
        if triggered:
            print(f"\n    -> The trigger works; capture with --trigger-pin {args.trigger_pin}.")
            if trig_ms > 1e3 * n_samples / scope.sampleRate:
                print("       The signature is longer than the window, so traces would be "
                      "truncated - lower --sample-rate or raise --samples.")
        else:
            print("\n    -> Armed, but no rising edge arrived within the timeout.")
            print("       Check the wiring against pass 1 above; a line that reads high the whole "
                  "time never produces an edge to trigger on.")
    finally:
        scope.disconnect()


def check_pico(args, target):
    from pico import pico3000

    channel = "ABCD".index(args.channel)
    n_samples = int(args.window_ms * 1e-3 * args.sample_rate)

    scope = pico3000()
    scope.connect()
    # Measure the trigger line directly: +-5V range (voltsPerDivision * 5), untriggered capture.
    scope.setChannel(channel, 1.0, args.sample_rate, n_points=n_samples)
    scope.setTriggerChannel(channel, enable=0)
    full_scale = scope.channelRanges[channel]

    print(f"[*] Capturing {args.window_ms:.0f} ms of channel {args.channel} at "
          f"{scope.sampleRate/1e6:.2f} MS/s while the target signs")
    try:
        scope.arm()
        t0 = time.time()
        target.sign(bytes(PAYLOAD_LEN))
        sign_ms = (time.time() - t0) * 1e3
        _samples, raw = scope.getNativeSignalBytes()

        volts = raw.astype(np.float32) / scope.maxADC.value * full_scale
        above = volts > args.level
        dt_ms = 1e3 / scope.sampleRate

        print(f"\n    signature took        {sign_ms:.1f} ms (host measured)")
        print(f"    window captured       {n_samples * dt_ms:.1f} ms")
        print(f"    level seen            min {volts.min():+.2f} V, max {volts.max():+.2f} V, "
              f"mean {volts.mean():+.2f} V")
        print(f"    samples above {args.level:.2f} V   {above.sum()} "
              f"({above.sum() * dt_ms:.1f} ms)")

        if not above.any():
            print(f"\n    -> No pulse above {args.level:.2f} V.")
            if volts.max() < 0.1:
                print("       The line is flat: the GPIO is not driving, is not connected to this "
                      "channel, or the probe ground is missing.")
                print("       If the trigger GPIO is one TROPIC01 also drives (its INT pin sits "
                      "on GPIO4), the two outputs fight - move the trigger to a free pin.")
            else:
                suggested = round(float(volts.max()) * 0.5, 2)
                print(f"       There IS a pulse, peaking at {volts.max():.2f} V - it just never "
                      f"reaches the {args.level:.2f} V level.")
                # A 3.3V logic level arriving at ~0.33V is the classic x10 probe signature.
                if 0.2 < volts.max() < 0.6:
                    print(f"       {volts.max():.2f} V is about 3.3 V / 10: this looks like a x10 "
                          "probe. Either switch the probe to x1, or keep x10 and lower the level.")
                print(f"       Re-run with --level {suggested}, then capture with "
                      f"--trigger-source {args.channel} --trigger-level {suggested}.")
        else:
            first = int(np.argmax(above))
            last = len(above) - 1 - int(np.argmax(above[::-1]))
            print(f"\n    -> Pulse found: rises at {first * dt_ms:.1f} ms into the window, "
                  f"width ~{(last - first + 1) * dt_ms:.1f} ms, peak {volts.max():.2f} V.")
            print(f"       The trigger works; capture with --trigger-source {args.channel} "
                  f"--trigger-level {args.level}.")
            if (last - first + 1) * dt_ms > args.window_ms * 0.9:
                print("       Note: the pulse fills the window, so it may extend past it - "
                      "raise --window-ms to measure its true width.")
    finally:
        scope.disconnect()


if __name__ == "__main__":
    main()
