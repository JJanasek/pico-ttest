## trigger_check.py
# Diagnostic for "scope did not trigger": captures the trigger line itself, untriggered, while the
# target performs one signature, and reports what the scope actually saw.
#
# It answers, in order:
#   1. does a pulse reach the scope at all,
#   2. how high does it get (is it above the trigger level),
#   3. how long is it, and where in the window does it start.
#
# Run it with the trigger wire on the channel you pass as --channel (default B, matching
# tvla_capture.py --trigger-source B). Ground the probe to the ESP32 ground.
import argparse
import time

import numpy as np

from pico import pico3000
from tropic_target import DEFAULT_BAUD, DEFAULT_PORT, TropicTarget

PAYLOAD_LEN = 32


def parse_args():
    p = argparse.ArgumentParser(
        description="Check whether the ESP32 trigger pulse reaches the PicoScope")
    p.add_argument("--channel", default="B", choices=("A", "B", "C", "D"),
                   help="channel the trigger wire is on (default: B)")
    p.add_argument("--curve", choices=("ed", "ec"), default="ed", help="curve to sign with")
    p.add_argument("--port", default=DEFAULT_PORT, help="serial port of the ESP32")
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="serial baud rate")
    p.add_argument("--level", type=float, default=1.5, help="trigger level to compare against")
    p.add_argument("--window-ms", type=float, default=500.0,
                   help="capture window in ms; must comfortably exceed one signature")
    p.add_argument("--sample-rate", type=float, default=1e6,
                   help="sample rate in S/s (1 MS/s is plenty to see a ms-scale pulse)")
    p.add_argument("-v", "--verbose", action="store_true", help="echo the target's '#' lines")
    return p.parse_args()


def main():
    args = parse_args()
    channel = "ABCD".index(args.channel)
    n_samples = int(args.window_ms * 1e-3 * args.sample_rate)

    target = TropicTarget(args.port, args.baud, verbose=args.verbose).open()
    print("[*] Target version: " + target.version())
    target.keygen(args.curve)

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
        target.close()


if __name__ == "__main__":
    main()
