## tvla_capture.py
# Non-specific (fixed vs random) TVLA trace collection for TROPIC01 on an ESP32, captured with
# a ChipWhisperer-Husky (default) or a PicoScope 3000 (--scope pico).
#
# Two campaigns, selected with --mode:
#   message  the key stays fixed, the signed message is either FIXED or fresh RANDOM,
#   scalar   the message stays fixed, the secret scalar written into the key slot before the
#            signature is either FIXED or fresh RANDOM.
#
# Flow per trace:
#   1. (scalar mode only) write the class's private key into the slot - outside the trigger window,
#   2. arm the scope (triggered by TVLA_TRIGGER_PIN on the ESP32),
#   3. sign the class's payload; the target raises the trigger for exactly the duration of the
#      libtropic sign call,
#   4. read the block back and append it to a .trs file, tagged with its class (0 = fixed,
#      1 = random) so a Welch t-test can split the set afterwards.
#
# Wiring, Husky:  ESP32 trigger GPIO -> 20-pin pin 16 (TIO4), ESP32 GND -> pin 17/19,
#                 shunt -> MEASURE SMA (differential across a high-side shunt, single-ended
#                 plus the short-circuit cap for a low-side one).
# Wiring, Pico:   ESP32 trigger GPIO -> a scope channel (--trigger-source, Ext is unreliable
#                 here), EM/shunt probe -> the measured channel (--channel, AC coupled).
#
# This replaces pico3000.py, which drove a ChipWhisperer target over simpleserial and flashed it
# with make + cw.program_target.
import argparse
import os
import shutil
import time

import numpy as np
import trsfile

# chipwhisperer is imported inside HuskyScope.connect(), so this stays cheap when --scope pico.
from husky import (
    DEFAULT_ADC_FREQ as HUSKY_DEFAULT_ADC_FREQ,
    DEFAULT_GAIN_DB as HUSKY_DEFAULT_GAIN_DB,
    DEFAULT_SAMPLE_RATE as HUSKY_DEFAULT_SAMPLE_RATE,
    DEFAULT_SAMPLES as HUSKY_DEFAULT_SAMPLES,
    DEFAULT_SKIP_MS as HUSKY_DEFAULT_SKIP_MS,
    MAX_STREAM_RATE as HUSKY_MAX_STREAM_RATE,
    DEFAULT_TRIGGER_PIN as HUSKY_DEFAULT_TRIGGER_PIN,
    TRIGGER_PINS as HUSKY_TRIGGER_PINS,
    HuskyScope,
)
from tropic_target import (
    DEFAULT_BAUD,
    DEFAULT_ENV,
    DEFAULT_PORT,
    DEFAULT_PROJECT_DIR,
    PRIVKEY_LEN,
    PlatformIOProject,
    TropicTarget,
)

# Payload length. ECDSA on TROPIC01 signs a message hash and requires exactly 32 bytes; Ed25519
# takes an arbitrary message, and 32 bytes keeps the two campaigns comparable.
PAYLOAD_LEN = 32

# The constant class of each campaign. Fixed but not degenerate (an all-zero scalar or message is
# a special case in more than one implementation and makes a poor "fixed" class).
FIXED_PAYLOAD = bytes.fromhex("da39a3ee5e6b4b0d3255bfef95601890afd80709") + bytes(PAYLOAD_LEN - 20)
FIXED_SCALAR = bytes.fromhex("0f1e2d3c4b5a69788796a5b4c3d2e1f0" * 2)

CURVE_NAMES = {"ed": "Ed25519-EdDSA", "ec": "P256-ECDSA"}

CLASS_FIXED = 0
CLASS_RANDOM = 1


def parse_args():
    p = argparse.ArgumentParser(
        description="Fixed-vs-random TVLA trace collection for TROPIC01 (ESP32 target, "
                    "ChipWhisperer-Husky or PicoScope 3000)"
    )

    p.add_argument("--scope", choices=("husky", "pico"), default="husky",
                   help="capture hardware (default: husky)")

    p.add_argument("-n", "--traces", type=int, default=3000, help="number of traces to collect")
    p.add_argument("-c", "--curve", choices=("ed", "ec"), default="ed",
                   help="ed = Ed25519/EdDSA, ec = P256/ECDSA (default: ed)")
    p.add_argument("-m", "--mode", choices=("message", "scalar"), default="message",
                   help="what the fixed-vs-random split varies: the signed message (default) "
                        "or the secret scalar in the key slot")
    p.add_argument("-o", "--outdir", default="traces", help="directory for the .trs file")
    p.add_argument("--traces-per-file", type=int, default=0,
                   help="roll over to a new .trs every N traces (0 = one file). A .trs is only "
                        "readable once closed, so on a long campaign this bounds what an "
                        "interrupted run loses")

    # Target / firmware.
    p.add_argument("--project-dir", default=DEFAULT_PROJECT_DIR, help="PlatformIO project path")
    p.add_argument("--env", default=DEFAULT_ENV, help="PlatformIO environment to build and flash")
    p.add_argument("--port", default=DEFAULT_PORT, help="serial port of the ESP32")
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="serial baud rate")
    p.add_argument("--no-flash", action="store_true",
                   help="skip the PlatformIO build/upload and talk to the firmware already on the board")

    # Acquisition, both scopes. The window must cover a whole signature (~123 ms measured);
    # the run reports the measured duration on the first trace.
    p.add_argument("--sample-rate", type=float, default=None,
                   help="sample rate in S/s (default: 10M on Husky, 12.5M on a PicoScope)")
    p.add_argument("--samples", type=lambda s: int(float(s)), default=None,
                   help="samples per trace. Husky defaults to 500,000 = the first 50 ms of "
                        "the ~101 ms signature at 10 MS/s; a PicoScope to 2,000,000 = 160 ms at "
                        "12.5 MS/s, covering the whole of it")
    p.add_argument("--pre-trigger", default="0",
                   help="samples captured before the trigger edge: a count, a fraction (0.1) or a "
                        "percentage of --samples (10%%). Default: 0")
    p.add_argument("--no-scope", action="store_true",
                   help="drive the target without capturing (useful while setting the scope up)")

    # Husky. One AC-coupled MEASURE input, so there is no channel, coupling or volts/division to
    # choose - the LNA gain sets the range instead.
    p.add_argument("--gain-db", type=float, default=HUSKY_DEFAULT_GAIN_DB,
                   help="Husky LNA gain in dB, -6.5 to 55 (default: %(default)s). The capture "
                        "warns when the ADC clips or when the trace uses too little of its range")
    p.add_argument("--gain-mode", choices=("high", "low"), default="high",
                   help="Husky LNA gain mode (default: high)")
    p.add_argument("--trigger-pin", default=HUSKY_DEFAULT_TRIGGER_PIN, choices=HUSKY_TRIGGER_PINS,
                   help="Husky input the ESP32 trigger GPIO is wired to: tio1-4 and nrst are on "
                        "the 20-pin header (tio4 is pin 16), aux is the AUX MCX "
                        "(default: %(default)s)")
    p.add_argument("--adc-freq", type=float, default=HUSKY_DEFAULT_ADC_FREQ,
                   help="Husky ADC clock in Hz; --sample-rate is reached by decimating from it "
                        "(default: %(default)s)")
    p.add_argument("--skip-ms", type=float, default=HUSKY_DEFAULT_SKIP_MS,
                   help="discard this many ms after the trigger before recording (Husky, "
                        "default: %(default)s). The "
                        "trigger GPIO's own edge couples into the measurement and clips the ADC; "
                        "skipping past it lets --gain-db be set for the signature instead")
    p.add_argument("--stream", action=argparse.BooleanOptionalAction, default=True,
                   help="stream samples (default) instead of filling Husky's 131,070 sample "
                        "buffer. Streaming keeps decimate at 1 - no aliasing - and lifts the "
                        "depth limit, at up to ~10 MS/s and with no pre-trigger samples. "
                        "--no-stream reverts to the decimated block mode, which is the only way "
                        "to cover the whole signature in one window")

    # PicoScope only.
    p.add_argument("--channel", type=int, default=0, help="scope channel to measure (0 = A)")
    p.add_argument("--volt-div", type=float, default=1e-2,
                   help="volts per division on the measured channel; the range picked is 5x this "
                        "(default: 0.01 = +-50mV)")
    p.add_argument("--coupling", default="AC", choices=("AC", "DC"),
                   help="coupling of the measured channel. AC removes the DC bias of a power or "
                        "EM trace so the small range is usable (default: AC)")
    p.add_argument("--offset", type=float, default=0.0, help="analog offset in volts")
    p.add_argument("--trigger-source", default="ext", choices=("ext", "A", "B", "C", "D"),
                   help="where the ESP32 trigger GPIO is wired. Ext exists only on 3000 Series D "
                        "models; on A/B models use an analog channel (default: ext)")
    p.add_argument("--trigger-level", type=float, default=1.5,
                   help="trigger threshold in volts (default: 1.5, mid-rail for 3.3V logic)")
    p.add_argument("--trigger-range", type=float, default=5.0,
                   help="full scale of the trigger channel in volts, DC coupled (default: 5.0)")

    p.add_argument("-v", "--verbose", action="store_true", help="echo the target's '#' log lines")

    args = p.parse_args()
    # The two scopes have nothing like the same capture depth, so their sensible defaults differ:
    # Husky's whole buffer is 131,070 samples, which covers a ~123 ms signature only at 800 kS/s.
    if args.sample_rate is None:
        args.sample_rate = HUSKY_DEFAULT_SAMPLE_RATE if args.scope == "husky" else 12.5e6
    if args.samples is None:
        args.samples = HUSKY_DEFAULT_SAMPLES if args.scope == "husky" else 2_000_000
    return args


def preTriggerSamples(spec, n_samples):
    """Resolves --pre-trigger: '10%' or '0.1' are fractions of the window, anything else a count."""
    text = str(spec).strip()
    if text.endswith("%"):
        fraction = float(text[:-1]) / 100.0
    else:
        value = float(text)
        if value >= 1 or value == 0:
            fraction = None
        else:
            fraction = value
        if fraction is None:
            samples = int(value)
            if not 0 <= samples < n_samples:
                raise ValueError(f"--pre-trigger {spec} is not within the {n_samples} sample window")
            return samples
    if not 0 <= fraction < 1:
        raise ValueError(f"--pre-trigger {spec} must be under 100% of the window")
    return int(round(fraction * n_samples))


def check_disk_budget(args, n_samples):
    """Traces are one byte per sample, so a long window times many traces adds up fast."""
    projected = n_samples * args.traces
    free = shutil.disk_usage(os.path.abspath(args.outdir) if os.path.isdir(args.outdir)
                             else os.path.dirname(os.path.abspath(args.outdir)) or ".").free
    gib = 1024 ** 3
    print("[*] Trace file will be about {:.1f} GiB ({:,} samples x {:,} traces), {:.1f} GiB free"
          .format(projected / gib, n_samples, args.traces, free / gib))
    if projected > free:
        raise RuntimeError(
            "not enough free space: {:.1f} GiB needed, {:.1f} GiB available. Reduce --traces, "
            "--samples or --sample-rate.".format(projected / gib, free / gib))
    if projected > 0.8 * free:
        print("[!] That is over 80% of the free space on this filesystem.")


def open_trace_file(args, n_samples, volt_div, time_div, pre_trigger=0, label_y="V", path=None):
    os.makedirs(args.outdir, exist_ok=True)
    if path is None:
        stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        path = os.path.join(args.outdir,
                            f"TROPIC01_{args.curve}_{args.mode}_{args.traces}_{stamp}.trs")

    # The acquisition settings go in the description because nothing else in the .trs records
    # them: two runs of the same campaign at different gains produce files that look nothing
    # alike, and without this there is no way to tell them apart afterwards.
    if args.scope == "husky":
        settings = ("husky gain={:.1f}dB/{} {:.3f}MS/s x{} samples skip={:.2f}ms {} trig={}"
                    .format(args.gain_db, args.gain_mode, args.sample_rate / 1e6, args.samples,
                            args.skip_ms, "stream" if args.stream else "block",
                            args.trigger_pin))
    else:
        settings = ("pico ch{} {:.4g}V/div {} {:.3f}MS/s x{} samples"
                    .format(args.channel, args.volt_div, args.coupling,
                            args.sample_rate / 1e6, args.samples))

    headers = {
        trsfile.Header.TRS_VERSION: 2,
        # The trigger position goes in the description: TRS headers are written unsigned, so the
        # negative OFFSET_X that would express it cannot be stored.
        trsfile.Header.DESCRIPTION:
            f"TROPIC01 {CURVE_NAMES[args.curve]} fixed-vs-random {args.mode} TVLA; "
            f"pre_trigger={int(pre_trigger)} samples; {settings}",
        trsfile.Header.NUMBER_SAMPLES: int(n_samples),
        trsfile.Header.LENGTH_DATA: 1,
        trsfile.Header.SAMPLE_CODING: trsfile.SampleCoding.BYTE,
        trsfile.Header.LABEL_X: "s",
        trsfile.Header.LABEL_Y: label_y,
        trsfile.Header.SCALE_X: 10 * time_div / n_samples,
        trsfile.Header.SCALE_Y: 10 * volt_div / np.iinfo(np.uint8).max,
        trsfile.Header.TRACE_PARAMETER_DEFINITIONS: trsfile.parametermap.TraceParameterDefinitionMap(
            {"ttest": trsfile.traceparameter.TraceParameterDefinition(
                trsfile.traceparameter.ParameterType.BYTE, 1, 0)}
        ),
    }
    return path, trsfile.trs_open(path, mode="w", headers=headers)


class TraceWriter:
    """Writes traces to one .trs, or to a series of them when --traces-per-file is set.

    A .trs only becomes readable when it is closed - trsfile finalises the header there, and an
    interrupted file fails to open with "TRS file has an unexpected length". On a campaign that
    runs for hours, rolling to a new file every N traces keeps everything collected so far safe.
    """

    def __init__(self, args, n_samples, volt_div, time_div, pre_trigger=0, label_y="V"):
        self._args = args
        self._header_args = (n_samples, volt_div, time_div, pre_trigger, label_y)
        self._per_file = args.traces_per_file
        self._stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        self._file = None
        self._index = 0
        self._in_file = 0
        self.paths = []
        self._roll()

    def _path(self):
        name = f"TROPIC01_{self._args.curve}_{self._args.mode}_{self._args.traces}_{self._stamp}"
        if self._per_file:
            name += f"_part{self._index:03d}"
        return os.path.join(self._args.outdir, name + ".trs")

    def _roll(self):
        self.close()
        path = self._path()
        _, self._file = open_trace_file(self._args, *self._header_args, path=path)
        self.paths.append(path)
        self._index += 1
        self._in_file = 0
        print("[*] Writing " + path)

    def append(self, trace):
        if self._per_file and self._in_file >= self._per_file:
            self._roll()
        self._file.append(trace)
        self._in_file += 1

    def close(self):
        if self._file is not None:
            self._file.close()
            self._file = None


def setup_scope(args, pre_trigger=0):
    """Connects and configures the capture hardware.

    Returns (scope, volt_div, time_div, label_y). The last three only feed the .trs header:
    a PicoScope trace is in volts, a Husky one is a fraction of ADC full scale (what that is in
    volts depends on the LNA gain and on the probe, so it is not recorded as volts).
    """
    if args.scope == "husky":
        return setup_husky(args, pre_trigger)
    return setup_pico(args, pre_trigger)


def setup_husky(args, pre_trigger=0):
    scope = HuskyScope(gain_db=args.gain_db, gain_mode=args.gain_mode,
                       trigger_pin=args.trigger_pin, adc_freq=args.adc_freq,
                       stream=args.stream, skip_ms=args.skip_ms)
    scope.connect()
    volt_div, time_div, sample_rate = scope.setChannel(args.sample_rate, args.samples, pre_trigger)
    scope.setTriggerChannel(enable=1)

    print("Scope settings:"
          "\n\tgain: {:.1f} dB ({} mode), AC coupled MEASURE input"
          "\n\tsampleRate: {:e}\n\ttimeDiv: {:e}"
          "\n\twindow: {:.3f} ms ({:.3f} ms before the trigger, {:.3f} ms after)"
          .format(args.gain_db, args.gain_mode, sample_rate, time_div,
                  1e3 * args.samples / sample_rate, 1e3 * pre_trigger / sample_rate,
                  1e3 * (args.samples - pre_trigger) / sample_rate))
    if sample_rate != args.sample_rate:
        print("[*] Requested {:.3f} MS/s, running at {:.3f} MS/s (the ADC clock has to be an "
              "integer multiple of it)".format(args.sample_rate / 1e6, sample_rate / 1e6))
    return scope, volt_div, time_div, "ADC full scale"


def setup_pico(args, pre_trigger=0):
    # picosdk is a separate install from chipwhisperer, so it is only pulled in when asked for.
    from pico import PS3000A_EXTERNAL, pico3000

    scope = pico3000()
    scope.connect()
    volt_div, time_div, sample_rate = scope.setChannel(
        args.channel, args.volt_div, args.sample_rate,
        n_points=args.samples, offset=args.offset, coupling=args.coupling
    )
    print("Scope settings:"
          "\n\tvoltDiv: {:e} ({} coupled)\n\tvoltRange: {}\n\ttimeDiv: {:e}\n\tsampleRate: {:e}"
          "\n\twindow: {:.3f} ms ({:.3f} ms before the trigger, {:.3f} ms after)"
          .format(volt_div, args.coupling, scope.voltRange, time_div, sample_rate,
                  1e3 * args.samples / sample_rate, 1e3 * pre_trigger / sample_rate,
                  1e3 * (args.samples - pre_trigger) / sample_rate))
    if args.trigger_source == "ext":
        trigger_channel = PS3000A_EXTERNAL
    else:
        trigger_channel = "ABCD".index(args.trigger_source)
        if trigger_channel == args.channel:
            raise ValueError("the trigger channel must differ from the measured channel")
        # An analog trigger only fires if its channel is enabled, and it stays DC coupled so a
        # long logic-high pulse does not droop back below the level.
        scope.enableChannel(trigger_channel, rangeVolts=args.trigger_range)

    scope.setTriggerChannel(trigger_channel, enable=1, level=args.trigger_level)
    return scope, volt_div, time_div, "V"


def setup_target(args):
    if not args.no_flash:
        PlatformIOProject(args.project_dir, args.env, args.port).upload()

    target = TropicTarget(args.port, args.baud, verbose=args.verbose).open()
    print("[*] Target version: " + target.version())

    if args.mode == "message":
        # One key for the whole campaign; only the message varies.
        pubkey = target.keygen(args.curve)
        print(f"[*] Generated {CURVE_NAMES[args.curve]} key, public key: {pubkey.hex()}")
    else:
        # The key is rewritten per trace; store the fixed one now so a rejected key or a worn out
        # slot shows up before the campaign rather than 2000 traces in.
        target.store_key(args.curve, FIXED_SCALAR)
        print(f"[*] Stored the fixed {CURVE_NAMES[args.curve]} scalar (rewritten per trace)")
    return target


def prepare_trial(args, target, trace_class):
    """Sets the target up for one trace and returns the payload to sign.

    Everything here happens before the scope is armed, so only the signature itself ends up
    inside the trigger window.
    """
    if args.mode == "message":
        return FIXED_PAYLOAD if trace_class == CLASS_FIXED else os.urandom(PAYLOAD_LEN)

    key = FIXED_SCALAR if trace_class == CLASS_FIXED else os.urandom(PRIVKEY_LEN)
    target.store_key(args.curve, key)
    return FIXED_PAYLOAD


def main():
    args = parse_args()
    start_time = time.time()

    scope = None
    writer = None
    # Resolved before anything is opened, so a bad value fails immediately.
    pre_trigger = preTriggerSamples(args.pre_trigger, args.samples)

    target = setup_target(args)
    try:
        if not args.no_scope:
            check_disk_budget(args, args.samples)
            scope, volt_div, time_div, label_y = setup_scope(args, pre_trigger)
            writer = TraceWriter(args, args.samples, volt_div, time_div, pre_trigger, label_y)

        collected = 0
        failures = 0
        sample_rate = scope.sampleRate if scope is not None else args.sample_rate
        window_ms = 1e3 * args.samples / sample_rate
        window_checked = False
        # How long the scope needs to fill its pre-trigger buffer, plus a margin.
        pre_trigger_wait = (pre_trigger / sample_rate) * 1.1 + 0.005 if pre_trigger else 0.0
        if pre_trigger_wait:
            print(f"[*] Holding each command back {pre_trigger_wait*1e3:.0f} ms so the trigger "
                  f"edge falls after the pre-trigger window")

        while collected < args.traces:
            try:
                # Randomly interleave the two classes so slow drift affects both equally.
                trace_class = CLASS_FIXED if np.random.randint(0, 2) == 0 else CLASS_RANDOM
                payload = prepare_trial(args, target, trace_class)

                if scope is not None:
                    scope.arm(preTrigger=pre_trigger)
                    # The scope only honours the trigger once the pre-trigger samples have been
                    # collected. Send the command after that window has elapsed, otherwise the
                    # rising edge lands during the fill and is dropped - and since the line then
                    # stays high for the whole signature, no second edge ever arrives.
                    if pre_trigger_wait:
                        time.sleep(pre_trigger_wait)

                sign_start = time.time()
                target.sign(payload)
                sign_ms = (time.time() - sign_start) * 1e3

                # A signature longer than the capture window means every trace is cut short, which
                # is invisible in the .trs file - say so on the first one rather than after 3000.
                if not window_checked:
                    window_checked = True
                    print(f"[*] Signature takes ~{sign_ms:.0f} ms (host measured, including "
                          f"serial round-trip), capture window is {window_ms:.0f} ms")
                    if sign_ms > window_ms:
                        # Not necessarily a fault: on Husky a partial window is the default,
                        # because dropping the tail is what buys decimate=1 and the resolution
                        # that comes with it. Say what is captured and leave the call to the user.
                        print(f"[*] So the window holds the first {window_ms:.0f} ms of it. To "
                              f"cover the whole signature, raise --samples to "
                              f">= {int(sign_ms * 1e-3 * args.sample_rate):,} - past Husky's "
                              f"131,070 sample buffer that needs --stream (on by default, up to "
                              f"{HUSKY_MAX_STREAM_RATE / 1e6:.0f} MS/s) or --no-stream with a "
                              f"lower --sample-rate.")

                if scope is not None:
                    samples, _raw = scope.getNativeSignalBytes()
                    writer.append(trsfile.Trace(
                        trsfile.SampleCoding.BYTE,
                        samples,
                        trsfile.parametermap.TraceParameterMap(
                            {"ttest": trsfile.parametermap.ByteArrayParameter([trace_class])}
                        ),
                    ))

                # Incremented only on success, so a dropped capture is retried rather than lost.
                collected += 1
                if collected % 100 == 0:
                    print(f"    {collected}/{args.traces}")
            except Exception as ex:
                failures += 1
                print(f"ERROR ({failures}): {ex}")
                if failures > max(20, args.traces // 10):
                    raise RuntimeError("too many consecutive failures, aborting") from ex

        print(f"Done: {collected} traces, {failures} retries")
        print(f"Total time: {time.time() - start_time:.1f}s")
        if writer is not None:
            for path in writer.paths:
                print("TRSFILE: " + path)
    finally:
        # Closing is what makes the files readable, so it happens even on an aborted run.
        if writer is not None:
            writer.close()
        if scope is not None:
            scope.disconnect()
        target.close()


if __name__ == "__main__":
    main()
