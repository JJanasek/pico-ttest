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
    FifoError,
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
    p.add_argument("--sign-delay", type=float, default=0.0,
                   help="pause this many ms after every signature. TROPIC01 has gone into alarm "
                        "mode three times, each after a burst of back-to-back signatures, and "
                        "--tiles multiplies that rate by the tile count. Spacing them out is the "
                        "cheap thing to try before concluding the chip cannot sustain a campaign")
    p.add_argument("--tiles", type=int, default=1,
                   help="capture each trace as N consecutive windows and concatenate them "
                        "(Husky, default: 1 = off). Husky holds 131,070 samples however fast the "
                        "ADC runs, so this is the only way to get high bandwidth over a long "
                        "span: every tile signs the SAME payload with the SAME key, and Ed25519 "
                        "is deterministic, so the tiles observe one computation through "
                        "successive windows. Costs one signature per tile per trace")
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

    if args.tiles < 1:
        p.error("--tiles must be at least 1")
    if args.tiles > 1:
        if args.scope != "husky":
            p.error("--tiles is a Husky-only workaround for its 131,070 sample buffer")
        if args.stream:
            p.error("--tiles needs --no-stream: tiling exists to get high bandwidth out of block "
                    "mode, and streaming already lifts the depth limit at up to 10 MS/s")
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
        if args.tiles > 1:
            settings += " tiles={}x{}".format(args.tiles, args.samples)
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
            check_disk_budget(args, args.samples * args.tiles)
            scope, volt_div, time_div, label_y = setup_scope(args, pre_trigger)
            # time_div describes one tile's window, but SCALE_X is derived as
            # 10 * time_div / total_samples - the .trs convention being ten divisions across the
            # whole trace. Scaling it by the tile count keeps seconds-per-sample at 1/rate;
            # without it the time axis came out compressed by exactly --tiles.
            writer = TraceWriter(args, args.samples * args.tiles, volt_div,
                                 time_div * args.tiles, pre_trigger, label_y)

        collected = 0
        failures = 0
        # Counted separately from the total: the abort is about a run that has stopped making
        # progress, not about a long campaign that collected a few duds along the way.
        consecutive = 0
        # Set while a rebuild has not yet been vindicated by a successful capture.
        rebuilt = False
        sample_rate = scope.sampleRate if scope is not None else args.sample_rate
        window_ms = 1e3 * args.samples * args.tiles / sample_rate
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

                # One signature per tile, all signing the same payload with the same key.
                # Ed25519 derives its nonce from the message, so every repetition is the same
                # computation - which is what makes the windows safe to concatenate.
                tile_samples = []
                phase = {"arm": 0.0, "sign": 0.0, "read": 0.0}
                sign_start = time.time()
                for tile in range(args.tiles):
                    if scope is not None:
                        mark = time.time()
                        if args.tiles > 1:
                            scope.set_tile(tile)
                        scope.arm(preTrigger=pre_trigger)
                        phase["arm"] += time.time() - mark
                        # The scope only honours the trigger once the pre-trigger samples have
                        # been collected. Send the command after that window has elapsed,
                        # otherwise the rising edge lands during the fill and is dropped - and
                        # since the line then stays high for the whole signature, no second edge
                        # ever arrives.
                        if pre_trigger_wait:
                            time.sleep(pre_trigger_wait)

                    tile_start = time.time()
                    target.sign(payload)
                    tile_ms = (time.time() - tile_start) * 1e3
                    phase["sign"] += tile_ms * 1e-3

                    if scope is not None:
                        mark = time.time()
                        chunk, _raw = scope.getNativeSignalBytes()
                        tile_samples.append(chunk)
                        phase["read"] += time.time() - mark

                    # Pace only once the capture has been read out. The ADC keeps sampling until
                    # capture() completes, so a sleep between the signature and the read leaves
                    # it filling a FIFO nobody is draining - at 200 MS/s that overflows within
                    # milliseconds and corrupts every trace.
                    if args.sign_delay:
                        time.sleep(args.sign_delay * 1e-3)
                # Per-signature duration, so the window check below compares like with like even
                # when a trace is assembled from several of them.
                sign_ms = tile_ms
                total_ms = (time.time() - sign_start) * 1e3

                # A signature longer than the capture window means every trace is cut short, which
                # is invisible in the .trs file - say so on the first one rather than after 3000.
                if not window_checked:
                    window_checked = True
                    print(f"[*] Signature takes ~{sign_ms:.0f} ms (host measured, including "
                          f"serial round-trip), capture window is {window_ms:.1f} ms")
                    if args.tiles > 1:
                        print(f"[*] {args.tiles} tiles x {args.samples:,} samples at "
                              f"{sample_rate/1e6:.0f} MS/s = {window_ms:.1f} ms covered, "
                              f"{args.tiles * args.samples:,} samples per trace "
                              f"({args.tiles} signatures, {total_ms/1e3:.1f} s per trace)")
                        # Where the per-trace time actually goes. Signing is the floor - one
                        # execution per tile - so only the arm and read shares are worth
                        # attacking if a campaign is taking too long.
                        print("[*] Per trace: {:.1f} s signing (the floor), {:.1f} s arming, "
                              "{:.1f} s reading out - {:.0f} ms per tile outside the signature"
                              .format(phase["sign"], phase["arm"], phase["read"],
                                      1e3 * (phase["arm"] + phase["read"]) / args.tiles))
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
                    samples = b"".join(tile_samples)
                    writer.append(trsfile.Trace(
                        trsfile.SampleCoding.BYTE,
                        samples,
                        trsfile.parametermap.TraceParameterMap(
                            {"ttest": trsfile.parametermap.ByteArrayParameter([trace_class])}
                        ),
                    ))

                # Incremented only on success, so a dropped capture is retried rather than lost.
                collected += 1
                consecutive = 0
                rebuilt = False
                if collected % 100 == 0:
                    print(f"    {collected}/{args.traces}")
            except Exception as ex:
                failures += 1
                consecutive += 1
                print(f"ERROR ({failures}): {ex}")

                # ALARM is latched in TROPIC01's hardware and no serial command clears it -
                # lt_reboot documents itself as a power-cycle equivalent yet still re-reports
                # alarm afterwards (libtropic.c:573). Only dropping the chip's 3.3 V helps, so
                # every retry is guaranteed to fail; a 5000 trace run burned 501 of them.
                if "LT_L1_CHIP_ALARM_MODE" in str(ex):
                    raise RuntimeError(
                        "TROPIC01 latched ALARM mode and cannot recover in software. Power-cycle "
                        "the chip - unplug the ESP32's USB so the shield's 3.3 V drops - then "
                        "restart. Completed --traces-per-file parts are already closed and "
                        "readable. If this keeps cutting long runs short, --sign-delay lowers the "
                        "duty cycle, which is the cheap thing to try."
                    ) from ex

                # An overrun wedges the capture datapath, and a bare re-arm then fails forever -
                # which is how one bad window turns into a whole run of identical errors. Rebuild
                # the scope before retrying rather than hammering a dead device.
                if scope is not None and isinstance(ex, FifoError):
                    # A rebuild that is immediately followed by another FIFO error means the
                    # datapath is wedged, not merely upset: reconnecting re-runs the FPGA
                    # register setup but does not reset the analog/USB path, so retrying can only
                    # fail the same way. Say so once rather than grinding through every retry.
                    if rebuilt:
                        raise RuntimeError(
                            "Husky failed again immediately after a reconnect, so its capture "
                            "datapath is wedged - reconnecting only resets FPGA registers. "
                            "Unplug it, plug it back in, and restart. If this keeps happening, "
                            "lower --sample-rate: 200 MS/s block mode leaves no FIFO drain "
                            "margin and has wedged after a few hundred captures."
                        ) from ex
                    print("[*] Rebuilding the scope connection after the FIFO error")
                    try:
                        scope.disconnect()
                        scope, _vd, _td, _ly = setup_scope(args, pre_trigger)
                        rebuilt = True
                    except Exception as rex:
                        raise RuntimeError(
                            "Husky did not come back after a FIFO error - a reconnect only resets "
                            "FPGA registers, so if the capture datapath is wedged it needs a "
                            "physical power cycle. Unplug it, plug it back in, and restart."
                        ) from rex

                if consecutive > max(20, args.traces // 10):
                    raise RuntimeError(
                        f"aborting after {consecutive} failures in a row ({failures} total)"
                    ) from ex

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
