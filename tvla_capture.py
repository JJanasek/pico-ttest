## tvla_capture.py
# Non-specific (fixed vs random) TVLA trace collection for TROPIC01 on an ESP32, captured with
# a ChipWhisperer-Husky (default) or a PicoScope 3000/2000/6000 (--scope pico, --pico-series).
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
# Wiring, Pico:   ESP32 trigger GPIO -> Ext on a D model (default), or an analog channel
#                 (--trigger-source B), EM/shunt probe -> the measured channel (--channel, AC
#                 coupled). Ext keeps every channel free, so the measured one gets 1 GS/s.
#                 On a 6000 Series (6404D) "ext" is the AUX BNC: 50 ohm, +-1 V, so feed it
#                 through a series resistor and trigger low (see --trigger-level).
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
# TR01_ECDSA_EDDSA_SIGNATURE_LENGTH: 64 bytes for both Ed25519 and P256.
SIG_LEN = 64
# TR01_L3_IV_SIZE: the 12-byte secure-channel L3 IV reported per sign by firmware >= 3.
NONCE_LEN = 12

# The constant class of each campaign. Fixed but not degenerate (an all-zero scalar or message is
# a special case in more than one implementation and makes a poor "fixed" class).
FIXED_PAYLOAD = bytes.fromhex("da39a3ee5e6b4b0d3255bfef95601890afd80709") + bytes(PAYLOAD_LEN - 20)
# Longest message the capture firmware accepts (TVLA_MAX_MSG_LEN in tvla_target.cpp).
MAX_MSG_LEN = 256


def fixed_payload(msg_len):
    """The fixed-class message at the requested length: FIXED_PAYLOAD, repeated or cut to fit."""
    reps = -(-msg_len // len(FIXED_PAYLOAD))
    return (FIXED_PAYLOAD * reps)[:msg_len]
FIXED_SCALAR = bytes.fromhex("0f1e2d3c4b5a69788796a5b4c3d2e1f0" * 2)

CURVE_NAMES = {"ed": "Ed25519-EdDSA", "ec": "P256-ECDSA"}

CLASS_FIXED = 0
CLASS_RANDOM = 1


def _msg_with_hw(n, hw, rng):
    """A random n-byte message with exact total Hamming weight hw and no zero byte.

    Each byte gets at least one set bit (so HW ranges n..8n), then the remaining set bits are
    scattered at random. The byte patterns are otherwise random, so at a fixed hw only the total
    weight is held constant - several messages at the same hw differ in their bytes, which is what
    tells a pure total-HW leak apart from one that depends on the pattern.
    """
    if not n <= hw <= 8 * n:
        raise ValueError(f"HW {hw} unreachable for {n} bytes with no zero byte (need {n}..{8*n})")
    bits = np.ones(n, dtype=int)
    extra = int(hw) - n
    while extra > 0:                       # add set bits to not-yet-full bytes
        i = int(rng.integers(n))
        if bits[i] < 8:
            bits[i] += 1
            extra -= 1
    out = bytearray()
    for b in bits:
        v = 0
        for p in rng.choice(8, size=int(b), replace=False):
            v |= 1 << int(p)
        out.append(v)
    return bytes(out)


def default_hw_levels(n):
    """Five Hamming-weight levels spread over the no-zero-byte range (1.5n..7.5n)."""
    return [int(round(n * f)) for f in (1.5, 3.0, 4.5, 6.0, 7.5)]


def hw_sweep_set(msg_len, levels, patterns, seed):
    """The --hw-sweep message set: `patterns` random messages at each Hamming-weight level."""
    rng = np.random.default_rng(seed)
    return [_msg_with_hw(msg_len, hw, rng) for hw in levels for _ in range(patterns)]


def hw(msg):
    return sum(bin(b).count("1") for b in msg)


def parse_args():
    p = argparse.ArgumentParser(
        description="Fixed-vs-random TVLA trace collection for TROPIC01 (ESP32 target, "
                    "ChipWhisperer-Husky or PicoScope 3000/2000/6000)"
    )

    p.add_argument("--scope", choices=("husky", "pico"), default="husky",
                   help="capture hardware (default: husky)")

    p.add_argument("-n", "--traces", type=int, default=3000, help="number of traces to collect")
    p.add_argument("-c", "--curve", choices=("ed", "ec"), default="ed",
                   help="ed = Ed25519/EdDSA, ec = P256/ECDSA (default: ed)")
    p.add_argument("-m", "--mode", choices=("message", "scalar"), default="message",
                   help="what the fixed-vs-random split varies: the signed message (default) "
                        "or the secret scalar in the key slot")
    p.add_argument("--priv-key", default=None,
                   help="message mode: store this 32-byte hex private key instead of generating "
                        "one on the chip. The default keygen keeps the private key inside the "
                        "secure element (only the public key is knowable); supplying a known key "
                        "records it, giving ground truth for validating a recovered key")
    p.add_argument("--msg-len", type=int, default=PAYLOAD_LEN,
                   help="message length in bytes (default 32; Ed25519 only, up to 256). The nonce "
                        "TMAC runs per 18 B and the SHA-512 per 128 B, so sweeping the length "
                        "shows which part of the trace is message-dependent")
    p.add_argument("--hw-sweep", action="store_true",
                   help="message mode: instead of fixed-vs-random, cycle through a fixed set of "
                        "messages spanning Hamming weight (several patterns per level, no zero "
                        "bytes). Averaging many traces per message settles whether the message "
                        "leak is total HW (a load/transport) or depends on the byte pattern (a "
                        "diffused, TMAC-style absorption), with far fewer traces than random CPA. "
                        "Records ttest = message index, msg = the bytes signed")
    p.add_argument("--hw-levels", default=None,
                   help="comma-separated target total Hamming weights for --hw-sweep "
                        "(default: 1.5n..7.5n spread over msg_len n)")
    p.add_argument("--hw-patterns", type=int, default=3,
                   help="distinct random patterns per HW level for --hw-sweep (default 3); "
                        "same-HW patterns test whether anything beyond total HW leaks")
    p.add_argument("--hw-seed", type=int, default=0,
                   help="RNG seed for the --hw-sweep message set (default 0; reproducible)")
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
    p.add_argument("--sample-bits", type=int, default=None, choices=(8, 16),
                   help="bits per stored sample (Husky, default: 16; a PicoScope is always 8). "
                        "Husky's ADC is "
                        "12 bit, so 8 discards four of them; 16 keeps the lot at twice the disk. "
                        "It matters most at low gain, where the signal uses little of the ADC "
                        "range and quantisation starts to show")
    p.add_argument("--sign-delay", type=float, default=0.0,
                   help="pause this many ms after every signature. TROPIC01 has gone into alarm "
                        "mode three times, each after a burst of back-to-back signatures, and "
                        "--tiles multiplies that rate by the tile count. Spacing them out is the "
                        "cheap thing to try before concluding the chip cannot sustain a campaign")
    p.add_argument("--tiles", type=int, default=1,
                   help="capture each trace as N consecutive windows and concatenate them "
                        "(Husky, default: 1 = off). Husky holds 131,070 samples however fast the "
                        "ADC runs, so this is the only way to get high bandwidth over a long "
                        "span: every tile signs the SAME payload with the SAME key. Note the "
                        "signature nonce still differs per tile - TROPIC01 folds the secure-"
                        "channel nonce (which increments per L3 command) into it - so the "
                        "message-processing part of each window lines up across tiles but the "
                        "nonce-dependent part (scalar mult, H(R,A,M)) is a fresh execution each "
                        "tile. Costs one signature per tile per trace")
    p.add_argument("--stream", action=argparse.BooleanOptionalAction, default=True,
                   help="stream samples (default) instead of filling Husky's 131,070 sample "
                        "buffer. Streaming keeps decimate at 1 - no aliasing - and lifts the "
                        "depth limit, at up to ~10 MS/s and with no pre-trigger samples. "
                        "--no-stream reverts to the decimated block mode, which is the only way "
                        "to cover the whole signature in one window")

    # PicoScope only.
    p.add_argument("--pico-series", choices=("3000", "2000", "6000"), default="3000",
                   help="PicoScope family: 3000 (ps3000a driver), 2000 (ps2000a driver: 2205A "
                        "MSO, 2206B-2208B, 2405A-2408B) or 6000 (ps6000 driver: 6402-6404 C/D, "
                        "6407; not the 6000E Series)")
    p.add_argument("--channel", type=int, default=0, help="scope channel to measure (0 = A)")
    p.add_argument("--bw-limit", action="store_true",
                   help="PicoScope: switch on the measured channel's bandwidth limiter (the "
                        "GUI's 'Bandwidth limit' option): 20 MHz on the 3000 Series and the "
                        "6402/6403, 25 MHz on the 6404")
    p.add_argument("--volt-div", type=float, default=1e-2,
                   help="volts per division on the measured channel; the range picked is 5x this "
                        "(default: 0.01 = +-50mV, the smallest range a 6404D has)")
    p.add_argument("--coupling", default="AC", choices=("AC", "DC", "DC50"),
                   help="coupling of the measured channel. AC removes the DC bias of a power or "
                        "EM trace so the small range is usable (default: AC). DC50 = DC into "
                        "50 ohm, 6000 Series only - for an amplifier output, never straight "
                        "across a shunt that carries DC")
    p.add_argument("--offset", type=float, default=0.0, help="analog offset in volts")
    p.add_argument("--trigger-source", default="ext", choices=("ext", "A", "B", "C", "D"),
                   help="where the ESP32 trigger GPIO is wired. Ext exists only on 3000 Series D "
                        "models; on A/B models use an analog channel. On the 6000 Series it is "
                        "the AUX input (default: ext)")
    p.add_argument("--trigger-level", type=float, default=None,
                   help="trigger threshold in volts (default: 1.5, mid-rail for 3.3V logic; "
                        "0.15 on the 6000 Series AUX input, which is 50 ohm with a +-1V range "
                        "- feed it through a ~470 ohm series resistor, giving ~0.3V)")
    p.add_argument("--trigger-range", type=float, default=5.0,
                   help="full scale of the trigger channel in volts, DC coupled (default: 5.0)")

    p.add_argument("-v", "--verbose", action="store_true", help="echo the target's '#' log lines")

    g = p.add_argument_group("setup notes",
                             "recorded verbatim in the .trs so a set can be told apart later")
    g.add_argument("--shunt-ohm", type=float, default=None, help="shunt resistor value")
    g.add_argument("--probe", default=None,
                   help="how the signal is picked up, e.g. 'shunt low-side', 'EM probe at X,Y'")
    g.add_argument("--supply-v", type=float, default=None, help="chip supply voltage")
    g.add_argument("--temperature-c", type=float, default=None, help="ambient/board temperature")
    g.add_argument("--spect-fw-rev", default=None,
                   help="revision of the SPECT firmware on the chip (ts-spect-fw git hash/tag)")
    g.add_argument("--chip-note", default=None,
                   help="chip identification: sample number, revision, clock configuration")
    g.add_argument("--marker", default=None,
                   help="what an extra scope channel records, e.g. 'CH2 = SPI CS'")
    g.add_argument("--note", default=None, help="free text")

    args = p.parse_args()
    # The two scopes have nothing like the same capture depth, so their sensible defaults differ:
    # Husky's whole buffer is 131,070 samples, which covers a ~123 ms signature only at 800 kS/s.
    if args.sample_rate is None:
        args.sample_rate = HUSKY_DEFAULT_SAMPLE_RATE if args.scope == "husky" else 12.5e6
    if args.samples is None:
        args.samples = HUSKY_DEFAULT_SAMPLES if args.scope == "husky" else 2_000_000
    # A fixed default of 16 made every --scope pico run fail the check below unless
    # --sample-bits 8 was passed by hand.
    if args.sample_bits is None:
        args.sample_bits = 16 if args.scope == "husky" else 8
    if args.trigger_level is None:
        aux = args.pico_series == "6000" and args.trigger_source == "ext"
        args.trigger_level = 0.15 if aux else 1.5
    if args.coupling == "DC50" and args.pico_series != "6000":
        p.error("--coupling DC50 needs --pico-series 6000; the other series have no 50 ohm input")

    if args.curve == "ec" and args.msg_len != PAYLOAD_LEN:
        p.error("--msg-len is Ed25519 only: ECDSA signs a fixed 32-byte hash")
    if not 1 <= args.msg_len <= MAX_MSG_LEN:
        p.error(f"--msg-len must be 1..{MAX_MSG_LEN}")

    args.hw_sweep_msgs = None
    if args.hw_sweep:
        if args.mode != "message":
            p.error("--hw-sweep varies the message; it needs message mode (not scalar)")
        n = args.msg_len
        if args.hw_levels:
            try:
                levels = [int(x) for x in args.hw_levels.split(",") if x.strip()]
            except ValueError:
                p.error("--hw-levels must be comma-separated integers")
        else:
            levels = default_hw_levels(n)
        for h in levels:
            if not n <= h <= 8 * n:
                p.error(f"--hw-levels: HW {h} unreachable for {n} bytes with no zero byte "
                        f"(need {n}..{8*n})")
        if args.hw_patterns < 1:
            p.error("--hw-patterns must be >= 1")
        args.hw_levels_list = levels
        args.hw_sweep_msgs = hw_sweep_set(n, levels, args.hw_patterns, args.hw_seed)

    if args.priv_key is not None:
        if args.mode != "message":
            p.error("--priv-key is for message mode; scalar mode already sets the key per trace")
        try:
            kb = bytes.fromhex(args.priv_key)
        except ValueError:
            p.error("--priv-key must be hex")
        if len(kb) != PRIVKEY_LEN:
            p.error(f"--priv-key must be {PRIVKEY_LEN} bytes ({2*PRIVKEY_LEN} hex chars)")
        args.priv_key = kb

    if args.tiles < 1:
        p.error("--tiles must be at least 1")
    if args.sample_bits == 16 and args.scope != "husky":
        p.error("--sample-bits 16 is Husky only; the PicoScope path returns signed bytes")
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
    """A long window times many traces adds up fast, doubly so at 16 bits per sample."""
    projected = n_samples * args.traces * (2 if getattr(args, "sample_bits", 8) == 16 else 1)
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


def git_commit():
    """Short hash of the capture code, so a trace set can be tied back to how it was made."""
    try:
        import subprocess
        out = subprocess.run(["git", "-C", os.path.dirname(os.path.abspath(__file__)),
                              "rev-parse", "--short", "HEAD"],
                             capture_output=True, text=True, timeout=5)
        rev = out.stdout.strip()
        return (rev + ("-dirty" if git_changes() else "")) if rev else "unknown"
    except Exception:
        return "unknown"


def git_changes():
    """Tracked files that differ from the commit, so a "-dirty" set says what was different.

    Untracked files (trace sets, notes) do not change how a capture is made and are left out.
    """
    try:
        import subprocess
        out = subprocess.run(["git", "-C", os.path.dirname(os.path.abspath(__file__)),
                              "status", "--porcelain", "--untracked-files=no"],
                             capture_output=True, text=True, timeout=5)
        return [line[3:] for line in out.stdout.splitlines() if line.strip()]
    except Exception:
        return []


def trace_set_params(args, n_samples, pre_trigger, meta):
    """Campaign-level metadata: everything that is the same for every trace in the set.

    Stored as typed TRACE_SET_PARAMETERS rather than crammed into the description string, so an
    analysis script can read the public key, sample rate or gain back as real values. The message
    (and, in scalar mode, the per-trace key) vary per trace and go in each Trace instead.
    """
    from trsfile.parametermap import TraceSetParameterMap
    from trsfile.traceparameter import StringParameter, ByteArrayParameter, DoubleArrayParameter, IntegerArrayParameter

    def s(v): return StringParameter(str(v))
    def b(v): return ByteArrayParameter(list(v))
    def d(v): return DoubleArrayParameter([float(v)])
    def i(v): return IntegerArrayParameter([int(v)])

    p = TraceSetParameterMap()
    p["target"] = s("TROPIC01")
    p["curve"] = s(CURVE_NAMES[args.curve])
    p["mode"] = s(args.mode)
    p["scope"] = s(args.scope)
    p["sample_rate_Sps"] = d(meta.get("sample_rate", args.sample_rate))
    p["samples_per_tile"] = i(args.samples)
    p["tiles"] = i(args.tiles)
    p["total_samples"] = i(n_samples)
    p["pre_trigger_samples"] = i(pre_trigger)
    p["fixed_payload"] = b(fixed_payload(args.msg_len))
    p["msg_len"] = i(args.msg_len)
    if getattr(args, "hw_sweep_msgs", None):
        p["msg_scheme"] = s("hw-sweep: ttest = message index (not fixed-vs-random); msg = the "
                            "signed bytes; Hamming weight = popcount(msg)")
        p["hw_levels"] = s(",".join(str(x) for x in args.hw_levels_list))
        p["hw_patterns"] = i(args.hw_patterns)
        p["hw_seed"] = i(args.hw_seed)
        p["hw_messages"] = s(",".join(m.hex() for m in args.hw_sweep_msgs))
    p["sample_period_s"] = d(1.0 / meta.get("sample_rate", args.sample_rate))
    p["window_s"] = d(n_samples / meta.get("sample_rate", args.sample_rate))
    # Time origin: nothing else in the file says what t=0 is.
    p["time_origin"] = s("sample 0 = {} samples after the trigger edge; trigger is raised by the "
                         "ESP32 just before libtropic's sign call and dropped when it returns, so "
                         "it brackets the whole L3 Sign transaction (secure channel, SPI, key load, "
                         "SPECT, response)".format("-" + str(pre_trigger) if pre_trigger else "0"))
    p["tile_note"] = s("each tile is a separate signature; do not analyse across tile boundaries"
                       if args.tiles > 1 else "single capture per trace")
    p["ecc_slot"] = i(meta.get("slot", 0))
    for key, val in (("shunt_ohm", args.shunt_ohm), ("supply_V", args.supply_v),
                     ("temperature_C", args.temperature_c)):
        if val is not None:
            p[key] = d(val)
    for key, val in (("probe", args.probe), ("spect_fw_rev", args.spect_fw_rev),
                     ("chip_note", args.chip_note), ("marker_channel", args.marker),
                     ("note", args.note)):
        if val:
            p[key] = s(val)

    if args.mode == "message":
        if meta.get("pubkey") is not None:
            p["public_key"] = b(meta["pubkey"])
        if meta.get("privkey") is not None:
            p["private_key"] = b(meta["privkey"])
    if args.mode == "scalar":
        p["fixed_scalar"] = b(FIXED_SCALAR)

    if args.scope == "husky":
        p["gain_dB"] = d(args.gain_db)
        p["gain_mode"] = s(args.gain_mode)
        p["skip_ms"] = d(args.skip_ms)
        p["stream"] = s("stream" if args.stream else "block")
        p["adc_clock_Sps"] = d(args.adc_freq)
        p["sample_bits"] = i(args.sample_bits)
        p["trigger"] = s(args.trigger_pin)
    else:
        p["pico_series"] = s(args.pico_series)
        p["pico_model"] = s(meta.get("scope_model", "unknown"))
        p["channel"] = i(args.channel)
        p["bw_limit"] = s(meta.get("bw_limit", "full"))
        p["volt_per_div"] = d(args.volt_div)
        p["coupling"] = s(args.coupling)
        p["trigger"] = s(str(args.trigger_source))
        p["trigger_level_V"] = d(args.trigger_level)
        if args.trigger_source != "ext":
            p["trigger_range_V"] = d(args.trigger_range)
        p["offset_V"] = d(args.offset)
        if meta.get("scope_serial"):
            p["scope_serial"] = s(meta["scope_serial"])

    p["firmware"] = s(meta.get("fw_version", "unknown"))
    p["capture_git"] = s(meta.get("git", "unknown"))
    if meta.get("git_changes"):
        p["capture_git_changes"] = s(", ".join(meta["git_changes"]))
    p["created_utc"] = s(meta.get("created", ""))
    p["command"] = s(meta.get("command", ""))
    return p


def open_trace_file(args, n_samples, volt_div, time_div, pre_trigger=0, label_y="V",
                    meta=None, path=None):
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
        settings += " {}bit".format(args.sample_bits)
    else:
        settings = ("pico{} ch{} {:.4g}V/div {} {:.3f}MS/s x{} samples"
                    .format(args.pico_series, args.channel, args.volt_div, args.coupling,
                            args.sample_rate / 1e6, args.samples))

    coding = (trsfile.SampleCoding.SHORT if getattr(args, "sample_bits", 8) == 16
              else trsfile.SampleCoding.BYTE)
    full_scale = 32767 if coding is trsfile.SampleCoding.SHORT else 255

    headers = {
        trsfile.Header.TRS_VERSION: 2,
        # The trigger position goes in the description: TRS headers are written unsigned, so the
        # negative OFFSET_X that would express it cannot be stored.
        trsfile.Header.DESCRIPTION:
            f"TROPIC01 {CURVE_NAMES[args.curve]} fixed-vs-random {args.mode} TVLA; "
            f"pre_trigger={int(pre_trigger)} samples; {settings}",
        trsfile.Header.NUMBER_SAMPLES: int(n_samples),
        trsfile.Header.LENGTH_DATA: _trace_param_bytes(args),
        trsfile.Header.SAMPLE_CODING: coding,
        trsfile.Header.LABEL_X: "s",
        trsfile.Header.LABEL_Y: label_y,
        trsfile.Header.SCALE_X: 10 * time_div / n_samples,
        trsfile.Header.SCALE_Y: 10 * volt_div / full_scale,
        trsfile.Header.TRACE_PARAMETER_DEFINITIONS: _trace_param_defs(args),
    }
    if meta is not None:
        headers[trsfile.Header.TRACE_SET_PARAMETERS] = trace_set_params(
            args, n_samples, pre_trigger, meta)
    return path, trsfile.trs_open(path, mode="w", headers=headers)


def _trace_param_bytes(args):
    """Bytes of per-trace data, derived from the definitions so the two cannot disagree."""
    return sum(d.length * d.param_type.byte_size for d in _trace_param_defs(args).values())


def _trace_param_defs(args):
    """Per-trace fields: the class byte, the message signed, and (scalar mode) the key written.

    Storing the message per trace is what turns a t-test-only set into one a real analysis can
    use - without it there is no record of what was actually signed.
    """
    Def = trsfile.traceparameter.TraceParameterDefinition
    T = trsfile.traceparameter.ParameterType
    offset = 0
    defs = {}
    defs["ttest"] = Def(T.BYTE, 1, offset); offset += 1
    defs["msg"] = Def(T.BYTE, args.msg_len, offset); offset += args.msg_len
    defs["sig"] = Def(T.BYTE, SIG_LEN, offset); offset += SIG_LEN
    # Host-side timing of this trace: when it was taken, and how long the sign call took as the
    # host saw it (serial round-trip included). Drift over a campaign and outlier traces show here.
    defs["t_unix"] = Def(T.DOUBLE, 1, offset); offset += 8
    defs["sign_ms"] = Def(T.FLOAT, 1, offset); offset += 4
    # Percentage of this trace's samples sitting on the ADC rails, so a saturated trace (the
    # first one after setup, while AC coupling settles, can be clipped end to end) is found by
    # filtering on a field rather than by eye.
    defs["clip_pct"] = Def(T.FLOAT, 1, offset); offset += 4
    if getattr(args, "has_nonce", False):
        defs["nonce"] = Def(T.BYTE, NONCE_LEN, offset); offset += NONCE_LEN
    if args.mode == "scalar":
        defs["key"] = Def(T.BYTE, PRIVKEY_LEN, offset); offset += PRIVKEY_LEN
    return trsfile.parametermap.TraceParameterDefinitionMap(defs)


def _trace_params(args, trace_class, payload, key, signature, nonce=None, t_unix=0.0,
                  sign_ms=0.0, clip_pct=0.0):
    """Per-trace values matching _trace_param_defs: class, message, signature, nonce, (scalar) key."""
    Byte = trsfile.parametermap.ByteArrayParameter
    # The signature is padded/truncated to the fixed field width; a mismatch means the target
    # returned something unexpected and is worth seeing rather than silently dropping.
    sig = bytes(signature or b"")
    if len(sig) != SIG_LEN:
        sig = (sig + bytes(SIG_LEN))[:SIG_LEN]
    params = {"ttest": Byte([int(trace_class)]), "msg": Byte(list(payload)),
              "sig": Byte(list(sig)),
              "t_unix": trsfile.parametermap.DoubleArrayParameter([float(t_unix)]),
              "sign_ms": trsfile.traceparameter.FloatArrayParameter([float(sign_ms)]),
              "clip_pct": trsfile.traceparameter.FloatArrayParameter([float(clip_pct)])}
    if getattr(args, "has_nonce", False):
        nb = bytes(nonce or b"")
        nb = (nb + bytes(NONCE_LEN))[:NONCE_LEN]
        params["nonce"] = Byte(list(nb))
    if args.mode == "scalar":
        params["key"] = Byte(list(key if key is not None else FIXED_SCALAR))
    return trsfile.parametermap.TraceParameterMap(params)


class TraceWriter:
    """Writes traces to one .trs, or to a series of them when --traces-per-file is set.

    A .trs only becomes readable when it is closed - trsfile finalises the header there, and an
    interrupted file fails to open with "TRS file has an unexpected length". On a campaign that
    runs for hours, rolling to a new file every N traces keeps everything collected so far safe.
    """

    def __init__(self, args, n_samples, volt_div, time_div, pre_trigger=0, label_y="V",
                 meta=None):
        self._args = args
        self._meta = meta
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
        _, self._file = open_trace_file(self._args, *self._header_args,
                                        meta=self._meta, path=path)
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
                       stream=args.stream, skip_ms=args.skip_ms,
                       sample_bits=args.sample_bits)
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
    from pico import PICO_SERIES

    scope = PICO_SERIES[args.pico_series]()
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
        # Fail here, with the reason, rather than on the first arm with PICO_TRIGGER_ERROR.
        if not scope.hasExtTrigger():
            raise ValueError("PicoScope {} has no Ext trigger input - wire the trigger GPIO to "
                             "an analog channel and pass e.g. --trigger-source B"
                             .format(scope.variant))
        trigger_channel = scope.EXT_CHANNEL
    else:
        trigger_channel = "ABCD".index(args.trigger_source)
        if trigger_channel == args.channel:
            raise ValueError("the trigger channel must differ from the measured channel")
        # An analog trigger only fires if its channel is enabled, and it stays DC coupled so a
        # long logic-high pulse does not droop back below the level.
        scope.enableChannel(trigger_channel, rangeVolts=args.trigger_range)

    if args.bw_limit:
        scope.setBandwidthLimit(args.channel)

    scope.setTriggerChannel(trigger_channel, enable=1, level=args.trigger_level)
    # Enabling the trigger channel can move the sample rate, so report what the scope ended up
    # with rather than what setChannel first chose.
    scope.describe(trigger_channel, args.trigger_level, pre_trigger, args.coupling)
    return scope, scope.voltDiv, scope.timeDiv, "V"


def setup_target(args):
    if not args.no_flash:
        PlatformIOProject(args.project_dir, args.env, args.port).upload()

    target = TropicTarget(args.port, args.baud, verbose=args.verbose).open()
    fw_version = target.version()
    print("[*] Target version: " + fw_version)
    # Firmware >= 3 reports the L3 nonce per sign; older firmware does not.
    try:
        args.has_nonce = int(fw_version.split()[-1]) >= 3
    except (ValueError, IndexError):
        args.has_nonce = False
    if not args.has_nonce:
        print("[!] This firmware is older than tvla-target 3, so the per-trace secure-channel "
              "nonce will not be recorded. Flash the repo's firmware/src/tvla_target.cpp "
              f"(the project built is {args.project_dir})")

    pubkey = None
    privkey = None
    if args.mode == "message":
        # One key for the whole campaign; only the message varies.
        if getattr(args, "priv_key", None) is not None:
            target.store_key(args.curve, args.priv_key)
            privkey = args.priv_key
            print(f"[*] Stored known {CURVE_NAMES[args.curve]} private key: {privkey.hex()}")
        else:
            pubkey = target.keygen(args.curve)
            print(f"[*] Generated {CURVE_NAMES[args.curve]} key, public key: {pubkey.hex()}")
    else:
        # The key is rewritten per trace; store the fixed one now so a rejected key or a worn out
        # slot shows up before the campaign rather than 2000 traces in.
        target.store_key(args.curve, FIXED_SCALAR)
        print(f"[*] Stored the fixed {CURVE_NAMES[args.curve]} scalar (rewritten per trace)")
    return target, pubkey, privkey, fw_version


def prepare_trial(args, target, trace_class):
    """Sets the target up for one trace and returns the payload to sign.

    Everything here happens before the scope is armed, so only the signature itself ends up
    inside the trigger window.
    """
    if args.mode == "message":
        payload = (fixed_payload(args.msg_len) if trace_class == CLASS_FIXED
                   else os.urandom(args.msg_len))
        return payload, None

    key = FIXED_SCALAR if trace_class == CLASS_FIXED else os.urandom(PRIVKEY_LEN)
    target.store_key(args.curve, key)
    return fixed_payload(args.msg_len), key


def main():
    args = parse_args()
    start_time = time.time()

    scope = None
    writer = None
    # Resolved before anything is opened, so a bad value fails immediately.
    pre_trigger = preTriggerSamples(args.pre_trigger, args.samples)

    import datetime, shlex, sys as _sys
    target, pubkey, privkey, fw_version = setup_target(args)
    meta = {
        "pubkey": pubkey,
        "privkey": privkey,
        "fw_version": fw_version,
        "git": git_commit(),
        "git_changes": git_changes(),
        "created": datetime.datetime.now(datetime.timezone.utc)
                       .strftime("%Y-%m-%dT%H:%M:%SZ"),
        "command": shlex.join(_sys.argv),
        "sample_rate": args.sample_rate,
        # Slots as in tvla_target.cpp (ECC_SLOT_ED25519 / ECC_SLOT_ECDSA).
        "slot": 1 if args.curve == "ed" else 2,
    }
    try:
        if not args.no_scope:
            check_disk_budget(args, args.samples * args.tiles)
            scope, volt_div, time_div, label_y = setup_scope(args, pre_trigger)
            # The rate the scope actually runs at (quantized to its timebase grid), not the one
            # asked for - sample_period_s and window_s are derived from it.
            meta["sample_rate"] = scope.sampleRate
            meta["scope_model"] = getattr(scope, "variant", "unknown")
            meta["scope_serial"] = getattr(scope, "serial", "")
            # What the limiter really is depends on the model (20 or 25 MHz), not on the flag.
            meta["bw_limit"] = getattr(scope, "bandwidthLimit", "full")
            # time_div describes one tile's window, but SCALE_X is derived as
            # 10 * time_div / total_samples - the .trs convention being ten divisions across the
            # whole trace. Scaling it by the tile count keeps seconds-per-sample at 1/rate;
            # without it the time axis came out compressed by exactly --tiles.
            writer = TraceWriter(args, args.samples * args.tiles, volt_div,
                                 time_div * args.tiles, pre_trigger, label_y, meta=meta)

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

        if scope is not None and args.scope == "pico":
            # The first block after setup can sit on the rail end to end while the AC coupling
            # settles - trace 0 of the first 6404D set was 100% clipped. Take one and discard it.
            try:
                scope.arm(preTrigger=pre_trigger)
                if pre_trigger_wait:
                    time.sleep(pre_trigger_wait)
                target.sign(fixed_payload(args.msg_len))
                scope.getNativeSignalBytes()
                print("[*] Discarded one warm-up capture (AC coupling settling)")
            except TimeoutError as ex:
                print(f"[!] Warm-up capture failed ({ex}); the campaign loop will retry")

        # --hw-sweep: draw each trace's message index from a reshuffled bag of the whole set, so
        # every message gets equal counts and the order is reshuffled each pass (drift hits all).
        msg_schedule = None
        if getattr(args, "hw_sweep_msgs", None):
            _bag = []

            def msg_schedule():
                if not _bag:
                    _bag.extend(int(j) for j in np.random.permutation(len(args.hw_sweep_msgs)))
                return _bag.pop()

            print(f"[*] HW-sweep: {len(args.hw_sweep_msgs)} messages, {args.hw_patterns} "
                  f"patterns x levels {args.hw_levels_list} (ttest = message index):")
            for idx, mm in enumerate(args.hw_sweep_msgs):
                print(f"      [{idx:2d}] HW {hw(mm):3d}  {mm.hex()}")

        while collected < args.traces:
            try:
                if msg_schedule is not None:
                    trace_class = msg_schedule()
                    payload, trace_key = args.hw_sweep_msgs[trace_class], None
                else:
                    # Randomly interleave the two classes so slow drift affects both equally.
                    trace_class = CLASS_FIXED if np.random.randint(0, 2) == 0 else CLASS_RANDOM
                    payload, trace_key = prepare_trial(args, target, trace_class)

                # One signature per tile, all signing the same payload with the same key. The
                # signature nonce differs per tile (TROPIC01 folds the per-command secure-channel
                # nonce into it), so tiling lines up the message-processing windows across tiles
                # while the nonce-dependent parts are a fresh execution each tile - fine for a
                # message-dependent leak, not for one that rides on the nonce.
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
                    signature, nonce = target.sign(payload)
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
                    # execution per tile - so only the arm and read shares are worth attacking
                    # if a campaign is taking too long.
                    per_tile = (" - {:.0f} ms per tile outside the signature".format(
                        1e3 * (phase["arm"] + phase["read"]) / args.tiles)
                        if args.tiles > 1 else "")
                    print("[*] Per trace: {:.2f} s signing (the floor), {:.2f} s arming, "
                          "{:.2f} s reading out{}".format(phase["sign"], phase["arm"],
                                                          phase["read"], per_tile))
                    if sign_ms > window_ms:
                        # Not necessarily a fault: on Husky a partial window is the default,
                        # because dropping the tail is what buys decimate=1 and the resolution
                        # that comes with it. Say what is captured and leave the call to the user.
                        needed = int(sign_ms * 1e-3 * sample_rate)
                        if args.scope == "husky":
                            how = (f" - past Husky's 131,070 sample buffer that needs --stream "
                                   f"(on by default, up to {HUSKY_MAX_STREAM_RATE / 1e6:.0f} MS/s) "
                                   f"or --no-stream with a lower --sample-rate")
                        else:
                            how = " (the trigger pulse itself is ~101 ms; this includes serial)"
                        print(f"[*] So the window holds the first {window_ms:.0f} ms of it. To "
                              f"cover the whole signature, raise --samples to >= {needed:,}{how}.")

                if scope is not None:
                    samples = b"".join(tile_samples)
                    # Both scopes clip their output to the full integer range, so the rails are
                    # the dtype's min and max.
                    dtype = np.int16 if args.sample_bits == 16 else np.int8
                    rails = np.iinfo(dtype)
                    arr = np.frombuffer(samples, dtype=dtype)
                    clip_pct = 100.0 * np.count_nonzero((arr == rails.max) | (arr == rails.min)) / arr.size
                    # sign_ms is the signing alone (summed over tiles). total_ms also holds the
                    # arm and the read-out - seconds for a deep PicoScope block - and recording
                    # that under this name made every trace look like a 3 s signature.
                    writer.append(trsfile.Trace(
                        trsfile.SampleCoding.SHORT if args.sample_bits == 16
                        else trsfile.SampleCoding.BYTE,
                        samples,
                        _trace_params(args, trace_class, payload, trace_key,
                                      signature, nonce, t_unix=sign_start,
                                      sign_ms=phase["sign"] * 1e3, clip_pct=clip_pct),
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
