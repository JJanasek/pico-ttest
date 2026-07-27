## tvla_capture.py
# Non-specific (fixed vs random) TVLA trace collection for TROPIC01 on an ESP32, captured with
# a PicoScope 3000.
#
# Two campaigns, selected with --mode:
#   message  the key stays fixed, the signed message is either FIXED or fresh RANDOM,
#   scalar   the message stays fixed, the secret scalar written into the key slot before the
#            signature is either FIXED or fresh RANDOM.
#
# Flow per trace:
#   1. (scalar mode only) write the class's private key into the slot - outside the trigger window,
#   2. arm the scope (trigger = EXT, driven by TVLA_TRIGGER_PIN on the ESP32),
#   3. sign the class's payload; the target raises the trigger for exactly the duration of the
#      libtropic sign call,
#   4. read the block back and append it to a .trs file, tagged with its class (0 = fixed,
#      1 = random) so a Welch t-test can split the set afterwards.
#
# Wiring: ESP32 GPIO25 -> PicoScope EXT trigger input, EM/shunt probe -> channel A.
#
# This replaces pico3000.py, which drove a ChipWhisperer target over simpleserial and flashed it
# with make + cw.program_target.
import argparse
import os
import time

import numpy as np
import trsfile

from pico import PS3000A_EXTERNAL, pico3000
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
        description="Fixed-vs-random TVLA trace collection for TROPIC01 (ESP32 target, PicoScope 3000)"
    )

    p.add_argument("-n", "--traces", type=int, default=3000, help="number of traces to collect")
    p.add_argument("-c", "--curve", choices=("ed", "ec"), default="ed",
                   help="ed = Ed25519/EdDSA, ec = P256/ECDSA (default: ed)")
    p.add_argument("-m", "--mode", choices=("message", "scalar"), default="message",
                   help="what the fixed-vs-random split varies: the signed message (default) "
                        "or the secret scalar in the key slot")
    p.add_argument("-o", "--outdir", default="traces", help="directory for the .trs file")

    # Target / firmware.
    p.add_argument("--project-dir", default=DEFAULT_PROJECT_DIR, help="PlatformIO project path")
    p.add_argument("--env", default=DEFAULT_ENV, help="PlatformIO environment to build and flash")
    p.add_argument("--port", default=DEFAULT_PORT, help="serial port of the ESP32")
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="serial baud rate")
    p.add_argument("--no-flash", action="store_true",
                   help="skip the PlatformIO build/upload and talk to the firmware already on the board")

    # Scope. Defaults assume a signature takes well under 80 ms; check with a single trace first.
    p.add_argument("--channel", type=int, default=0, help="scope channel to measure (0 = A)")
    p.add_argument("--volt-div", type=float, default=2e-1, help="volts per division")
    p.add_argument("--offset", type=float, default=0.0, help="analog offset in volts")
    p.add_argument("--sample-rate", type=float, default=12.5e6, help="sample rate in S/s")
    p.add_argument("--samples", type=int, default=2_000_000,
                   help="samples per trace. The default is a 160 ms window at 12.5 MS/s, sized to "
                        "cover one ~123 ms TROPIC01 signature (2 MB per trace on disk)")
    p.add_argument("--pre-trigger", type=int, default=0, help="samples captured before the trigger")
    p.add_argument("--trigger-source", default="ext", choices=("ext", "A", "B", "C", "D"),
                   help="where the ESP32 trigger GPIO is wired. Ext exists only on 3000 Series D "
                        "models; on A/B models use an analog channel (default: ext)")
    p.add_argument("--trigger-level", type=float, default=1.5,
                   help="trigger threshold in volts (default: 1.5, mid-rail for 3.3V logic)")
    p.add_argument("--no-scope", action="store_true",
                   help="drive the target without capturing (useful while setting the scope up)")

    p.add_argument("-v", "--verbose", action="store_true", help="echo the target's '#' log lines")
    return p.parse_args()


def open_trace_file(args, n_samples, volt_div, time_div):
    os.makedirs(args.outdir, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    path = os.path.join(args.outdir,
                        f"TROPIC01_{args.curve}_{args.mode}_{args.traces}_{stamp}.trs")

    headers = {
        trsfile.Header.TRS_VERSION: 2,
        trsfile.Header.DESCRIPTION:
            f"TROPIC01 {CURVE_NAMES[args.curve]} fixed-vs-random {args.mode} TVLA",
        trsfile.Header.NUMBER_SAMPLES: int(n_samples),
        trsfile.Header.LENGTH_DATA: 1,
        trsfile.Header.SAMPLE_CODING: trsfile.SampleCoding.BYTE,
        trsfile.Header.LABEL_X: "s",
        trsfile.Header.LABEL_Y: "V",
        trsfile.Header.SCALE_X: 10 * time_div / n_samples,
        trsfile.Header.SCALE_Y: 10 * volt_div / np.iinfo(np.uint8).max,
        trsfile.Header.TRACE_PARAMETER_DEFINITIONS: trsfile.parametermap.TraceParameterDefinitionMap(
            {"ttest": trsfile.traceparameter.TraceParameterDefinition(
                trsfile.traceparameter.ParameterType.BYTE, 1, 0)}
        ),
    }
    return path, trsfile.trs_open(path, mode="w", headers=headers)


def setup_scope(args):
    scope = pico3000()
    scope.connect()
    volt_div, time_div, sample_rate = scope.setChannel(
        args.channel, args.volt_div, args.sample_rate,
        n_points=args.samples, offset=args.offset
    )
    print("Scope settings:"
          "\n\tvoltDiv: {:e}\n\tvoltRange: {}\n\ttimeDiv: {:e}\n\tsampleRate: {:e}"
          "\n\twindow: {:.3f} ms".format(volt_div, scope.voltRange, time_div, sample_rate,
                                         1e3 * args.samples / sample_rate))
    if args.trigger_source == "ext":
        trigger_channel = PS3000A_EXTERNAL
    else:
        trigger_channel = "ABCD".index(args.trigger_source)
        if trigger_channel == args.channel:
            raise ValueError("the trigger channel must differ from the measured channel")
        # An analog trigger only fires if its channel is enabled; 3.3V logic needs a +-5V range.
        scope.enableChannel(trigger_channel, rangeVolts=5.0)

    scope.setTriggerChannel(trigger_channel, enable=1, level=args.trigger_level)
    return scope, volt_div, time_div


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
    trace_file = None
    trace_path = None

    target = setup_target(args)
    try:
        if not args.no_scope:
            scope, volt_div, time_div = setup_scope(args)
            trace_path, trace_file = open_trace_file(args, args.samples, volt_div, time_div)
            print("[*] Writing " + trace_path)

        collected = 0
        failures = 0
        window_ms = 1e3 * args.samples / (scope.sampleRate if scope is not None else args.sample_rate)
        window_checked = False

        while collected < args.traces:
            try:
                # Randomly interleave the two classes so slow drift affects both equally.
                trace_class = CLASS_FIXED if np.random.randint(0, 2) == 0 else CLASS_RANDOM
                payload = prepare_trial(args, target, trace_class)

                if scope is not None:
                    scope.arm(preTrigger=args.pre_trigger)

                sign_start = time.time()
                target.sign(payload)
                sign_ms = (time.time() - sign_start) * 1e3

                # A signature longer than the capture window means every trace is cut short, which
                # is invisible in the .trs file - say so on the first one rather than after 3000.
                if not window_checked:
                    window_checked = True
                    print(f"[*] Signature takes ~{sign_ms:.0f} ms, capture window is "
                          f"{window_ms:.0f} ms")
                    if sign_ms > window_ms:
                        print(f"[!] The window is shorter than a signature - traces will be "
                              f"truncated. Raise --samples (>= {int(sign_ms * 1e-3 * args.sample_rate):,}) "
                              f"or lower --sample-rate.")

                if scope is not None:
                    samples, _raw = scope.getNativeSignalBytes()
                    trace_file.append(trsfile.Trace(
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
        if trace_path:
            print("TRSFILE: " + trace_path)
    finally:
        if trace_file is not None:
            trace_file.close()
        if scope is not None:
            scope.disconnect()
        target.close()


if __name__ == "__main__":
    main()
