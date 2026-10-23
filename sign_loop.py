## sign_loop.py
# Drives the target through signatures in a loop without touching any scope, so the capture can
# be set up by hand in the PicoScope GUI (or any other instrument) while the trigger pulses.
#
# Uses the same firmware, key setup and payloads as tvla_capture.py, so what the GUI shows is what
# a campaign records. In the GUI trigger on the same input the campaign would (Ext / AUX, or the
# channel GPIO4 is on), rising edge; the pulse brackets one whole Sign (~101 ms).
#
#   python sign_loop.py                      # sign the fixed message until Ctrl-C
#   python sign_loop.py -n 50 --class random # 50 signatures of fresh random messages
#   python sign_loop.py --flash              # build and flash the firmware first
import argparse
import time

import numpy as np

from tropic_target import (
    DEFAULT_BAUD,
    DEFAULT_ENV,
    DEFAULT_PORT,
    DEFAULT_PROJECT_DIR,
    PRIVKEY_LEN,
    TargetError,
)
from tvla_capture import (
    CLASS_FIXED,
    CLASS_RANDOM,
    CURVE_NAMES,
    FIXED_SCALAR,
    MAX_MSG_LEN,
    PAYLOAD_LEN,
    fixed_payload,
    prepare_trial,
    setup_target,
)

# Consecutive failed signatures before giving up: one is worth retrying, a run of them means the
# chip is in alarm mode or the link is gone.
MAX_CONSECUTIVE_FAILURES = 3


def parse_args():
    p = argparse.ArgumentParser(
        description="Sign in a loop without capturing, for setting a scope up by hand")
    p.add_argument("-n", "--count", type=int, default=0,
                   help="number of signatures (default: 0 = until Ctrl-C)")
    p.add_argument("-c", "--curve", choices=("ed", "ec"), default="ed",
                   help="ed = Ed25519/EdDSA, ec = P256/ECDSA (default: ed)")
    p.add_argument("-m", "--mode", choices=("message", "scalar"), default="message",
                   help="what --class varies, as in tvla_capture.py: the signed message "
                        "(default) or the secret scalar in the key slot")
    p.add_argument("--class", dest="trace_class", choices=("fixed", "random", "mix"),
                   default="fixed",
                   help="fixed = the same input every time, so successive captures overlay "
                        "(default); random = a fresh one each time; mix = random 50/50, like a "
                        "campaign")
    p.add_argument("--msg-len", type=int, default=PAYLOAD_LEN,
                   help="message length in bytes (default 32; Ed25519 only, up to 256)")
    p.add_argument("--priv-key", default=None,
                   help="message mode: store this 32-byte hex private key instead of generating one")
    p.add_argument("--sign-delay", type=float, default=200.0,
                   help="pause this many ms after every signature (default: %(default)s). Gives "
                        "the GUI time to read out and re-arm, and keeps the rate below the "
                        "bursts that have put TROPIC01 into alarm mode")

    p.add_argument("--flash", action="store_true",
                   help="build and flash the firmware first (default: use what is on the board)")
    p.add_argument("--project-dir", default=DEFAULT_PROJECT_DIR, help="PlatformIO project path")
    p.add_argument("--env", default=DEFAULT_ENV, help="PlatformIO environment to build and flash")
    p.add_argument("--port", default=DEFAULT_PORT, help="serial port of the ESP32")
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="serial baud rate")
    p.add_argument("-v", "--verbose", action="store_true",
                   help="echo the target's '#' log lines and print each signature")

    args = p.parse_args()
    # setup_target() and prepare_trial() take tvla_capture's argument names.
    args.no_flash = not args.flash
    if args.curve == "ec" and args.msg_len != PAYLOAD_LEN:
        p.error("--msg-len is Ed25519 only: ECDSA signs a fixed 32-byte hash")
    if not 1 <= args.msg_len <= MAX_MSG_LEN:
        p.error(f"--msg-len must be 1..{MAX_MSG_LEN}")
    if args.priv_key is not None:
        if args.mode != "message":
            p.error("--priv-key is for message mode; scalar mode already sets the key per signature")
        try:
            args.priv_key = bytes.fromhex(args.priv_key)
        except ValueError:
            p.error("--priv-key must be hex")
        if len(args.priv_key) != PRIVKEY_LEN:
            p.error(f"--priv-key must be {PRIVKEY_LEN} bytes ({2*PRIVKEY_LEN} hex chars)")
    return args


def pick_class(args):
    if args.trace_class == "fixed":
        return CLASS_FIXED
    if args.trace_class == "random":
        return CLASS_RANDOM
    return CLASS_FIXED if np.random.randint(0, 2) == 0 else CLASS_RANDOM


def main():
    args = parse_args()
    target, _pubkey, _privkey, _fw = setup_target(args)
    if args.mode == "scalar" and args.trace_class != "fixed":
        print("[!] Scalar mode rewrites the key slot for every random signature - mind the ECC "
              "slot's write endurance on long runs")

    # setup_target() leaves the fixed scalar in the slot in scalar mode; tracking what is there
    # saves a slot write per fixed signature.
    in_slot = FIXED_SCALAR if args.mode == "scalar" else None
    done = failed = consecutive = 0
    times_ms = []
    print(f"[*] Signing {CURVE_NAMES[args.curve]}, {args.mode} mode, {args.trace_class} "
          f"{'scalar' if args.mode == 'scalar' else 'message'}, "
          f"{args.count if args.count else 'until Ctrl-C'}"
          f"{' signatures' if args.count else ''}, {args.sign_delay:.0f} ms apart")
    try:
        while not args.count or done < args.count:
            trace_class = pick_class(args)
            try:
                if args.mode == "scalar" and trace_class == CLASS_FIXED and in_slot == FIXED_SCALAR:
                    payload = fixed_payload(args.msg_len)
                else:
                    payload, key = prepare_trial(args, target, trace_class)
                    if key is not None:
                        in_slot = key
                start = time.time()
                signature, nonce = target.sign(payload)
                sign_ms = (time.time() - start) * 1e3
            except (TargetError, TimeoutError) as ex:
                failed += 1
                consecutive += 1
                print(f"[!] Signature {done + 1} failed: {ex}")
                if "ret=10" in str(ex):
                    print("    LT_L1_CHIP_ALARM_MODE: only a full power cycle of the ESP32 "
                          "clears it (see README, Troubleshooting)")
                if consecutive >= MAX_CONSECUTIVE_FAILURES:
                    print(f"[!] {consecutive} failures in a row, stopping")
                    break
                time.sleep(1.0)
                continue

            consecutive = 0
            done += 1
            times_ms.append(sign_ms)
            label = "fixed " if trace_class == CLASS_FIXED else "random"
            line = f"[{done:5d}] {label} {sign_ms:6.1f} ms"
            if args.verbose:
                line += f"  sig {signature.hex()}"
                if nonce is not None:
                    line += f"  nonce {nonce.hex()}"
            print(line)
            if args.sign_delay:
                time.sleep(args.sign_delay * 1e-3)
    except KeyboardInterrupt:
        print()
    finally:
        target.close()

    if times_ms:
        t = np.array(times_ms)
        print(f"[*] {done} signatures, {failed} failed; host-measured sign time "
              f"mean {t.mean():.1f} ms, min {t.min():.1f}, max {t.max():.1f}")
    else:
        print(f"[*] No signature completed ({failed} failed)")


if __name__ == "__main__":
    main()
