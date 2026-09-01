## align_trs.py
# Aligns a .trs set by cross-correlating each trace against a reference, then writes a new set.
#
# Trace-to-trace timing drifts because TROPIC01's oscillator and the scope's ADC clock are
# independent, and because the masked scalar multiplication takes a slightly different path every
# execution. Left uncorrected, a leak that is sharp in one trace lands a few hundred samples away
# in the next, and the per-sample t-test averages it into the noise floor.
#
#   python align_trs.py in.trs -o aligned.trs --window 20:25 --max-shift 2000
#   python align_trs.py tiled.trs -o aligned.trs --tiles 39 --max-shift 500
#
# --tiles matters: in a tiled capture every tile is a separate execution with its own drift, so a
# single shift for the whole trace is meaningless. Each tile is aligned independently, and
# correlating across a tile seam would be comparing unrelated executions.
import argparse

import numpy as np
import trsfile


def parse_args():
    p = argparse.ArgumentParser(description="Align a .trs set by cross-correlation")
    p.add_argument("path", help="input .trs")
    p.add_argument("-o", "--output", required=True, help="output .trs")
    p.add_argument("--window", default=None,
                   help="ms range to correlate on, as START:END (default: the whole trace, or the "
                        "whole tile). Pick a region with structure - correlating on flat noise "
                        "produces a meaningless shift")
    p.add_argument("--max-shift", type=int, default=1000,
                   help="largest shift in samples to consider, each way (default: %(default)s). "
                        "Too large invites spurious matches, too small clips real drift")
    p.add_argument("--tiles", type=int, default=1,
                   help="align each of N tiles independently (tiled captures only)")
    p.add_argument("--reference", type=int, default=25,
                   help="average this many traces to build the reference (default: %(default)s)")
    return p.parse_args()


def best_shift(segment, reference, max_shift):
    """Shift that best aligns `segment` onto `reference`, by FFT cross-correlation.

    Both are mean-removed first so the correlation responds to shape rather than DC offset, which
    otherwise dominates and pins every shift to zero.
    """
    a = segment - segment.mean()
    b = reference - reference.mean()
    n = 1 << int(np.ceil(np.log2(a.size + b.size)))
    corr = np.fft.irfft(np.fft.rfft(a, n) * np.conj(np.fft.rfft(b, n)), n)
    # Lags [-max_shift, +max_shift] live at the two ends of the circular correlation.
    lags = np.concatenate([corr[-max_shift:], corr[:max_shift + 1]])
    return int(np.argmax(lags)) - max_shift


def main():
    args = parse_args()

    with trsfile.open(args.path, "r") as ts:
        headers = dict(ts.get_headers())
        n_samples = headers[trsfile.Header.NUMBER_SAMPLES]
        rate = 1.0 / headers[trsfile.Header.SCALE_X]
        count = len(ts)

        if n_samples % args.tiles:
            raise SystemExit(f"{n_samples:,} samples is not divisible by {args.tiles} tiles")
        tile_len = n_samples // args.tiles

        if args.window:
            start_ms, end_ms = (float(x) for x in args.window.split(":"))
            lo, hi = int(start_ms * 1e-3 * rate), int(end_ms * 1e-3 * rate)
            if args.tiles > 1:
                raise SystemExit("--window is relative to the whole trace; with --tiles the "
                                 "correlation region is the tile itself")
        else:
            lo, hi = 0, tile_len if args.tiles > 1 else n_samples
        if not 0 <= lo < hi <= n_samples:
            raise SystemExit(f"--window {args.window} is outside the {n_samples/rate*1e3:.1f} ms trace")

        print(f"[*] {count} traces x {n_samples:,} samples at {rate/1e6:.3f} MS/s")
        if args.tiles > 1:
            print(f"[*] aligning {args.tiles} tiles of {tile_len:,} samples independently")
        else:
            print(f"[*] correlating on {lo/rate*1e3:.2f} - {hi/rate*1e3:.2f} ms")

        # Reference from the first few traces: averaging suppresses noise so the correlation
        # locks onto the operation rather than onto whichever transient a single trace happened
        # to have.
        refs = np.zeros((args.tiles, hi - lo) if args.tiles > 1 else (1, hi - lo))
        used = min(args.reference, count)
        for i in range(used):
            samples = np.asarray(ts[i].samples, dtype=np.float64)
            for t in range(args.tiles):
                base = t * tile_len
                refs[t if args.tiles > 1 else 0] += samples[base + lo: base + hi]
        refs /= used
        print(f"[*] reference built from {used} traces")

        out_headers = {k: v for k, v in headers.items() if k is not trsfile.Header.NUMBER_TRACES}
        out_headers[trsfile.Header.DESCRIPTION] = (
            str(headers.get(trsfile.Header.DESCRIPTION, "")) +
            f"; aligned (max_shift={args.max_shift}, tiles={args.tiles})").lstrip("; ")

        shifts = []
        with trsfile.trs_open(args.output, mode="w", headers=out_headers) as out:
            for i, trace in enumerate(ts):
                samples = np.asarray(trace.samples, dtype=np.float64)
                fixed = samples.copy()
                for t in range(args.tiles):
                    base = t * tile_len
                    seg = samples[base + lo: base + hi]
                    shift = best_shift(seg, refs[t if args.tiles > 1 else 0], args.max_shift)
                    shifts.append(shift)
                    # Roll only within the tile: a tile is a separate execution, so spilling
                    # samples across a seam would mix two of them.
                    fixed[base:base + tile_len] = np.roll(samples[base:base + tile_len], shift)
                out.append(trsfile.Trace(trace.sample_coding,
                                         np.clip(np.rint(fixed), -128, 127).astype(np.int8),
                                         parameters=trace.parameters))
                if (i + 1) % 250 == 0:
                    print(f"    {i+1}/{count}")

    shifts = np.array(shifts)
    print(f"[*] Wrote {args.output}")
    print(f"[*] shifts: mean {shifts.mean():+.1f}, std {shifts.std():.1f}, "
          f"range {shifts.min():+d} .. {shifts.max():+d} samples "
          f"({shifts.std()/rate*1e6:.2f} us of jitter)")
    at_limit = np.mean(np.abs(shifts) >= args.max_shift) * 100
    if at_limit > 1:
        print(f"[!] {at_limit:.1f}% of shifts hit the +-{args.max_shift} limit - real drift is "
              f"being clipped, raise --max-shift")


if __name__ == "__main__":
    main()
