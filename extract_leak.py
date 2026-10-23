## extract_leak.py
# Pulls the little that instruction-mapping needs out of a big .trs set, so a 125 GB campaign
# never has to be copied anywhere. The leak we are localising is a low-frequency feature (the
# fixed-vs-random peak is well under 10 kHz), so a per-class mean and t-curve decimated to a few
# MS/s keep every structure that matters while shrinking the whole 50 ms window to a few MB.
#
#   python extract_leak.py traces/TROPIC01_ed_message_2000_*_part*.trs -o leak.npz
#   python extract_leak.py set.trs -o leak.npz --decim 100 --per-class 20 --center 18.4e6 --half 1e6
#
# Output (one .npz):
#   mean_fixed, mean_random  per-class mean trace, decimated by --decim
#   t                        Welch t at the decimated rate (sign is arbitrary; |t| locates the leak)
#   win_fixed, win_random    --per-class full-rate traces each, sliced to [center-half, center+half]
#   meta                     decim, sample_rate, win_lo, counts - everything needed to put it back
#                            on a real time axis
#
# Decimation is a block mean (reshape + mean), which is a brick-wall-ish low-pass: fine for a leak
# far below the new Nyquist, and it is exactly what makes the <10 kHz peak survive a 100x shrink.
import argparse
import numpy as np
import trsfile


def parse_args():
    p = argparse.ArgumentParser(
        description="Extract a small per-class mean + t-curve (and a few full-rate windows) "
                    "from a large .trs set, for instruction mapping without copying the traces")
    p.add_argument("parts", nargs="+", help="the .trs file(s), in order (a glob of parts is fine)")
    p.add_argument("-o", "--output", required=True, help="output .npz")
    p.add_argument("--decim", type=int, default=100,
                   help="decimation factor for the mean/t curves (default 100: 1.25 GS/s -> "
                        "12.5 MS/s, 625k points over 50 ms, ~5 MB total)")
    p.add_argument("--per-class", type=int, default=20,
                   help="full-rate traces per class to keep, windowed to the peak (default 20; "
                        "0 = none, curves only)")
    p.add_argument("--center", type=float, default=18.4e6,
                   help="sample index the window is centred on (default 18.4e6, the peak)")
    p.add_argument("--half", type=float, default=1.0e6,
                   help="half-width of the window in samples (default 1e6: +-0.8 ms at 1.25 GS/s)")
    p.add_argument("--sample-rate", type=float, default=1.25e9,
                   help="achieved sample rate, for the time axis in the output (default 1.25e9)")
    return p.parse_args()


def main():
    args = parse_args()
    center, half = int(args.center), int(args.half)
    lo, hi = max(0, center - half), center + half

    # Decimated running stats per class, for mean and Welch t. Allocated lazily once the first
    # trace tells us the decimated length. float64 keeps sum-of-squares honest over 2000 traces.
    acc = {0: None, 1: None}          # class -> dict(n, s, ss) ; 0 = fixed, 1 = random
    wins = {0: [], 1: []}             # class -> list of full-rate windowed int8 slices
    D = args.decim
    n_total = 0

    for path in args.parts:
        with trsfile.open(path, "r") as ts:
            for trace in ts:
                cls = int(trace.parameters["ttest"].value[0])
                if cls not in acc:
                    continue
                x = np.asarray(trace.samples)

                # Decimate by block mean; drop the ragged tail so the reshape is exact.
                m = (x.size // D) * D
                dec = x[:m].reshape(-1, D).mean(axis=1).astype(np.float64)
                a = acc[cls]
                if a is None:
                    a = acc[cls] = {"n": 0,
                                    "s": np.zeros(dec.size),
                                    "ss": np.zeros(dec.size)}
                a["n"] += 1
                a["s"] += dec
                a["ss"] += dec * dec

                if args.per_class and len(wins[cls]) < args.per_class:
                    wins[cls].append(x[lo:hi].astype(np.int8))
                n_total += 1
        print(f"[*] {path}: {n_total} traces so far "
              f"(fixed {acc[0]['n'] if acc[0] else 0}, random {acc[1]['n'] if acc[1] else 0})")

    if acc[0] is None or acc[1] is None:
        raise SystemExit("need both classes (ttest=0 and ttest=1); found only one")

    def mean(a):
        return a["s"] / a["n"]

    def var(a):
        # population -> sample variance; n is in the thousands so the correction is cosmetic,
        # but it keeps the t honest for small --per-class cross-checks.
        v = a["ss"] / a["n"] - mean(a) ** 2
        return v * a["n"] / max(a["n"] - 1, 1)

    mf, mr = mean(acc[0]), mean(acc[1])
    t = (mf - mr) / np.sqrt(var(acc[0]) / acc[0]["n"] + var(acc[1]) / acc[1]["n"] + 1e-30)

    out = {
        "mean_fixed": mf.astype(np.float32),
        "mean_random": mr.astype(np.float32),
        "t": t.astype(np.float32),
        "meta": np.array([D, args.sample_rate, lo, hi, acc[0]["n"], acc[1]["n"]], dtype=np.float64),
    }
    if args.per_class:
        out["win_fixed"] = np.stack(wins[0]) if wins[0] else np.empty((0, hi - lo), np.int8)
        out["win_random"] = np.stack(wins[1]) if wins[1] else np.empty((0, hi - lo), np.int8)

    np.savez_compressed(args.output, **out)

    import os
    peak = lo + int(np.argmax(np.abs(t[lo // D:hi // D]))) * D if t.size else center
    print(f"[*] fixed {acc[0]['n']}, random {acc[1]['n']} traces")
    print(f"[*] |t| peak in window near sample {peak} "
          f"(t = {t[peak // D]:+.2f} at {D}x decimation)")
    print(f"[*] wrote {args.output} ({os.path.getsize(args.output) / 1024**2:.1f} MiB)")


if __name__ == "__main__":
    main()
