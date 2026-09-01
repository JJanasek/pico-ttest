## ttest.py
# Non-specific fixed-vs-random TVLA: Welch's t per sample, plus a spectrum of the class
# difference so two captures at different sample rates can be compared directly.
#
#   python ttest.py traces/set.trs
#   python ttest.py traces/a.trs traces/b.trs        # compare two acquisitions
#
# Traces are accumulated as running sums rather than loaded, so a set larger than RAM is fine.
# Comparing acquisitions of different sizes uses t/sqrt(N): |t| grows as sqrt(N), so dividing it
# out leaves the per-trace leakage strength, which is what says whether a change to the capture
# (more bandwidth, a different probe) actually bought anything.
import argparse
import numpy as np
import trsfile

# The usual TVLA threshold: |t| above this is taken as evidence of class-dependent leakage.
THRESHOLD = 4.5


def parse_args():
    p = argparse.ArgumentParser(description="Fixed-vs-random TVLA t-test on .trs sets")
    p.add_argument("paths", nargs="+", help="the .trs files to analyse")
    p.add_argument("--top", type=int, default=5, help="how many peaks to list (default: 5)")
    p.add_argument("--blocks", type=int, default=10,
                   help="how many equal time blocks to summarise (default: 10)")
    p.add_argument("--decimate", type=int, default=1,
                   help="low-pass by averaging N samples and keep every Nth, before the t-test. "
                        "Running a set at --decimate 1 and again at --decimate 10 answers whether "
                        "its bandwidth above the decimated Nyquist carries leakage - and because "
                        "it is the SAME traces both times, trace alignment is identical and "
                        "cannot bias the comparison the way comparing two acquisitions does")
    return p.parse_args()


def decimate(samples, factor):
    """Boxcar-averages `factor` samples and keeps one, as a crude anti-aliased downsample.

    A boxcar is a poor filter, but it is the honest comparison here: it is what the leakage would
    look like if the ADC had simply sampled slower, and it needs no SciPy.
    """
    if factor <= 1:
        return samples
    usable = (samples.size // factor) * factor
    return samples[:usable].reshape(-1, factor).mean(axis=1)


def welch(path, factor=1):
    """Streams one trace set and returns (t per sample, sample rate, class counts)."""
    with trsfile.open(path, "r") as ts:
        headers = ts.get_headers()
        n_samples = headers[trsfile.Header.NUMBER_SAMPLES] // factor
        scale_x = headers[trsfile.Header.SCALE_X] * factor
        counts = [0, 0]
        total = [np.zeros(n_samples), np.zeros(n_samples)]
        squares = [np.zeros(n_samples), np.zeros(n_samples)]
        for trace in ts:
            cls = int(trace.parameters["ttest"].value[0])
            samples = decimate(np.asarray(trace.samples, dtype=np.float64), factor)[:n_samples]
            counts[cls] += 1
            total[cls] += samples
            squares[cls] += samples * samples

    if min(counts) < 2:
        raise SystemExit(f"{path}: need at least 2 traces per class, got {counts}")

    mean = [total[c] / counts[c] for c in (0, 1)]
    # Sample variance with Bessel's correction, from the running sums.
    var = [(squares[c] / counts[c] - mean[c] ** 2) * counts[c] / (counts[c] - 1) for c in (0, 1)]
    # The epsilon keeps a sample that never varies in either class from dividing by zero; such a
    # sample carries no information, and its t is legitimately 0.
    t = (mean[0] - mean[1]) / np.sqrt(var[0] / counts[0] + var[1] / counts[1] + 1e-30)
    rate = 1.0 / scale_x if scale_x else float("nan")
    return t, rate, counts


def report(path, t, rate, counts, args):
    n = sum(counts)
    ms = np.arange(t.size) / rate * 1e3
    print(f"\n=== {path.split('/')[-1]} ===")
    print(f"{counts[0]} fixed / {counts[1]} random, {t.size:,} samples at {rate/1e6:.3f} MS/s "
          f"({t.size/rate*1e3:.1f} ms)")

    over = np.abs(t) > THRESHOLD
    print(f"max |t| = {np.abs(t).max():.2f} at {ms[int(np.argmax(np.abs(t)))]:.3f} ms   "
          f"| over {THRESHOLD}: {int(over.sum()):,} samples ({100*over.mean():.3f}%)")
    # Normalised so acquisitions of different sizes are comparable.
    print(f"t/sqrt(N) = {np.abs(t).max()/np.sqrt(n):.4f}  <- compare this between acquisitions")

    # List peaks that are actually separated in time, rather than the same peak N times.
    order = np.argsort(-np.abs(t))
    peaks, guard = [], max(1, int(1e-3 * rate))
    for i in order:
        if all(abs(i - j) > guard for j in peaks):
            peaks.append(int(i))
        if len(peaks) >= args.top:
            break
    print(f"top {len(peaks)} separated peaks:")
    for i in peaks:
        print(f"   {ms[i]:9.3f} ms  t = {t[i]:+8.2f}")

    print(f"max |t| per block:")
    step = max(1, t.size // args.blocks)
    for k in range(0, t.size, step):
        seg = np.abs(t[k:k+step])
        flag = "  <-- leak" if seg.max() > THRESHOLD else ""
        print(f"   {ms[k]:8.2f} - {ms[min(k+step, t.size-1)]:8.2f} ms : {seg.max():7.2f}{flag}")


def main():
    args = parse_args()
    results = []
    for path in args.paths:
        t, rate, counts = welch(path, args.decimate)
        report(path, t, rate, counts, args)
        results.append((path, t, rate, sum(counts)))

    if len(results) > 1:
        print("\n=== comparison (t/sqrt(N): per-trace leakage strength) ===")
        for path, t, rate, n in results:
            print(f"   {np.abs(t).max()/np.sqrt(n):.4f}  {rate/1e6:>8.1f} MS/s  n={n:<6}  "
                  f"{path.split('/')[-1]}")
        print("\nA higher value means the acquisition captures more leakage per trace. If the "
              "faster capture does not beat the slower one here, its extra bandwidth is not "
              "buying signal and the cheaper acquisition is the better campaign.")


if __name__ == "__main__":
    main()
