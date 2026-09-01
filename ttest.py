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
import math

import numpy as np
import trsfile

# The threshold TVLA is usually quoted with. It is derived for a SINGLE comparison, and using it
# on a whole trace is the most common way to read leakage into noise: with a million samples,
# values above it occur by chance in every run.
CLASSIC_THRESHOLD = 4.5

# Family-wise error rate the corrected threshold targets.
ALPHA = 1e-5


def corrected_threshold(n_samples, alpha=ALPHA):
    """|t| a trace of n_samples must exceed before one point is evidence of leakage.

    Every sample is its own hypothesis test, so the chance of at least one large |t| under the
    null grows with the trace length - the classic 4.5 gets crossed by noise alone once traces
    run to millions of points. This is the Bonferroni correction: solve the two-sided normal
    tail for alpha/n_samples. It is conservative, because neighbouring samples are correlated
    and the effective number of independent tests is smaller than the sample count.
    """
    target = alpha / n_samples
    lo, hi = 0.0, 40.0
    for _ in range(200):
        mid = (lo + hi) / 2
        if math.erfc(mid / math.sqrt(2)) > target:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def null_expected_max(n_samples):
    """Roughly the largest |t| a leak-free trace of this length produces anyway."""
    return math.sqrt(2 * math.log(max(n_samples, 2)))


def parse_args():
    p = argparse.ArgumentParser(description="Fixed-vs-random TVLA t-test on .trs sets")
    p.add_argument("paths", nargs="+", help="the .trs files to analyse")
    p.add_argument("--top", type=int, default=5, help="how many peaks to list (default: 5)")
    p.add_argument("--blocks", type=int, default=10,
                   help="how many equal time blocks to summarise (default: 10)")
    p.add_argument("--spectrum", action="store_true",
                   help="report how signal power is distributed in frequency instead of running "
                        "the t-test. Answers in seconds, from a handful of traces, whether the "
                        "measurement chain passes the band the leakage lives in - a t-test needs "
                        "thousands of traces to tell you the same thing")
    p.add_argument("--rate", type=float, default=None,
                   help="sample rate in S/s, for sets whose header has no SCALE_X")
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

    threshold = corrected_threshold(t.size)
    expected = null_expected_max(t.size)
    peak = np.abs(t).max()
    over = np.abs(t) > threshold
    print(f"max |t| = {peak:.2f} at {ms[int(np.argmax(np.abs(t)))]:.3f} ms")
    print(f"threshold for {t.size:,} samples: {threshold:.2f}  "
          f"(the classic {CLASSIC_THRESHOLD} is a single-test figure and is meaningless here)")
    print(f"a leak-free trace this long peaks around |t| = {expected:.2f} by chance alone")
    if peak > threshold:
        print(f"-> {int(over.sum()):,} samples exceed the corrected threshold: real leakage")
    elif peak > expected:
        print(f"-> peak is above the chance level but below the threshold: suggestive, not "
              f"conclusive - collect more traces")
    else:
        print(f"-> peak is at or below what noise alone produces: NO evidence of leakage")
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
        flag = "  <-- leak" if seg.max() > threshold else ""
        print(f"   {ms[k]:8.2f} - {ms[min(k+step, t.size-1)]:8.2f} ms : {seg.max():7.2f}{flag}")


def spectrum(path, rate, traces=20):
    """Where the signal's power sits in frequency, averaged over a few traces.

    The band that matters is set by the target, not the scope: a 100 ms asymmetric operation
    leaks in its envelope, far below the MHz range a fast-target rig is built for. If the chain
    does not pass that band, no amount of gain, bandwidth or averaging recovers it.
    """
    with trsfile.open(path, "r") as ts:
        if rate is None:
            scale_x = ts.get_headers().get(trsfile.Header.SCALE_X)
            if not scale_x:
                raise SystemExit(f"{path} has no SCALE_X; pass --rate")
            rate = 1.0 / scale_x
        n = min(traces, len(ts))
        power = None
        for i in range(n):
            a = np.asarray(ts[i].samples, dtype=np.float64)
            a = a - a.mean()
            p = np.abs(np.fft.rfft(a)) ** 2
            power = p if power is None else power + p
    freq = np.fft.rfftfreq(a.size, 1.0 / rate)
    total = power.sum()
    print(f"\n=== {path.split('/')[-1]} — spectrum ===")
    print(f"{n} traces, {a.size:,} samples at {rate/1e6:.3f} MS/s")
    edges = [0, 1e3, 10e3, 100e3, 500e3, 2e6, 10e6, rate / 2]
    print(f"{'band':>22} {'share of power':>16}")
    for lo, hi in zip(edges, edges[1:]):
        if lo >= rate / 2:
            break
        m = (freq >= lo) & (freq < min(hi, rate / 2))
        share = 100 * power[m].sum() / total
        bar = "#" * int(share / 2)
        print(f"{lo/1e3:>9.0f}-{min(hi, rate/2)/1e3:<8.0f} kHz {share:>10.2f}%  {bar}")
    below = 100 * power[freq < 10e3].sum() / total
    print(f"\nbelow 10 kHz: {below:.2f}%")
    print("  A working PicoScope measurement of this target had 91%; a passive probe into the")
    print("  Husky's ~1k input had 11%, and its t-test found nothing. Under ~50% here means the")
    print("  chain is high-passing away the band the leakage is in.")


def main():
    args = parse_args()
    if args.spectrum:
        for path in args.paths:
            spectrum(path, args.rate)
        return
    results = []
    for path in args.paths:
        t, rate, counts = welch(path, args.decimate)
        if args.rate:
            rate = args.rate / args.decimate
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
