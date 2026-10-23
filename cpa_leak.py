## cpa_leak.py
# Known-message CPA around the leak, to tell a plaintext message load/copy apart from the keyed
# TMAC compression that follows it in EdDSA nonce generation.
#
# The only predictor is HW(M), the Hamming weight of the recorded (public) message - nothing else,
# because the TMAC value r1 = TMAC(prefix, sch||scn||M) depends on the secret nonce-prefix and on
# the secure-channel value sch that libtropic discards, so it cannot be modelled. That is the point:
#
#   * HW(M) correlates at the leak  -> the leak is raw message transport (a load/copy). Harmless:
#     M is public. A per-byte "diagonal" (byte i peaking later than byte i-1) nails a serial load.
#   * TVLA fires there but HW(M) does NOT correlate -> the leaking value is diffused, i.e. the TMAC
#     (or SHA) compression of M. That is a keyed primitive and deserves its own test.
#
# Only random-class traces carry information (fixed M has no variance). Streams one trace at a time
# and keeps only running sums, so it does not blow up memory the way holding every window would.
#
#   python cpa_leak.py TROPIC01_..._part*.trs --per-byte -o cpa.npz --plot cpa.png
#
# If the HW(M) correlation peak looks smeared or sits off the known leak time, the windows need
# aligning first (trace-to-trace jitter); say so and we add a cross-correlation alignment pass.
import argparse
import numpy as np
import trsfile

POP = np.array([bin(i).count("1") for i in range(256)], dtype=np.float64)   # byte -> Hamming weight


def parse_args():
    p = argparse.ArgumentParser(description="Known-message HW(M) CPA around the leak")
    p.add_argument("parts", nargs="+", help="the .trs file(s)")
    p.add_argument("--center", type=float, default=18_379_647, help="leak sample (default: the |t| peak)")
    p.add_argument("--half", type=float, default=400_000, help="half-window in samples (default 4e5 = +-0.32 ms)")
    p.add_argument("--sample-rate", type=float, default=1.25e9, help="for the time axis")
    p.add_argument("--pre-trigger", type=float, default=1_250_000, help="pre-trigger samples, for the time axis")
    p.add_argument("--per-byte", action="store_true", help="also correlate each message byte separately")
    p.add_argument("-o", "--output", help="write the correlation trace(s) to this .npz")
    p.add_argument("--plot", help="write a PNG of the result")
    return p.parse_args()


def main():
    args = parse_args()
    center, half = int(args.center), int(args.half)
    lo, hi = max(0, center - half), center + half
    W = hi - lo

    # Running sums over random-class traces: everything CPA needs, per sample, in a few vectors.
    n = 0
    Sx = np.zeros(W); Sxx = np.zeros(W)                 # sum x, sum x^2        (per sample)
    Sh = 0.0; Shh = 0.0; Shx = np.zeros(W)              # sum h, sum h^2, sum h*x   (h = total HW)
    nbytes = None
    if args.per_byte:
        Shb = None; Shhb = None; Shxb = None            # per-byte versions, sized on first trace

    for path in args.parts:
        with trsfile.open(path, "r") as ts:
            for tr in ts:
                if int(tr.parameters["ttest"].value[0]) != 1:     # random only
                    continue
                x = np.asarray(tr.samples[lo:hi], dtype=np.float64)
                mb = np.frombuffer(bytes(tr.parameters["msg"].value), np.uint8)
                hb = POP[mb]                                        # per-byte HW
                h = hb.sum()                                        # total HW(M)

                n += 1
                Sx += x; Sxx += x * x
                Sh += h; Shh += h * h; Shx += h * x
                if args.per_byte:
                    if Shb is None:
                        nbytes = mb.size
                        Shb = np.zeros(nbytes); Shhb = np.zeros(nbytes)
                        Shxb = np.zeros((nbytes, W))
                    Shb += hb; Shhb += hb * hb
                    Shxb += hb[:, None] * x[None, :]
        print(f"[*] {path}: {n} random traces so far")

    if n < 3:
        raise SystemExit("need random-class traces (ttest=1); found too few")

    # Pearson r per sample between h and x.
    def corr(Sh, Shh, Shx):
        num = n * Shx - Sh * Sx
        den = np.sqrt((n * Shh - Sh * Sh) * (n * Sxx - Sx * Sx) + 1e-30)
        return num / den

    r = corr(Sh, Shh, Shx)
    t_axis = (np.arange(lo, hi) - args.pre_trigger) / args.sample_rate * 1e3   # ms after trigger
    pk = int(np.argmax(np.abs(r)))
    thr = 3.5 / np.sqrt(n)                                  # ~3.5 sigma significance for Pearson r
    print(f"\n[*] total HW(M) model, {n} random traces, |r| significant above ~{thr:.3f}")
    print(f"[*] peak |r| = {abs(r[pk]):.3f} (r={r[pk]:+.3f}) at {t_axis[pk]:.3f} ms "
          f"(sample {lo+pk}); leak is at {(center-args.pre_trigger)/args.sample_rate*1e3:.3f} ms")
    verdict = ("HW(M) EXPLAINS the leak -> plaintext message transport (load/copy), not the TMAC"
               if abs(r[pk]) > thr and abs(t_axis[pk] - (center-args.pre_trigger)/args.sample_rate*1e3) < 0.02
               else "HW(M) does NOT explain the leak at its location -> consistent with the diffused "
                    "TMAC/hash compression (or the window needs aligning)")
    print(f"[*] {verdict}")

    out = {"r_total": r.astype(np.float32), "t_ms": t_axis.astype(np.float32),
           "n_random": n, "thr": thr}
    rb = None
    if args.per_byte:
        rb = np.stack([corr(Shb[i], Shhb[i], Shxb[i]) for i in range(nbytes)]).astype(np.float32)
        out["r_byte"] = rb
        # a serial load shows each byte's |r| peaking at a monotonically increasing time
        pk_t = np.array([t_axis[np.argmax(np.abs(rb[i]))] for i in range(nbytes)])
        mono = np.corrcoef(np.arange(nbytes), pk_t)[0, 1]
        print(f"[*] per-byte: peak-time vs byte-index correlation = {mono:+.2f} "
              f"(near +1 => serial load diagonal; near 0 => no byte ordering)")

    if args.output:
        np.savez_compressed(args.output, **out)
        print(f"[*] wrote {args.output}")

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2 if args.per_byte else 1, 1,
                               figsize=(11, 6 if args.per_byte else 3.2), squeeze=False)
        a = ax[0, 0]
        a.plot(t_axis, r, lw=0.5, color="#4e79a7")
        a.axhline(thr, color="#555", ls=":", lw=0.8); a.axhline(-thr, color="#555", ls=":", lw=0.8)
        a.axvline((center-args.pre_trigger)/args.sample_rate*1e3, color="#e15759", ls="--", lw=1)
        a.set_ylabel("r(HW(M), sample)"); a.set_xlabel("time after trigger (ms)")
        a.set_title(f"Known-message CPA, total HW(M), {n} random traces "
                    f"(dashed red = leak; dotted = ~significance)")
        if args.per_byte:
            b = ax[1, 0]
            im = b.imshow(np.abs(rb), aspect="auto", origin="lower",
                          extent=[t_axis[0], t_axis[-1], 0, nbytes], cmap="magma")
            b.axvline((center-args.pre_trigger)/args.sample_rate*1e3, color="#39a2ff", ls="--", lw=1)
            b.set_ylabel("message byte index"); b.set_xlabel("time after trigger (ms)")
            b.set_title("per-byte |r| (a bright diagonal = serial load of M)")
            fig.colorbar(im, ax=b, label="|r|")
        fig.tight_layout()
        fig.savefig(args.plot, dpi=130)
        print(f"[*] wrote {args.plot}")


if __name__ == "__main__":
    main()
