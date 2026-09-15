## trs_info.py
# Prints the metadata of a .trs set: the campaign-level parameters (key, curve, scope setup,
# firmware, git, timestamp) and a sample of the per-trace fields (class, message, key).
#
#   python trs_info.py traces/set.trs
#
# Useful because a viewer that re-saves a set often drops the DESCRIPTION string; the structured
# TRACE_SET_PARAMETERS survive, and this reads them back as real values.
import argparse
import trsfile


def parse_args():
    p = argparse.ArgumentParser(description="Show the metadata of a .trs trace set")
    p.add_argument("paths", nargs="+", help="the .trs files to inspect")
    p.add_argument("--traces", type=int, default=3, help="per-trace rows to print (default: 3)")
    return p.parse_args()


def fmt(param):
    """Render a trace-set parameter: bytes as hex, short numeric arrays inline, strings as-is."""
    v = param.value
    if isinstance(v, (bytes, bytearray)):
        return v.hex()
    if isinstance(v, list):
        if v and isinstance(v[0], int) and len(v) > 4:      # a byte array
            return bytes(v).hex()
        if len(v) == 1:
            return str(v[0])
        return str(v)
    return str(v)


def main():
    args = parse_args()
    for path in args.paths:
        with trsfile.open(path, "r") as ts:
            h = ts.get_headers()
            print(f"\n=== {path} ===")
            print(f"traces: {len(ts)}   samples: {h.get(trsfile.Header.NUMBER_SAMPLES)}   "
                  f"coding: {h.get(trsfile.Header.SAMPLE_CODING)}")
            desc = h.get(trsfile.Header.DESCRIPTION)
            if desc:
                print(f"description: {desc}")

            tsp = h.get(trsfile.Header.TRACE_SET_PARAMETERS)
            if tsp:
                print("\ncampaign metadata:")
                width = max(len(k) for k in tsp.keys())
                for k in tsp.keys():
                    print(f"  {k:>{width}} : {fmt(tsp[k])}")
            else:
                print("\n(no TRACE_SET_PARAMETERS - captured before metadata was added, "
                      "or written by another tool)")

            if len(ts):
                keys = list(ts[0].parameters.keys())
                print(f"\nper-trace fields: {', '.join(keys)}")
                for i in range(min(args.traces, len(ts))):
                    parts = []
                    for k in keys:
                        v = ts[i].parameters[k].value
                        v = bytes(v).hex() if len(v) > 2 else v[0]
                        parts.append(f"{k}={v}")
                    print(f"  [{i}] " + "  ".join(str(p) for p in parts))


if __name__ == "__main__":
    main()
