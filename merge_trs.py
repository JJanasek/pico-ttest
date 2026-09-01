## merge_trs.py
# Joins the parts written by `tvla_capture.py --traces-per-file N` back into a single .trs.
#
# Rolling to a new file every N traces is what keeps a long campaign safe - a .trs only becomes
# readable when it is closed, so an interrupted run loses at most the part in progress. Analysis
# usually wants one file, so this puts them back together.
#
#   python merge_trs.py traces/TROPIC01_ed_message_1000_*_part*.trs -o traces/merged.trs
#
# Parts are merged in the order given; a shell glob sorts them by part number already. The output
# keeps the first part's headers, with the trace count and description updated.
import argparse
import os

import trsfile

# Headers that must agree across parts for the merge to mean anything: traces of different
# lengths or codings cannot live in one set.
COMPATIBILITY_HEADERS = (
    trsfile.Header.NUMBER_SAMPLES,
    trsfile.Header.SAMPLE_CODING,
    trsfile.Header.LENGTH_DATA,
)


def parse_args():
    p = argparse.ArgumentParser(description="Merge .trs parts into a single trace set")
    p.add_argument("parts", nargs="+", help="the .trs files to join, in order")
    p.add_argument("-o", "--output", required=True, help="path of the merged .trs")
    p.add_argument("-f", "--force", action="store_true", help="overwrite the output if it exists")
    return p.parse_args()


def main():
    args = parse_args()

    if os.path.exists(args.output) and not args.force:
        raise SystemExit(f"{args.output} already exists (pass --force to overwrite)")
    if args.output in args.parts:
        raise SystemExit("the output would overwrite one of the inputs")

    # Read every part's headers first, so an incompatible set fails before anything is written.
    headers = None
    total = 0
    parts = list(args.parts)
    for path in list(parts):
        try:
            with trsfile.open(path, "r") as ts:
                part_headers = dict(ts.get_headers())
                count = len(ts)
        except Exception as ex:  # noqa: BLE001 - trsfile reports a truncated file several ways
            # A .trs only becomes readable when it is closed, so the last part of a campaign that
            # is still running - or was interrupted - fails to open. That is the normal case when
            # merging partway through, and dropping it costs nothing. An unreadable part anywhere
            # else means real data loss, and silently skipping it would leave a set with a hole.
            if path == parts[-1]:
                print(f"[!] {path} is not readable ({ex}) - skipping it. That is expected while "
                      f"the campaign is still writing this part.")
                parts.remove(path)
                continue
            raise SystemExit(
                f"{path} is not readable ({ex}), and it is not the final part - merging would "
                f"silently drop traces from the middle of the set. Fix or exclude it explicitly.")
        if headers is None:
            headers = part_headers
        else:
            for key in COMPATIBILITY_HEADERS:
                if part_headers.get(key) != headers.get(key):
                    raise SystemExit(
                        f"{path} does not match {args.parts[0]}: {key.name} is "
                        f"{part_headers.get(key)} vs {headers.get(key)}")
        print(f"[*] {path}: {count} traces")
        total += count

    # NUMBER_TRACES is maintained by trsfile as traces are appended, so it is left out here; the
    # description is rewritten because the parts' copies all claim the campaign's full length.
    headers = {k: v for k, v in headers.items() if k is not trsfile.Header.NUMBER_TRACES}
    description = headers.get(trsfile.Header.DESCRIPTION, "")
    headers[trsfile.Header.DESCRIPTION] = (
        f"{description}; merged from {len(parts)} parts").lstrip("; ")

    written = 0
    with trsfile.trs_open(args.output, mode="w", headers=headers) as out:
        for path in parts:
            with trsfile.open(path, "r") as ts:
                for trace in ts:
                    # Trace carries its own parameters (the 'ttest' class byte), so re-wrapping
                    # it preserves the fixed/random split the whole campaign exists to record.
                    out.append(trsfile.Trace(trace.sample_coding, trace.samples,
                                             parameters=trace.parameters))
                    written += 1

    print(f"[*] Wrote {written} traces to {args.output} "
          f"({os.path.getsize(args.output) / 1024**2:.1f} MiB)")
    if written != total:
        raise SystemExit(f"expected {total} traces, wrote {written}")


if __name__ == "__main__":
    main()
