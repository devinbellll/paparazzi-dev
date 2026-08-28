"""mfcdata -- firmware runs into the shared run-record contract.

    python3 -m mfcdata check
    python3 -m mfcdata verify RUN.csv --source flight
    python3 -m mfcdata sd    FLIGHT.data  -o <effort>/data/tracking_att_flight.csv
    python3 -m mfcdata sim   CAPTURE.csv  -o <effort>/data/tracking_att_sitl.csv

Each converter writes ``<stem>.csv`` plus ``<stem>.meta.json``. The stem is
the join key: everything downstream refers to a run by it and never re-reads
raw bytes.

This is the ingest half only. Plots and run notes are the MATLAB side's job --
one plotting stack, so figures from a flight and from a simulation can sit on
the same page without clashing.
"""

import argparse
import os
import sys

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = "mfcdata"

from . import check, sd, sim, verify  # noqa: E402


def default_out(in_path, suffix):
    base = os.path.splitext(os.path.basename(in_path))[0]
    return os.path.join(os.path.dirname(in_path), base + suffix + ".csv")


def main(argv=None):
    p = argparse.ArgumentParser(prog="mfcdata", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    c = sub.add_parser("check", help="contract vs messages.xml and BRANCH_MAP")
    c.add_argument("--root", default=None, help="repo root (default: inferred)")

    v = sub.add_parser("verify", help="are the role bindings actually right?")
    v.add_argument("csv", help="a converted run CSV")
    v.add_argument("--source", default="flight", choices=["flight", "sitl"])

    d = sub.add_parser("sd", help="SD-card flight log -> CSV + sidecar")
    d.add_argument("data", help="the .data file from sd2log")
    d.add_argument("-o", "--out", default=None)
    d.add_argument("--log", default=None, help="matching .log (default: alongside)")
    d.add_argument("--ac", default=None, help="filter to one ac_id")
    d.add_argument("--maneuver", default=None)
    d.add_argument("--commit", default=None,
                   help="firmware commit that FLEW this. Not guessed from the "
                        "working tree -- omitted entirely if you do not pass it.")
    d.add_argument("--t0", type=float, default=None,
                   help="manual time anchor, recorded in the sidecar")

    s = sub.add_parser("sim", help="SITL capture -> CSV + sidecar")
    s.add_argument("capture", help=".jsonl (legacy) or .csv (current)")
    s.add_argument("-o", "--out", default=None)
    s.add_argument("--airframe", default=None)
    s.add_argument("--maneuver", default=None)
    s.add_argument("--commit", default=None)
    s.add_argument("--t0", type=float, default=None)
    s.add_argument("--set", dest="sets", action="append", default=None,
                   metavar="NAME=VALUE",
                   help="a gain override this run was flown with; repeatable")

    a = p.parse_args(argv)

    if a.cmd == "check":
        n, _ = check.run(repo_root=a.root)
        return 1 if n else 0

    if a.cmd == "verify":
        n, _ = verify.run(a.csv, source_name=a.source)
        return 1 if n else 0

    if a.cmd == "sd":
        out = a.out or default_out(a.data, "_pj")
        os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
        csv_path, meta_path = sd.convert(
            a.data, out, log_path=a.log, ac=a.ac,
            maneuver=a.maneuver, commit=a.commit, t0=a.t0)
    else:
        out = a.out or default_out(a.capture, "")
        os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
        csv_path, meta_path = sim.convert(
            a.capture, out, maneuver=a.maneuver, commit=a.commit,
            t0=a.t0, sets=a.sets, airframe=a.airframe)

    print(csv_path)
    print(meta_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
