"""SITL capture -> contract-shaped CSV + provenance sidecar.

Wraps tools/scope2csv.py for the legacy .jsonl captures. A capture that is
already CSV is passed through unchanged -- the current sim path writes CSV
directly, so re-converting it would only risk changing it.

The sidecar is the point. A SITL run's gains currently survive only as console
text in the _debug.log beside it, which nothing reads; pass them with --set so
the run records what it was actually flown with.
"""

import csv
import os
import shutil
import sys

from . import sidecar

_HERE = os.path.dirname(os.path.abspath(__file__))
_TOOLS = os.path.dirname(_HERE)
_REPO = os.path.dirname(_TOOLS)
for _p in (_TOOLS, _REPO):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import scope2csv  # noqa: E402

TIME_COL = "time"


def times_of(path):
    with open(path, newline="", encoding="utf-8") as f:
        r = csv.reader(f)
        header = next(r, None)
        if not header:
            return []
        # The two converters disagree on the time column's name; accept both
        # so a hand-converted file still works here.
        for cand in (TIME_COL, "Time", "t"):
            if cand in header:
                i = header.index(cand)
                break
        else:
            return []
        out = []
        for row in r:
            try:
                out.append(float(row[i]))
            except (ValueError, IndexError):
                continue
        return out


def parse_sets(pairs):
    """--set gain=value, repeated. Recorded verbatim, not interpreted."""
    out = {}
    for p in pairs or []:
        if "=" not in p:
            raise SystemExit("--set expects name=value, got {!r}".format(p))
        k, v = p.split("=", 1)
        out[k.strip()] = v.strip()
    return out or None


def convert(in_path, out_path, maneuver=None, commit=None, t0=None,
            sets=None, airframe=None):
    ext = os.path.splitext(in_path)[1].lower()
    if ext == ".jsonl":
        scope2csv.convert(in_path, out_path)
    elif ext == ".csv":
        if os.path.abspath(in_path) != os.path.abspath(out_path):
            shutil.copyfile(in_path, out_path)
    else:
        raise SystemExit(
            "Unrecognised SITL capture {!r}. Expected .jsonl (legacy scope "
            "capture) or .csv (current sim output).".format(in_path)
        )

    times = times_of(out_path)
    meta = sidecar.build(
        "sitl", in_path, times,
        airframe=airframe,
        maneuver=maneuver,
        firmware_commit=commit,
        t0_offset_s=t0,
        overrides=parse_sets(sets),
        time_column=TIME_COL,
    )
    return out_path, sidecar.write(out_path, meta)
