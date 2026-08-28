"""SD-card flight log -> contract-shaped CSV + provenance sidecar.

Wraps tools/sdlog2scope.py rather than reimplementing it, then does the two
things that converter does not:

1. APPLIES THE FIELD ALIASES. sdlog2scope imports BRANCH_MAP from
   pj_json_relay but never calls apply_field_aliases, so an offline flight
   file carries raw ROTORCRAFT_FP/* while a SITL capture carries EST/*. That
   is not a naming inconsistency -- the alias also carries fixed-point scale
   factors and an ENU->NED negation on the vertical axis. Skipping it leaves
   position in raw int32 counts and altitude with the wrong sign, which plots
   perfectly plausibly and is upside down.

2. WRITES THE SIDECAR, so the run records which aircraft and what rate.
"""

import csv
import os
import sys

from . import sidecar

_HERE = os.path.dirname(os.path.abspath(__file__))
_TOOLS = os.path.dirname(_HERE)
_REPO = os.path.dirname(_TOOLS)
for _p in (_TOOLS, _REPO):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import sdlog2scope  # noqa: E402
from pj_json_relay import FIELD_ALIAS  # noqa: E402

TIME_COL = "Time"


def alias_columns():
    """(source column, target column, scale) for every aliased field.

    Read off FIELD_ALIAS so the scales and the vertical negation have exactly
    one definition. Duplicating them here to save an import is how a sign
    error gets introduced.
    """
    out = []
    for src_branch, fields in FIELD_ALIAS.items():
        for src_field, (dst_branch, dst_field, scale) in fields.items():
            out.append((
                "/uav/{}/{}".format(src_branch, src_field),
                "/uav/{}/{}".format(dst_branch, dst_field),
                scale,
            ))
    return out


def apply_aliases(path):
    """Add aliased columns to a converted CSV, in place.

    Aliases only ever ADD columns; the raw source stays so the untransformed
    values remain available. An alias never overwrites an existing column --
    if a genuine EST/* is present it wins, matching the live relay's rule.
    """
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.reader(f))
    if not rows:
        return 0

    header, body = rows[0], rows[1:]
    index = {name: i for i, name in enumerate(header)}

    todo = [(index[s], d, k) for s, d, k in alias_columns()
            if s in index and d not in index]
    if not todo:
        return 0

    header = header + [d for _, d, _ in todo]
    out = []
    for row in body:
        extra = []
        for i, _, scale in todo:
            raw = row[i].strip() if i < len(row) else ""
            extra.append("" if raw == "" else repr(float(raw) * scale))
        out.append(row + extra)

    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(out)
    return len(todo)


def identity_from_log(log_path):
    """(aircraft, started_at) from the .log header, or (None, None).

    The .log embeds the full airframe conf -- the only place the aircraft's
    identity survives the trip off the SD card. Note the attribute is
    uppercase NAME on <airframe>, not the lowercase name you would expect,
    and there is no <aircraft> element at all.

    time_of_day is a real epoch and is the run's wall-clock start, which is
    otherwise unrecoverable: the .data timestamps are seconds since boot.
    """
    if not log_path or not os.path.isfile(log_path):
        return None, None
    try:
        import datetime
        import xml.etree.ElementTree as ET
        root = ET.parse(log_path).getroot()

        name = None
        for af in root.iter("airframe"):
            name = af.get("NAME") or af.get("name")
            if name:
                break

        started = None
        tod = root.get("time_of_day")
        if tod:
            started = datetime.datetime.fromtimestamp(
                float(tod)).astimezone().isoformat(timespec="seconds")
        return name, started
    except Exception:
        # A malformed header is not worth failing a conversion over; the
        # sidecar simply records less.
        return None, None


def ac_id_from_data(data_path, limit=50):
    """The ac_id flying, read off the records.

    Not in the .log header -- each .data line is "<t> <ac_id> <MSG> ...", so
    the records are the only source. Reads a handful of lines rather than the
    whole file, and returns None if they disagree, because a mixed log has no
    single answer to record.
    """
    seen = set()
    try:
        with open(data_path, encoding="utf-8", errors="replace") as f:
            for i, line in enumerate(f):
                if i >= limit:
                    break
                parts = line.split()
                if len(parts) >= 2:
                    seen.add(parts[1])
    except OSError:
        return None
    return seen.pop() if len(seen) == 1 else None


def times_of(path):
    with open(path, newline="", encoding="utf-8") as f:
        r = csv.reader(f)
        header = next(r, None)
        if not header or TIME_COL not in header:
            return []
        i = header.index(TIME_COL)
        out = []
        for row in r:
            try:
                out.append(float(row[i]))
            except (ValueError, IndexError):
                continue
        return out


def convert(data_path, out_path, log_path=None, ac=None,
            maneuver=None, commit=None, t0=None):
    log_path = sdlog2scope.find_log(data_path, log_path)
    sdlog2scope.convert(data_path, out_path, log_path=log_path, ac_filter=ac)

    n_alias = apply_aliases(out_path)
    times = times_of(out_path)
    name, started = identity_from_log(log_path)
    ac_id = ac_id_from_data(data_path)

    meta = sidecar.build(
        "flight", data_path, times,
        aircraft=name,
        ac_id=ac_id,
        timestamp=started,
        maneuver=maneuver,
        firmware_commit=commit,
        t0_offset_s=t0,
        time_column=TIME_COL,
        aliases_applied=n_alias or None,
        log_file=os.path.basename(log_path) if log_path else None,
    )
    return out_path, sidecar.write(out_path, meta)
