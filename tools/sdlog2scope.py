#!/usr/bin/env python3
"""sdlog2scope.py — convert a raw Paparazzi flight log (.log + .data) into the
wide `/uav/...` CSV schema used by plotjuggler_mfc.xml.

Same output contract as convert_sd_to_pj.py (identical BRANCH_MAP, identical
`/uav/<BRANCH>/<field>` column names), but it reads the *raw* decoded log
instead of the GCS-exported CSV. That means full message rate (every sample the
recorder wrote) rather than the exporter's fixed 4 Hz resample, and no manual
CSV export step:

    *.TLM (SD card)  --sd2log-->  .log + .data  --sdlog2scope.py-->  _pj.csv

The `.log` is a Paparazzi XML log header; its `<protocol>` section carries the
full message/field definitions, so field names come from the log itself — there
is no hardcoded message table to keep in sync with messages.xml.

Rows are built by forward-filling: every message updates its columns, and a row
is emitted per distinct timestamp (or per `--trigger` message). Values are
written verbatim as the log stored them (raw units, no scaling), matching what
the GCS CSV export produces. Array fields stay comma-joined in a single quoted
cell, exactly as convert_sd_to_pj.py leaves them.

Usage:
    python3 sdlog2scope.py FLIGHT.data                    # -> FLIGHT_pj.csv
    python3 sdlog2scope.py FLIGHT.data -o out.csv
    python3 sdlog2scope.py FLIGHT.data --ac 177           # only this ac_id
    python3 sdlog2scope.py FLIGHT.data --trigger STAB_MFC # one row per STAB_MFC
    python3 sdlog2scope.py FLIGHT.data -m STAB_MFC,GUIDANCE_MFC,WLS_U

To decode a raw SD .TLM first (needs the Paparazzi ground segment):
    sd2log FLIGHT.TLM   # produces FLIGHT.log + FLIGHT.data
"""

import argparse
import csv
import datetime
import os
import sys
import xml.etree.ElementTree as ET

# Single source of truth for the branch renaming — shared with convert_sd_to_pj.py.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from convert_sd_to_pj import BRANCH_MAP
except ImportError:  # standalone copy of this file
    BRANCH_MAP = {
        "STAB_MFC": "MFC_STAB",
        "GUIDANCE_MFC": "MFC_GUIDANCE",
        "GUIDANCE_MFC_ACC2ATT": "MFC_ACC2ATT",
        "ACC2ATT": "MFC_ACC2ATT",
    }

LEAD_COLS = ["Time", "UTC", "GPS_lat(deg)", "GPS_long(deg)"]


def column(msg, field):
    """`STAB_MFC`, `sp_phi` -> `/uav/MFC_STAB/sp_phi` (same as convert_sd_to_pj)."""
    return f"/uav/{BRANCH_MAP.get(msg, msg)}/{field}"


def parse_protocol(log_path):
    """Return ({msg_name: [field names]}, time_of_day) from the .log XML header.

    Messages are collected from every msg_class; the telemetry class wins on a
    name clash, since that is what the airborne recorder writes.
    """
    root = ET.parse(log_path).getroot()
    fields = {}
    for msg_class in root.iter("msg_class"):
        telemetry = msg_class.get("NAME") == "telemetry"
        for msg in msg_class.findall("message"):
            name = msg.get("NAME")
            if name in fields and not telemetry:
                continue
            fields[name] = [f.get("NAME") for f in msg.findall("field")]
    try:
        time_of_day = float(root.get("time_of_day"))
    except (TypeError, ValueError):
        time_of_day = None
    return fields, time_of_day


def find_log(data_path, explicit=None):
    if explicit:
        return explicit
    guess = os.path.splitext(data_path)[0] + ".log"
    if not os.path.exists(guess):
        sys.exit(f"sdlog2scope: no .log next to {data_path} (looked for {guess}); "
                 "pass it with --log")
    return guess


def iter_records(data_path, ac_filter=None):
    """Yield (t, msg, [value tokens]) from the whitespace-separated .data log."""
    with open(data_path) as f:
        for line in f:
            args = line.split()
            if len(args) < 3:
                continue
            try:
                t = float(args[0])
            except ValueError:
                continue
            if ac_filter is not None and args[1] != ac_filter:
                continue
            yield t, args[2], args[3:]


def scan_messages(data_path, ac_filter=None):
    """First pass: which messages actually occur, in order of first appearance."""
    seen = {}
    for _, msg, _ in iter_records(data_path, ac_filter):
        seen.setdefault(msg, None)
    return list(seen)


def clean(tok):
    """Log tokens are raw; only string/enum fields carry surrounding quotes."""
    if len(tok) >= 2 and tok[0] == '"' and tok[-1] == '"':
        return tok[1:-1]
    return tok


def utc_string(time_of_day, t):
    if time_of_day is None:
        return ""
    # The GCS export floors time_of_day to the second — match it exactly.
    stamp = datetime.datetime.fromtimestamp(int(time_of_day) + t,
                                            datetime.timezone.utc)
    return stamp.strftime("%H:%M:%S.%f")[:-3]


def convert(data_path, out_path, log_path=None, ac_filter=None,
            messages=None, trigger=None):
    proto, time_of_day = parse_protocol(find_log(data_path, log_path))

    present = scan_messages(data_path, ac_filter)
    if messages:
        wanted = [m for m in messages if m in present]
        missing = [m for m in messages if m not in present]
        if missing:
            print(f"sdlog2scope: not in the log, skipped: {', '.join(missing)}",
                  file=sys.stderr)
    else:
        wanted = present

    unknown = [m for m in wanted if m not in proto]
    if unknown:
        print("sdlog2scope: no protocol definition (dropped): "
              f"{', '.join(unknown)}", file=sys.stderr)
    wanted = [m for m in wanted if m in proto]
    if not wanted:
        sys.exit("sdlog2scope: nothing to convert — no known messages in the log")
    if trigger and trigger not in wanted:
        sys.exit(f"sdlog2scope: trigger message {trigger} is not in the log")

    kept = set(wanted)
    header = list(LEAD_COLS)
    columns = {}                     # msg -> [column index per field]
    for m in wanted:
        columns[m] = list(range(len(header), len(header) + len(proto[m])))
        header += [column(m, f) for f in proto[m]]

    gps_fields = proto.get("GPS_INT", [])
    lat_i = gps_fields.index("lat") if "lat" in gps_fields else None
    lon_i = gps_fields.index("lon") if "lon" in gps_fields else None

    row = [""] * len(header)         # rolling forward-filled state
    dirty = False                    # a kept message landed on pending_t
    pending_t = None
    n = 0

    with open(out_path, "w", newline="") as f_out:
        writer = csv.writer(f_out)
        writer.writerow(header)

        def emit(t):
            nonlocal n
            row[0] = f"{t:.4f}".rstrip("0").rstrip(".")
            row[1] = utc_string(time_of_day, t)
            writer.writerow(row)
            n += 1

        for t, msg, vals in iter_records(data_path, ac_filter):
            # Timestamp changed: the previous timestamp's row is complete.
            if t != pending_t:
                if trigger is None and dirty:
                    emit(pending_t)
                pending_t = t
                dirty = False

            if msg == "GPS_INT" and lat_i is not None and lon_i is not None \
                    and len(vals) > max(lat_i, lon_i):
                row[2] = f"{int(vals[lat_i]) * 1e-7:.9f}"
                row[3] = f"{int(vals[lon_i]) * 1e-7:.9f}"

            if msg not in kept:
                continue
            cols = columns[msg]
            for i, tok in enumerate(vals[:len(cols)]):
                row[cols[i]] = clean(tok)
            dirty = True

            if trigger is not None and msg == trigger:
                emit(t)

        if trigger is None and dirty:
            emit(pending_t)

    return n, len(header)


def main():
    ap = argparse.ArgumentParser(
        description="Convert a raw Paparazzi .log/.data flight log to the /uav "
                    "PlotJuggler CSV schema (same output as convert_sd_to_pj.py).")
    ap.add_argument("data", help="decoded log .data file (from sd2log)")
    ap.add_argument("-o", "--out", help="output CSV (default: <data>_pj.csv)")
    ap.add_argument("--log", help="the matching .log (default: alongside the .data)")
    ap.add_argument("--ac", help="only convert this ac_id (e.g. 177)")
    ap.add_argument("-m", "--messages",
                    help="comma-separated messages to keep (default: all)")
    ap.add_argument("--trigger", metavar="MSG",
                    help="emit one row per MSG instead of one per timestamp")
    args = ap.parse_args()

    out_path = args.out or os.path.splitext(args.data)[0] + "_pj.csv"
    messages = args.messages.split(",") if args.messages else None

    n, cols = convert(args.data, out_path, log_path=args.log, ac_filter=args.ac,
                      messages=messages, trigger=args.trigger)
    print(f"wrote {out_path} ({n} rows, {cols} columns)")
    if n == 0:
        print("  (no rows — check --ac matches the log's ac_id)", file=sys.stderr)


if __name__ == "__main__":
    main()
