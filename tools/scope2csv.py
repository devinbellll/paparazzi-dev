#!/usr/bin/env python3
"""scope2csv.py — write NPS scope packets to the wide `/uav/...` CSV schema.

This is the sim-side counterpart of sdlog2scope.py: both produce the SAME
on-disk format (one time-indexed comma CSV, canonical `/uav/<BRANCH>/<field>`
columns), so a sim run and a real flight load into the same PlotJuggler layout
and the same analysis script.

    NPS scope (nps_scope.c, UDP JSON)  --scope2csv-->  mfc_sim_<TS>.csv
    SD card .TLM --sd2log--> .log/.data --sdlog2scope--> FLIGHT_pj.csv

The scope registers its variable set once at init and emits every registered
name in every datagram, so the column set is fixed by the first packet: we take
the header from packet 1 and forward-fill nothing. The outer "timestamp" field
carries sim time (fdm.time) and becomes the leading `time` column.

Used live by sim_anton.py (ScopeCsvWriter), and standalone to convert a capture
that was recorded as newline-delimited JSON:

    python3 scope2csv.py capture.jsonl -o run.csv
"""

import argparse
import csv
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from pj_json_relay import normalize_obj, sanitize  # noqa: E402

TIME_COL = "time"


def flatten(obj, prefix=""):
    """{"uav": {"MFC_STAB": {"sp_phi": v}}} -> {"/uav/MFC_STAB/sp_phi": v}.

    The scope emits some branches as already-flat slash-joined keys
    ("MFC_STAB/sp_phi": v) and others as nested dicts ("TRUTH": {...}); both
    land on the same column name because the separator is identical.
    """
    out = {}
    for key, val in obj.items():
        path = f"{prefix}/{key}"
        if isinstance(val, dict):
            out.update(flatten(val, path))
        elif isinstance(val, list):
            # arrays are registered per-element by nps_scope_register_array, so
            # a genuine list here is rare; index it rather than dropping it.
            for i, item in enumerate(val):
                out[f"{path}_{i}"] = item
        else:
            out[path] = val
    return out


def row_of_packet(obj):
    """Normalized packet -> (time, {column: value}). Returns (None, {}) if empty."""
    obj = normalize_obj(obj)
    t = obj.get("timestamp")
    body = {k: v for k, v in obj.items() if isinstance(v, dict)}
    return t, flatten(body)


WARMUP_PACKETS = 400
WARMUP_SECONDS = 3.0


class ScopeCsvWriter:
    """Streaming writer: learn the columns, then emit one row per timestamp.

    A CSV header has to be written before the rows, but the two feeds reveal
    their columns very differently:

      * the NPS scope puts EVERY registered variable in EVERY datagram, so the
        column set is complete from packet 1;
      * server's ivy stream sends ONE message per datagram, so the columns only
        accumulate as each message type comes round.

    Taking the header from the first packet therefore silently truncated the ivy
    feed to whichever message happened to arrive first. Instead, buffer a short
    warmup window (WARMUP_PACKETS packets or WARMUP_SECONDS of stream time,
    whichever comes first), take the header from the union of everything seen,
    then stream. Values forward-fill across packets and a row is emitted per
    distinct timestamp -- the same model tools/sdlog2scope.py uses, so the two
    converters produce comparable files.

    Columns first seen after warmup are dropped (with one warning) rather than
    corrupting row alignment.
    """

    def __init__(self, path, columns=None):
        self.path = path
        self._f = None
        self._writer = None
        self._cols = None
        self._warned = False
        self._warmup = []            # [(t, flat)] buffered until the header is known
        self._t0 = None
        self._row = {}               # forward-filled state
        self._pending_t = None
        self.n_rows = 0
        if columns is not None:      # offline: exact column set known up front
            self._start(columns)

    # ── internals ────────────────────────────────────────────────────────────
    def _start(self, cols):
        self._cols = sorted(cols)
        self._f = open(self.path, "w", newline="", buffering=1)
        self._writer = csv.writer(self._f)
        self._writer.writerow([TIME_COL] + self._cols)

    def _emit(self, t):
        self._writer.writerow([t] + [self._row.get(c, "") for c in self._cols])
        self.n_rows += 1

    def _feed(self, t, flat):
        """Apply one packet to the forward-filled row, emitting on time change."""
        if self._pending_t is not None and t != self._pending_t:
            self._emit(self._pending_t)
        self._pending_t = t
        if not self._warned and not set(flat) <= set(self._cols):
            extra = sorted(set(flat) - set(self._cols))[:5]
            print("scope2csv: column(s) first seen after warmup, dropped: "
                  f"{', '.join(extra)}", file=sys.stderr)
            self._warned = True
        self._row.update(flat)

    def _flush_warmup(self):
        cols = set()
        for _, flat in self._warmup:
            cols.update(flat)
        self._start(cols)
        for t, flat in self._warmup:
            self._feed(t, flat)
        self._warmup = []

    # ── public ───────────────────────────────────────────────────────────────
    def write_packet(self, obj):
        t, flat = row_of_packet(obj)
        if not flat:
            return
        if self._cols is None:
            if self._t0 is None:
                self._t0 = t
            self._warmup.append((t, flat))
            elapsed = (t - self._t0) if isinstance(t, (int, float)) and isinstance(self._t0, (int, float)) else 0
            if len(self._warmup) >= WARMUP_PACKETS or elapsed >= WARMUP_SECONDS:
                self._flush_warmup()
            return
        self._feed(t, flat)

    def close(self):
        if self._cols is None and self._warmup:
            self._flush_warmup()       # run shorter than the warmup window
        if self._writer is not None and self._pending_t is not None:
            self._emit(self._pending_t)
            self._pending_t = None
        if self._f is not None:
            self._f.close()
            self._f = None


def iter_packets(in_path):
    """Yield parsed, sanitized packets from a newline-delimited JSON capture."""
    with open(in_path, "rb") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            clean = sanitize(line)
            if clean is None:
                continue
            try:
                yield json.loads(clean)
            except json.JSONDecodeError:
                continue


def convert(in_path, out_path):
    """Two-pass: the file is seekable, so take the exact column union rather
    than the live path's warmup approximation."""
    cols = set()
    for obj in iter_packets(in_path):
        cols.update(row_of_packet(obj)[1])
    if not cols:
        sys.exit(f"scope2csv: no usable packets in {in_path}")

    w = ScopeCsvWriter(out_path, columns=cols)
    try:
        for obj in iter_packets(in_path):
            w.write_packet(obj)
    finally:
        w.close()
    return w.n_rows, len(w._cols or [])


def main():
    ap = argparse.ArgumentParser(
        description="Convert a captured NPS scope .jsonl to the /uav wide CSV schema.")
    ap.add_argument("capture", help="newline-delimited JSON scope capture")
    ap.add_argument("-o", "--out", help="output CSV (default: <capture>.csv)")
    args = ap.parse_args()

    out_path = args.out or os.path.splitext(args.capture)[0] + ".csv"
    n, cols = convert(args.capture, out_path)
    print(f"wrote {out_path} ({n} rows, {cols + 1} columns)")


if __name__ == "__main__":
    main()
