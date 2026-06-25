#!/usr/bin/env python3
"""
sdlog2scope.py — convert a Paparazzi flight log (.data) into the NPS_SCOPE JSON
schema, so a real SD-card flight loads/analyses identically to an NPS sim capture.

Pipeline (per the MFC Flight-Test Enablement plan, Phase 3):

    *.TLM (SD card)  --sd2log-->  .log + .data  --sdlog2scope.py-->  scope JSON

This tool consumes the decoded `.data` text log (whitespace-separated
"time ac_id MSG field1 field2 …", message fields in messages.xml order). It emits
one JSON object per timestamp, keyed with the SAME scope keys the firmware
registers in NPS via NPS_SCOPE_VAR (mfc/*, mfc_g/*, wls/*, truth/*) — see
stabilization_mfc.c and guidance_mfc.c. The output is newline-delimited JSON
(one object per line), loadable in PlotJuggler and by analyze_mfc.py.

The firmware writes the SAME binary pprzlog to the card regardless; only this
offline converter is new, and its output schema == the scope schema. That single
message+field -> scope-key mapping table below is the source of cross-compatibility
between "Ivy/pprzlink messages" and "the custom JSON logging type".

Usage:
    python3 sdlog2scope.py FLIGHT.data            # -> stdout (ndjson)
    python3 sdlog2scope.py FLIGHT.data -o out.json
    python3 sdlog2scope.py FLIGHT.data --ac 111   # only this ac_id

To decode a raw SD .TLM first (needs the Paparazzi ground segment / sd2log):
    sd2log FLIGHT.TLM   # produces FLIGHT.log + FLIGHT.data
"""

import argparse
import json
import sys

R2D = 57.29577951308232

# ── message field order (must match messages.xml) ─────────────────────────────
# STAB_MFC (id 212): per-axis sp/meas/err/fk/cmd + 4 motor outputs.
STAB_MFC_FIELDS = [
    "sp_phi", "sp_theta", "sp_psi",
    "me_phi", "me_theta", "me_psi",
    "err_phi", "err_theta", "err_psi",
    "fk_phi", "fk_theta", "fk_psi",
    "cmd_phi", "cmd_theta", "cmd_psi",
    "u0", "u1", "u2", "u3",
]
# GUIDANCE_MFC (id 57): per-axis pos sp/meas/err/fk + accel|thrust cmd.
GUIDANCE_MFC_FIELDS = [
    "sp_x", "sp_y", "sp_z",
    "me_x", "me_y", "me_z",
    "err_x", "err_y", "err_z",
    "fk_x", "fk_y", "fk_z",
    "cmd_x", "cmd_y", "cmd_z",
]

# ── message+field -> scope-key mapping (THE cross-compatibility contract) ──────
# value = scope key, or ("array_key", index) to pack into a JSON array.
STAB_MFC_MAP = {
    "sp_phi":   "mfc/roll/sp",  "sp_theta": "mfc/pitch/sp",  "sp_psi":   "mfc/yaw/sp",
    "me_phi":   "mfc/roll/meas", "me_theta": "mfc/pitch/meas", "me_psi":  "mfc/yaw/meas",
    "err_phi":  "mfc/roll/err", "err_theta": "mfc/pitch/err", "err_psi": "mfc/yaw/err",
    "fk_phi":   "mfc/roll/fk",  "fk_theta": "mfc/pitch/fk",   "fk_psi":  "mfc/yaw/fk",
    "cmd_phi":  "mfc/roll/cmd", "cmd_theta": "mfc/pitch/cmd", "cmd_psi": "mfc/yaw/cmd",
    "u0": ("mfc/act", 0), "u1": ("mfc/act", 1), "u2": ("mfc/act", 2), "u3": ("mfc/act", 3),
}
GUIDANCE_MFC_MAP = {
    "sp_x":  "mfc_g/x/sp",   "sp_y":  "mfc_g/y/sp",   "sp_z":  "mfc_g/z/sp",
    "me_x":  "mfc_g/x/meas", "me_y":  "mfc_g/y/meas", "me_z":  "mfc_g/z/meas",
    "err_x": "mfc_g/x/err",  "err_y": "mfc_g/y/err",  "err_z": "mfc_g/z/err",
    "fk_x":  "mfc_g/x/fk",   "fk_y":  "mfc_g/y/fk",   "fk_z":  "mfc_g/z/fk",
    "cmd_x": "mfc_g/x/cmd",  "cmd_y": "mfc_g/y/cmd",  "cmd_z": "mfc_g/z/cmd",
}


def _floats(tok):
    """Parse a log value token; arrays are comma-joined in one token."""
    if "," in tok:
        return [float(x) for x in tok.split(",")]
    return float(tok)


def convert(data_path, ac_filter=None):
    """Yield one scope-JSON dict per STAB_MFC sample, forward-filling the slower
    GUIDANCE_MFC / WLS / truth values (mirrors the scope's one-datagram-per-step).
    STAB_MFC is the row trigger because it is the highest-rate MFC message."""
    # rolling "most recent value" store, reset only by new messages
    cur = {}

    def set_key(key, val):
        if isinstance(key, tuple):
            arr_key, idx = key
            arr = cur.setdefault(arr_key, [])
            while len(arr) <= idx:
                arr.append(0.0)
            arr[idx] = val
        else:
            cur[key] = val

    with open(data_path) as f:
        for line in f:
            args = line.split()
            if len(args) < 3:
                continue
            try:
                t = float(args[0])
            except ValueError:
                continue
            ac_id = args[1]
            if ac_filter is not None and ac_id != ac_filter:
                continue
            msg = args[2]
            vals = args[3:]

            if msg == "STAB_MFC" and len(vals) >= len(STAB_MFC_FIELDS):
                for name, tok in zip(STAB_MFC_FIELDS, vals):
                    set_key(STAB_MFC_MAP[name], _floats(tok))
                # truth/* from the MFC measured attitude (no JSBSim truth in flight):
                # me_* are radians -> scope truth uses degrees.
                cur["truth/phi"]   = cur["mfc/roll/meas"]  * R2D
                cur["truth/theta"] = cur["mfc/pitch/meas"] * R2D
                cur["truth/psi"]   = cur["mfc/yaw/meas"]   * R2D
                row = dict(cur)
                row["t"] = t
                yield row

            elif msg == "GUIDANCE_MFC" and len(vals) >= len(GUIDANCE_MFC_FIELDS):
                for name, tok in zip(GUIDANCE_MFC_FIELDS, vals):
                    set_key(GUIDANCE_MFC_MAP[name], _floats(tok))
                # position truth/* + AGL from the guidance measured NED position.
                cur["truth/x"]   = cur["mfc_g/x/meas"]
                cur["truth/y"]   = cur["mfc_g/y/meas"]
                cur["truth/z"]   = cur["mfc_g/z/meas"]
                cur["truth/agl"] = -cur["mfc_g/z/meas"]   # NED down -> AGL up

            elif msg == "WLS_V":
                cur["wls/v"] = _floats(vals[0]) if vals else []
            elif msg == "WLS_U":
                cur["wls/u"] = _floats(vals[0]) if vals else []


def main():
    ap = argparse.ArgumentParser(description="Convert a Paparazzi .data log to NPS_SCOPE JSON.")
    ap.add_argument("data", help="decoded log .data file (from sd2log / GCS server)")
    ap.add_argument("-o", "--out", help="output file (default: stdout)")
    ap.add_argument("--ac", help="only convert this ac_id (e.g. 111)")
    args = ap.parse_args()

    out = open(args.out, "w") if args.out else sys.stdout
    n = 0
    try:
        for row in convert(args.data, ac_filter=args.ac):
            out.write(json.dumps(row) + "\n")
            n += 1
    finally:
        if args.out:
            out.close()
    print(f"sdlog2scope: wrote {n} scope rows{' to ' + args.out if args.out else ''}",
          file=sys.stderr)
    if n == 0:
        print("  (no STAB_MFC rows found — check the log has the MFC FlightRecorder "
              "messages and the right --ac id)", file=sys.stderr)


if __name__ == "__main__":
    main()
