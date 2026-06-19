#!/usr/bin/env python3
"""
pprz_ctrl.py — GCS shell: send Paparazzi datalink commands to a running sim.

The Paparazzi binary protocol (pprz v2) uses the same UDP port 4243 that the
sim's link listens on.  This script encodes messages in that protocol and fires
them directly — no Ivy bus, no OCaml GCS required.  Run it alongside a running
sim_anton.py session.

Usage:
  python3 pprz_ctrl.py AIRCRAFT block <block_id>
  python3 pprz_ctrl.py AIRCRAFT switch <indi|mfc>
  python3 pprz_ctrl.py AIRCRAFT setting <index> <float_value>

Examples:
  python3 pprz_ctrl.py ANTON_MFC block 5          # jump to flight plan block 5
  python3 pprz_ctrl.py ANTON_MFC switch mfc        # hand motor authority to MFC
  python3 pprz_ctrl.py ANTON_MFC switch indi       # hand motor authority back to INDI
  python3 pprz_ctrl.py ANTON_MFC setting 47 1.0   # equivalent to the switch command above

The setting index for dual_ctrl_active is 47 on ANTON_MFC.  If you rebuild
with different modules and the index shifts, find the new value with:
  grep -n "dual_ctrl_active" paparazzi/var/aircrafts/ANTON_MFC/ap/generated/settings.h

Why this works:
  sim_anton.py already uses binary pprz frames over UDP for BLOCK messages.
  SETTING (msg_id=4) and BLOCK (msg_id=5) share the same frame structure:
    STX | LEN | SENDER | RECEIVER | COMP_CLASS | MSG_ID | <payload> | CKA | CKB
  The sim's pprz_dl_event() parses any well-formed frame on port 4243.
"""

import os
import socket
import struct
import sys
import xml.etree.ElementTree as ET

PPRZ    = "/workspace/paparazzi"
_CONF   = os.environ.get("CONF") or "conf/airframes/ENAC/conf_enac.xml"

SIM_HOST = "127.0.0.1"
SIM_PORT = 4243

STX            = 0x99
CLASS_DATALINK = 2
MSG_BLOCK      = 5
MSG_SETTING    = 4

DUAL_CTRL_IDX  = 47   # settings.h index for dual_ctrl_active on ANTON_MFC
DUAL_CTRL_INDI = 0
DUAL_CTRL_MFC  = 1


def lookup_ac_id(name: str) -> int:
    conf_path = os.path.join(PPRZ, _CONF)
    try:
        tree = ET.parse(conf_path)
        for ac in tree.getroot().findall("aircraft"):
            if ac.get("name") == name:
                return int(ac.get("ac_id"))
    except Exception as e:
        print(f"Error parsing conf: {e}", file=sys.stderr)
    print(f"Error: aircraft '{name}' not found in {conf_path}", file=sys.stderr)
    sys.exit(1)


def _frame(ac_id: int, msg_id: int, payload: bytes) -> bytes:
    sender   = 0
    comp_cls = (0 << 4) | CLASS_DATALINK
    length   = 8 + len(payload)
    header   = struct.pack("BBBBBB", STX, length, sender, ac_id, comp_cls, msg_id) + payload
    ck_a = ck_b = 0
    for b in header[1:]:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return header + struct.pack("BB", ck_a, ck_b)

def block_frame(ac_id: int, block_id: int) -> bytes:
    return _frame(ac_id, MSG_BLOCK, bytes([block_id, ac_id]))

def setting_frame(ac_id: int, index: int, value: float) -> bytes:
    return _frame(ac_id, MSG_SETTING, struct.pack("BBf", index, ac_id, value))


def send(frame: bytes):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(frame, (SIM_HOST, SIM_PORT))
    sock.close()
    print(f"sent {len(frame)} bytes → {SIM_HOST}:{SIM_PORT}  [{frame.hex()}]")


def main():
    args = sys.argv[1:]
    if len(args) < 2:
        print(__doc__)
        sys.exit(1)

    ac_name = args[0]
    cmd     = args[1]
    ac_id   = lookup_ac_id(ac_name)

    if cmd == "block":
        if len(args) < 3:
            print("Usage: pprz_ctrl.py AIRCRAFT block <block_id>", file=sys.stderr)
            sys.exit(1)
        block_id = int(args[2])
        frame = block_frame(ac_id, block_id)
        print(f"[block] ac={ac_id} block={block_id}")
        send(frame)

    elif cmd == "switch":
        if len(args) < 3 or args[2] not in ("indi", "mfc"):
            print("Usage: pprz_ctrl.py AIRCRAFT switch <indi|mfc>", file=sys.stderr)
            sys.exit(1)
        law = DUAL_CTRL_MFC if args[2] == "mfc" else DUAL_CTRL_INDI
        frame = setting_frame(ac_id, DUAL_CTRL_IDX, float(law))
        print(f"[switch] ac={ac_id} → {args[2].upper()} (setting idx={DUAL_CTRL_IDX} val={law})")
        send(frame)

    elif cmd == "setting":
        if len(args) < 4:
            print("Usage: pprz_ctrl.py AIRCRAFT setting <index> <value>", file=sys.stderr)
            sys.exit(1)
        idx   = int(args[2])
        value = float(args[3])
        frame = setting_frame(ac_id, idx, value)
        print(f"[setting] ac={ac_id} idx={idx} val={value}")
        send(frame)

    else:
        print(f"Unknown command: {cmd}", file=sys.stderr)
        print("Commands: block, switch, setting", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
