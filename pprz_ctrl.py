#!/usr/bin/env python3
"""
pprz_ctrl.py — send interactive commands to a running sim_anton.py session.

sim_anton.py accepts text commands on its stdin.  This script finds the
running paparazzi-build container and writes a command line to PID 1's stdin
via /proc/1/fd/0 (the sim's stdin fd, reachable from any shell in the same
container namespace).

Usage:
  python3 pprz_ctrl.py block <block_id>
  python3 pprz_ctrl.py switch <indi|mfc>
  python3 pprz_ctrl.py setting <index> <float_value>

Examples:
  python3 pprz_ctrl.py block 5          # jump to flight plan block 5
  python3 pprz_ctrl.py switch mfc       # hand motor authority to MFC
  python3 pprz_ctrl.py switch indi      # hand motor authority back to INDI
  python3 pprz_ctrl.py setting 47 1.0   # equivalent to 'switch mfc'

You can also type commands directly in the sim terminal window:
  switch mfc
  block 5
"""

import shlex
import subprocess
import sys

PPRZ_IMAGE = "paparazzi-build:latest"


def find_sim_container() -> str:
    result = subprocess.run(
        ["docker", "ps", "--filter", f"ancestor={PPRZ_IMAGE}", "--format", "{{.ID}}"],
        capture_output=True, text=True,
    )
    ids = result.stdout.strip().split()
    if not ids:
        print(
            f"No running container with image '{PPRZ_IMAGE}' found.\n"
            "Is the sim running?  Start it with:  ./sim.sh AIRCRAFT",
            file=sys.stderr,
        )
        sys.exit(1)
    return ids[0]


def send_cmd(cmd_str: str):
    cid = find_sim_container()
    subprocess.run(
        ["docker", "exec", cid, "sh", "-c",
         f"echo {shlex.quote(cmd_str)} > /proc/1/fd/0"],
        check=True,
    )
    print(f"→ {cmd_str!r}  (container {cid[:12]})")


def main():
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        sys.exit(1)

    cmd = args[0]

    if cmd == "block":
        if len(args) < 2:
            print("Usage: pprz_ctrl.py block <block_id>", file=sys.stderr)
            sys.exit(1)
        send_cmd(f"block {int(args[1])}")

    elif cmd == "switch":
        if len(args) < 2 or args[1] not in ("indi", "mfc"):
            print("Usage: pprz_ctrl.py switch <indi|mfc>", file=sys.stderr)
            sys.exit(1)
        send_cmd(f"switch {args[1]}")

    elif cmd == "setting":
        if len(args) < 3:
            print("Usage: pprz_ctrl.py setting <index> <value>", file=sys.stderr)
            sys.exit(1)
        send_cmd(f"setting {int(args[1])} {float(args[2])}")

    else:
        print(f"Unknown command: {cmd}", file=sys.stderr)
        print("Commands: block, switch, setting", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
