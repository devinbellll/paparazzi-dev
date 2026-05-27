#!/usr/bin/env python3
"""
Launch ANTON NPS simulation and print live aircraft state to the terminal.
Sends a takeoff command sequence 1 second after the sim is ready.

Usage:  python3 sim_anton.py
        Ctrl-C to stop (kills child processes cleanly).

Takeoff sequence (flight plan blocks):
  Block 3 "Start Engine" → NavResurrect() un-kills throttle
  Block 4 "Takeoff"      → climbs to CLIMB waypoint at nav.climb_vspeed

Command delivery:
  Commands are sent as pprz binary BLOCK messages over UDP to port 4243
  (the sim's primary datalink input — DOWNLINK_DEVICE=udp0, UDP0_PORT_IN=4243).
  pprz_dl_event() parses them and calls nav_goto_block() directly.
"""

import os
import sys
import signal
import socket
import subprocess
import threading
import time
import math
import struct
from ivy.std_api import IvyInit, IvyStart, IvyStop, IvyBindMsg

PPRZ    = "/workspace/paparazzi"
SIMSITL = f"{PPRZ}/var/aircrafts/ANTON_MFC/nps/simsitl"
SERVER  = f"{PPRZ}/sw/ground_segment/tmtc/server"
IVY_BUS = "127.255.255.255:2010"
AC_ID   = 218
R2D     = math.degrees(1)
SIM_HOST = "127.0.0.1"
SIM_PORT = 4243          # UDP0_PORT_IN — sim's primary datalink receive port

# ── pprz binary encoding ─────────────────────────────────────────────────────
STX              = 0x99
CLASS_DATALINK   = 2
MSG_BLOCK        = 5   # datalink::BLOCK — fields: block_id(u8), ac_id(u8)

def pprz_block_frame(block_id: int) -> bytes:
    """
    Encode a BLOCK message for nav_goto_block().
    Frame layout: STX | LEN | SENDER | RECEIVER | COMP_CLASS | MSG_ID | block_id | ac_id | CKA | CKB
    Checksum covers bytes [1..] (all except STX).
    """
    sender    = 0          # ground
    receiver  = AC_ID
    comp_cls  = (0 << 4) | CLASS_DATALINK
    payload   = bytes([block_id, AC_ID])
    length    = 8 + len(payload)   # total frame length without checksums
    header    = struct.pack("BBBBBB", STX, length, sender, receiver, comp_cls, MSG_BLOCK) + payload
    ck_a = ck_b = 0
    for b in header[1:]:           # skip STX
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return header + struct.pack("BB", ck_a, ck_b)


# ── shared state ────────────────────────────────────────────────────────────
state = {
    "phi": 0.0, "theta": 0.0, "psi": 0.0,   # deg
    "p":   0.0, "q":     0.0, "r":   0.0,   # deg/s
    "lat": 0.0, "lon":   0.0,                # deg
    "alt": 0.0, "agl":   0.0,               # m MSL, m AGL
    "vx":  0.0, "vy":    0.0, "vz":  0.0,   # m/s NED
    "ax":  0.0, "ay":    0.0, "az":  0.0,   # m/s² body
    "t":   0.0,
    "cmd": "",
}

# ── Ivy callbacks ────────────────────────────────────────────────────────────
def on_rate_attitude(agent, msg):
    parts = msg.split()
    if len(parts) < 8:
        return
    state["p"], state["q"], state["r"] = float(parts[2]), float(parts[3]), float(parts[4])
    state["phi"], state["theta"], state["psi"] = float(parts[5]), float(parts[6]), float(parts[7])
    state["t"] = time.monotonic()

def on_pos_llh(agent, msg):
    parts = msg.split()
    if len(parts) < 10:
        return
    state["lat"] = float(parts[2]) * R2D
    state["lon"] = float(parts[5]) * R2D
    state["alt"] = float(parts[7])
    state["agl"] = float(parts[9])

def on_speed_pos(agent, msg):
    parts = msg.split()
    if len(parts) < 8:
        return
    state["vx"], state["vy"], state["vz"] = float(parts[5]), float(parts[6]), float(parts[7])

def on_sensors(agent, msg):
    parts = msg.split()
    if len(parts) < 5:
        return
    state["ax"], state["ay"], state["az"] = float(parts[2]), float(parts[3]), float(parts[4])


# ── display ──────────────────────────────────────────────────────────────────
def bar(val, lo, hi, width=20, unit=""):
    frac = max(0.0, min(1.0, (val - lo) / (hi - lo)))
    filled = int(frac * width)
    return f"[{'█' * filled}{'░' * (width - filled)}] {val:+8.2f}{unit}"

CLEAR  = "\033[H\033[J"
BOLD   = "\033[1m"
RESET  = "\033[0m"
CYAN   = "\033[36m"
GREEN  = "\033[32m"
YELLOW = "\033[33m"
GREY   = "\033[90m"

def render():
    s = state
    age = time.monotonic() - s["t"]
    stale = age > 1.0
    status   = f"{GREY}stale ({age:.1f}s){RESET}" if stale else f"{GREEN}live{RESET}"
    cmd_line = f"  {YELLOW}CMD:{RESET} {s['cmd']}" if s["cmd"] else f"  {GREY}no command sent yet{RESET}"

    lines = [
        f"{BOLD}{'─' * 52}{RESET}",
        f"  {BOLD}{CYAN}ANTON NPS Simulation{RESET}   {status}",
        cmd_line,
        f"{'─' * 52}",
        "",
        f"  {BOLD}POSITION{RESET}",
        f"    Lat   {s['lat']:+12.6f} °",
        f"    Lon   {s['lon']:+12.6f} °",
        f"    Alt   {s['alt']:+10.2f} m MSL",
        f"    AGL   {s['agl']:+10.2f} m",
        "",
        f"  {BOLD}ATTITUDE{RESET}",
        f"    Roll  {bar(s['phi'],   -45, 45, unit='°')}",
        f"    Pitch {bar(s['theta'], -45, 45, unit='°')}",
        f"    Yaw   {s['psi']:+8.2f} °",
        "",
        f"  {BOLD}RATES  (body){RESET}",
        f"    p     {bar(s['p'], -60, 60, unit='°/s')}",
        f"    q     {bar(s['q'], -60, 60, unit='°/s')}",
        f"    r     {bar(s['r'], -60, 60, unit='°/s')}",
        "",
        f"  {BOLD}VELOCITY  (NED){RESET}",
        f"    Vn  {s['vx']:+8.3f} m/s   Ve  {s['vy']:+8.3f} m/s",
        f"    Vd  {s['vz']:+8.3f} m/s   |V| {math.hypot(s['vx'], s['vy']):.3f} m/s",
        "",
        f"  {BOLD}ACCEL  (body){RESET}",
        f"    X {s['ax']:+8.3f} m/s²   Y {s['ay']:+8.3f} m/s²   Z {s['az']:+8.3f} m/s²",
        "",
        f"{'─' * 52}",
        f"  {GREY}Ctrl-C to stop{RESET}",
    ]
    sys.stdout.write(CLEAR + "\n".join(lines) + "\n")
    sys.stdout.flush()


# ── command sender ───────────────────────────────────────────────────────────
def send_block(sock: socket.socket, block_id: int, label: str):
    frame = pprz_block_frame(block_id)
    sock.sendto(frame, (SIM_HOST, SIM_PORT))
    state["cmd"] = f"BLOCK {block_id} ({label})  [{frame.hex()}]"

def takeoff_sequence(sock: socket.socket):
    """1 s after Ivy connects: Start Engine → Takeoff."""
    time.sleep(1.0)
    send_block(sock, 3, "Start Engine — NavResurrect")
    time.sleep(0.5)
    send_block(sock, 4, "Takeoff — climb to CLIMB wp")


# ── main ─────────────────────────────────────────────────────────────────────
def main():
    env = {**os.environ, "PAPARAZZI_HOME": PPRZ}

    print("Starting Paparazzi server …")
    server = subprocess.Popen(
        [SERVER, "-b", IVY_BUS, "-n"],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )

    print("Starting ANTON NPS sim …")
    sim = subprocess.Popen(
        [SIMSITL, "--norc"],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )

    # UDP socket for sending binary pprz commands to the sim's primary datalink
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def shutdown(sig=None, frame=None):
        print("\n\nShutting down …")
        sim.terminate()
        server.terminate()
        sock.close()
        try:
            IvyStop()
        except Exception:
            pass
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    print("Waiting for sim to start …")
    time.sleep(3)

    IvyInit("anton_monitor", "READY", None, None, None)
    IvyStart(IVY_BUS)
    IvyBindMsg(on_rate_attitude, r"(\d+ NPS_RATE_ATTITUDE .*)")
    IvyBindMsg(on_pos_llh,       r"(\d+ NPS_POS_LLH .*)")
    IvyBindMsg(on_speed_pos,     r"(\d+ NPS_SPEED_POS .*)")
    IvyBindMsg(on_sensors,       r"(\d+ NPS_SENSORS_SCALED .*)")

    state["t"] = time.monotonic()
    threading.Thread(target=takeoff_sequence, args=(sock,), daemon=True).start()

    while True:
        render()
        time.sleep(0.1)


if __name__ == "__main__":
    main()
