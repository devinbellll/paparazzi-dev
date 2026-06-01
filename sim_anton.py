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

Observability layers:
  Layer 0 — JSBSim truth:   NPS_RATE_ATTITUDE, NPS_POS_LLH, NPS_SPEED_POS,
                             NPS_GYRO_BIAS, NPS_SENSORS_SCALED, NPS_WIND
  Layer 1 — Firmware state: STAB_ATTITUDE (INDI att/rate/angular-accel),
                             STAB_MFC (MFC estimator, errors, commands),
                             ROTORCRAFT_CMD (roll/pitch/yaw/thrust ints)
  Logging: CSV file written to /tmp/mfc_sim_<timestamp>.csv
"""

import csv
import datetime
import math
import os
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
from collections import deque
from ivy.std_api import IvyInit, IvyStart, IvyStop, IvyBindMsg

PPRZ    = "/workspace/paparazzi"
SERVER  = f"{PPRZ}/sw/ground_segment/tmtc/server"
LINK    = f"{PPRZ}/sw/ground_segment/tmtc/link"
IVY_BUS = "127.255.255.255:2010"

_USE_MFC = "--mfc" in sys.argv
_GDB     = "--gdb" in sys.argv
AC_NAME  = "ANTON_MFC" if _USE_MFC else "ANTON"
AC_ID    = 218         if _USE_MFC else 217
SIMSITL  = f"{PPRZ}/var/aircrafts/{AC_NAME}/nps/simsitl"
R2D     = math.degrees(1)
SIM_HOST = "127.0.0.1"
SIM_PORT = 4243          # UDP0_PORT_IN — sim's primary datalink receive port

LOG_FILE       = f"/tmp/mfc_sim_{datetime.datetime.now():%Y%m%d_%H%M%S}.csv"
DEBUG_LOG_FILE = f"/tmp/mfc_sim_{datetime.datetime.now():%Y%m%d_%H%M%S}_debug.log"

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
    # Layer 0 — JSBSim truth (NPS_*)
    "phi": 0.0, "theta": 0.0, "psi": 0.0,   # deg (NPS_RATE_ATTITUDE)
    "p":   0.0, "q":     0.0, "r":   0.0,   # deg/s
    "lat": 0.0, "lon":   0.0,               # deg (NPS_POS_LLH)
    "alt": 0.0, "agl":   0.0,               # m MSL, m AGL
    "vx":  0.0, "vy":    0.0, "vz":  0.0,   # m/s NED (NPS_SPEED_POS)
    "ax":  0.0, "ay":    0.0, "az":  0.0,   # m/s² body (NPS_SENSORS_SCALED)
    "bias_p": 0.0, "bias_q": 0.0, "bias_r": 0.0,  # deg/s (NPS_GYRO_BIAS)
    "wind_n": 0.0, "wind_e": 0.0, "wind_d": 0.0,  # m/s NED (NPS_WIND)
    # Layer 1 — Firmware MFC state (STAB_MFC)
    "mfc_sp_phi":    0.0, "mfc_sp_theta":    0.0, "mfc_sp_psi":    0.0,
    "mfc_me_phi":    0.0, "mfc_me_theta":    0.0, "mfc_me_psi":    0.0,
    "mfc_err_phi":   0.0, "mfc_err_theta":   0.0, "mfc_err_psi":   0.0,
    "mfc_fk_phi":    0.0, "mfc_fk_theta":    0.0, "mfc_fk_psi":    0.0,
    "mfc_cmd_phi":   0.0, "mfc_cmd_theta":   0.0, "mfc_cmd_psi":   0.0,
    "mfc_u0":        0.0, "mfc_u1":          0.0,
    "mfc_u2":        0.0, "mfc_u3":          0.0,
    # Layer 1 — WLS virtual control inputs (WLS_V, v before allocation)
    "wls_v0":        0.0, "wls_v1":          0.0,
    "wls_v2":        0.0, "wls_v3":          0.0,
    # Layer 1 — INDI stabilizer state (STAB_ATTITUDE)
    "sa_att_phi":  0.0, "sa_att_theta":  0.0, "sa_att_psi":  0.0,  # rad
    "sa_ref_phi":  0.0, "sa_ref_theta":  0.0, "sa_ref_psi":  0.0,  # rad
    "sa_rate_p":   0.0, "sa_rate_q":     0.0, "sa_rate_r":   0.0,  # rad/s
    "sa_rref_p":   0.0, "sa_rref_q":     0.0, "sa_rref_r":   0.0,  # rad/s
    "sa_dacc_p":   0.0, "sa_dacc_q":     0.0, "sa_dacc_r":   0.0,  # rad/s² measured
    "sa_aref_p":   0.0, "sa_aref_q":     0.0, "sa_aref_r":   0.0,  # rad/s² INDI virtual cmd
    # Layer 1 — Firmware cmd ints (ROTORCRAFT_CMD)
    "rc_roll": 0, "rc_pitch": 0, "rc_yaw": 0, "rc_thrust": 0,
    "t":   0.0,
    "cmd": "",
}

# Ring buffer for printf output captured from simsitl stdout
debug_log: deque = deque(maxlen=20)

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

def on_gyro_bias(agent, msg):
    parts = msg.split()
    if len(parts) < 5:
        return
    state["bias_p"], state["bias_q"], state["bias_r"] = float(parts[2]), float(parts[3]), float(parts[4])

def on_wind(agent, msg):
    parts = msg.split()
    if len(parts) < 5:
        return
    state["wind_n"], state["wind_e"], state["wind_d"] = float(parts[2]), float(parts[3]), float(parts[4])

def on_stab_mfc(agent, msg):
    # Format: AC_ID STAB_MFC sp_phi sp_theta sp_psi me_phi me_theta me_psi
    #         err_phi err_theta err_psi fk_phi fk_theta fk_psi
    #         cmd_phi cmd_theta cmd_psi u0 u1 u2 u3   (21 tokens total)
    parts = msg.split()
    if len(parts) < 21:
        return
    i = 2
    state["mfc_sp_phi"],   state["mfc_sp_theta"],  state["mfc_sp_psi"]   = float(parts[i]),   float(parts[i+1]), float(parts[i+2])
    state["mfc_me_phi"],   state["mfc_me_theta"],  state["mfc_me_psi"]   = float(parts[i+3]), float(parts[i+4]), float(parts[i+5])
    state["mfc_err_phi"],  state["mfc_err_theta"], state["mfc_err_psi"]  = float(parts[i+6]), float(parts[i+7]), float(parts[i+8])
    state["mfc_fk_phi"],   state["mfc_fk_theta"],  state["mfc_fk_psi"]   = float(parts[i+9]), float(parts[i+10]),float(parts[i+11])
    state["mfc_cmd_phi"],  state["mfc_cmd_theta"], state["mfc_cmd_psi"]  = float(parts[i+12]),float(parts[i+13]),float(parts[i+14])
    state["mfc_u0"],       state["mfc_u1"]  = float(parts[i+15]), float(parts[i+16])
    state["mfc_u2"],       state["mfc_u3"]  = float(parts[i+17]), float(parts[i+18])

def on_wls_v_stab(agent, msg):
    # WLS_V layout: AC_ID WLS_V loop gamma iter v[,...] Wv[,...]
    parts = msg.split()
    if len(parts) < 6:
        return
    v = [float(x) for x in parts[5].split(',')]
    if len(v) >= 4:
        state["wls_v0"], state["wls_v1"] = v[0], v[1]
        state["wls_v2"], state["wls_v3"] = v[2], v[3]

def on_stab_attitude(agent, msg):
    # Link broadcasts float[] fields as comma-separated tokens (no spaces within array).
    # Layout: AC_ID STAB_ATTITUDE att_des att[3] att_ref[3] rate[3] rate_ref[3] ang_acc[3] ang_acc_ref[3] jerk u
    # parts:  [0]   [1]           [2]     [3]    [4]        [5]     [6]         [7]        [8]            [9]  [10]
    parts = msg.split()
    if len(parts) < 9:
        return
    att   = [float(x) for x in parts[3].split(',')]
    ref   = [float(x) for x in parts[4].split(',')]
    rate  = [float(x) for x in parts[5].split(',')]
    rref  = [float(x) for x in parts[6].split(',')]
    dacc  = [float(x) for x in parts[7].split(',')]
    aref  = [float(x) for x in parts[8].split(',')]
    state["sa_att_phi"],  state["sa_att_theta"], state["sa_att_psi"]  = att[0],  att[1],  att[2]
    state["sa_ref_phi"],  state["sa_ref_theta"], state["sa_ref_psi"]  = ref[0],  ref[1],  ref[2]
    state["sa_rate_p"],   state["sa_rate_q"],    state["sa_rate_r"]   = rate[0], rate[1], rate[2]
    state["sa_rref_p"],   state["sa_rref_q"],    state["sa_rref_r"]   = rref[0], rref[1], rref[2]
    state["sa_dacc_p"],   state["sa_dacc_q"],    state["sa_dacc_r"]   = dacc[0], dacc[1], dacc[2]
    state["sa_aref_p"],   state["sa_aref_q"],    state["sa_aref_r"]   = aref[0], aref[1], aref[2]

def on_rotorcraft_cmd(agent, msg):
    parts = msg.split()
    if len(parts) < 6:
        return
    state["rc_roll"], state["rc_pitch"] = int(float(parts[2])), int(float(parts[3]))
    state["rc_yaw"],  state["rc_thrust"] = int(float(parts[4])), int(float(parts[5]))


# ── simsitl stdout reader ────────────────────────────────────────────────────
def sim_stdout_reader(proc):
    """Read lines from simsitl stdout into debug_log (daemon thread)."""
    with open(DEBUG_LOG_FILE, "w", buffering=1) as f:
        for line in proc.stdout:
            stripped = line.rstrip()
            debug_log.append(stripped)
            f.write(stripped + "\n")


# ── display ──────────────────────────────────────────────────────────────────
def bar(val, lo, hi, width=18, unit=""):
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
BLUE   = "\033[34m"

def render():
    s = state
    age = time.monotonic() - s["t"]
    stale = age > 1.0
    status   = f"{GREY}stale ({age:.1f}s){RESET}" if stale else f"{GREEN}live{RESET}"
    cmd_line = f"  {YELLOW}CMD:{RESET} {s['cmd']}" if s["cmd"] else f"  {GREY}no command sent yet{RESET}"

    lines = [
        f"{BOLD}{'─' * 58}{RESET}",
        f"  {BOLD}{CYAN}ANTON NPS  —  {AC_NAME} (ac_id {AC_ID}){RESET}   {status}",
        cmd_line,
        f"{'─' * 58}",
        "",
        f"  {BOLD}LAYER 0 — JSBSim truth{RESET}",
        f"    Lat {s['lat']:+12.6f}°   Lon {s['lon']:+12.6f}°",
        f"    Alt {s['alt']:+10.2f} m MSL   AGL {s['agl']:+8.2f} m",
        f"    Roll  {bar(s['phi'],   -45, 45, unit='°')}",
        f"    Pitch {bar(s['theta'], -45, 45, unit='°')}",
        f"    Yaw   {s['psi']:+8.2f}°",
        f"    p {bar(s['p'], -60, 60, unit='°/s')}",
        f"    q {bar(s['q'], -60, 60, unit='°/s')}",
        f"    r {bar(s['r'], -60, 60, unit='°/s')}",
        f"    Vn {s['vx']:+7.3f} m/s  Ve {s['vy']:+7.3f} m/s  Vd {s['vz']:+7.3f} m/s",
        f"    Wind N {s['wind_n']:+6.2f}  E {s['wind_e']:+6.2f}  D {s['wind_d']:+6.2f} m/s",
        f"    Gyro bias p {s['bias_p']:+6.3f}  q {s['bias_q']:+6.3f}  r {s['bias_r']:+6.3f} °/s",
        "",
        f"  {BOLD}{BLUE}LAYER 1 — INDI stabilizer  (STAB_ATTITUDE){RESET}",
        f"    {'':6s}  {'att(°)':>9s}  {'ref(°)':>9s}  {'Δatt(°)':>9s}  {'rate(°/s)':>9s}  {'rref(°/s)':>9s}  {'acc_ref':>9s}",
        f"    {'roll':6s}  {math.degrees(s['sa_att_phi']):+9.3f}  {math.degrees(s['sa_ref_phi']):+9.3f}  {math.degrees(s['sa_att_phi']-s['sa_ref_phi']):+9.3f}  {math.degrees(s['sa_rate_p']):+9.3f}  {math.degrees(s['sa_rref_p']):+9.3f}  {math.degrees(s['sa_aref_p']):+9.3f}",
        f"    {'pitch':6s}  {math.degrees(s['sa_att_theta']):+9.3f}  {math.degrees(s['sa_ref_theta']):+9.3f}  {math.degrees(s['sa_att_theta']-s['sa_ref_theta']):+9.3f}  {math.degrees(s['sa_rate_q']):+9.3f}  {math.degrees(s['sa_rref_q']):+9.3f}  {math.degrees(s['sa_aref_q']):+9.3f}",
        f"    {'yaw':6s}  {math.degrees(s['sa_att_psi']):+9.3f}  {math.degrees(s['sa_ref_psi']):+9.3f}  {math.degrees(s['sa_att_psi']-s['sa_ref_psi']):+9.3f}  {math.degrees(s['sa_rate_r']):+9.3f}  {math.degrees(s['sa_rref_r']):+9.3f}  {math.degrees(s['sa_aref_r']):+9.3f}",
        f"    WLS v (cmd):   φ {s['wls_v0']:+8.2f}  θ {s['wls_v1']:+8.2f}  ψ {s['wls_v2']:+8.2f}  T {s['wls_v3']:+8.2f}",
        "",
        f"  {BOLD}{BLUE}LAYER 1 — MFC controller  (STAB_MFC){RESET}",
        f"    {'':6s}  {'sp(°)':>10s}  {'meas(°)':>10s}  {'err(°)':>10s}  {'F_k':>12s}  {'cmd':>10s}",
        f"    {'roll':6s}  {math.degrees(s['mfc_sp_phi']):+10.3f}  {math.degrees(s['mfc_me_phi']):+10.3f}  {math.degrees(s['mfc_err_phi']):+10.4f}  {s['mfc_fk_phi']:+12.4f}  {s['mfc_cmd_phi']:+10.4f}",
        f"    {'pitch':6s}  {math.degrees(s['mfc_sp_theta']):+10.3f}  {math.degrees(s['mfc_me_theta']):+10.3f}  {math.degrees(s['mfc_err_theta']):+10.4f}  {s['mfc_fk_theta']:+12.4f}  {s['mfc_cmd_theta']:+10.4f}",
        f"    {'yaw':6s}  {math.degrees(s['mfc_sp_psi']):+10.3f}  {math.degrees(s['mfc_me_psi']):+10.3f}  {math.degrees(s['mfc_err_psi']):+10.4f}  {s['mfc_fk_psi']:+12.4f}  {s['mfc_cmd_psi']:+10.4f}",
        f"    WLS v (cmd):   φ {s['wls_v0']:+8.2f}  θ {s['wls_v1']:+8.2f}  ψ {s['wls_v2']:+8.2f}  T {s['wls_v3']:+8.2f}",
        f"    WLS u (pprz):  NE {s['mfc_u0']:+7.0f}  SE {s['mfc_u1']:+7.0f}  SW {s['mfc_u2']:+7.0f}  NW {s['mfc_u3']:+7.0f}",
        "",
        f"  {BOLD}DEBUG (printf from firmware){RESET}",
        *[f"    {GREY}{line}{RESET}" for line in list(debug_log)[-8:]],
        f"{'─' * 58}",
        f"  {GREY}Log → {LOG_FILE}   Debug → {DEBUG_LOG_FILE}   Ctrl-C to stop{RESET}",
    ]
    sys.stdout.write(CLEAR + "\n".join(lines) + "\n")
    sys.stdout.flush()


# ── CSV logger ───────────────────────────────────────────────────────────────
def log_writer():
    with open(LOG_FILE, "w", newline="") as f:
        writer = None
        while True:
            row = {**state, "wall": time.time()}
            if writer is None:
                writer = csv.DictWriter(f, fieldnames=list(row.keys()))
                writer.writeheader()
            writer.writerow(row)
            f.flush()
            time.sleep(0.1)


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

    print("Starting Paparazzi link (UDP 4242) …")
    link = subprocess.Popen(
        [LINK, "-b", IVY_BUS, "-udp"],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )

    print("Starting ANTON NPS sim …")
    _sim_cmd = [SIMSITL, "--norc"]
    if _GDB:
        _sim_cmd = ["qemu-x86_64", "-g", "1234"] + _sim_cmd
    print("Starting ANTON NPS sim …" + (" (QEMU gdbstub :1234, waiting for debugger)" if _GDB else ""))
    sim = subprocess.Popen(
        _sim_cmd,
        env=env, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
        text=True, bufsize=1,
    )

    # UDP socket for sending binary pprz commands to the sim's primary datalink
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def shutdown(sig=None, frame=None):
        print("\n\nShutting down …")
        sim.terminate()
        link.terminate()
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

    # Layer 0 — JSBSim truth (direct NPS Ivy messages)
    IvyBindMsg(on_rate_attitude, r"(\d+ NPS_RATE_ATTITUDE .*)")
    IvyBindMsg(on_pos_llh,       r"(\d+ NPS_POS_LLH .*)")
    IvyBindMsg(on_speed_pos,     r"(\d+ NPS_SPEED_POS .*)")
    IvyBindMsg(on_sensors,       r"(\d+ NPS_SENSORS_SCALED .*)")
    IvyBindMsg(on_gyro_bias,     r"(\d+ NPS_GYRO_BIAS .*)")
    IvyBindMsg(on_wind,          r"(\d+ NPS_WIND .*)")

    # Layer 1 — Firmware PPRZ telemetry (bridged from UDP by pprz_server)
    IvyBindMsg(on_stab_attitude,  r"(\d+ STAB_ATTITUDE .*)")
    IvyBindMsg(on_stab_mfc,       r"(\d+ STAB_MFC .*)")
    IvyBindMsg(on_wls_v_stab,     r"(\d+ WLS_V .*)")
    IvyBindMsg(on_rotorcraft_cmd, r"(\d+ ROTORCRAFT_CMD .*)")

    state["t"] = time.monotonic()
    threading.Thread(target=sim_stdout_reader, args=(sim,), daemon=True).start()
    threading.Thread(target=takeoff_sequence, args=(sock,), daemon=True).start()
    threading.Thread(target=log_writer, daemon=True).start()
    print(f"Logging to {LOG_FILE}")
    print(f"Debug log → {DEBUG_LOG_FILE}")

    while True:
        render()
        time.sleep(0.1)


if __name__ == "__main__":
    main()
