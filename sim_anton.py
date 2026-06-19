#!/usr/bin/env python3
"""
Launch a Paparazzi NPS simulation and print live aircraft state to the terminal.
Sends a takeoff command sequence 1 second after the sim is ready.

Usage:  python3 sim_anton.py AIRCRAFT [--gdb] [--fg] [--render] [--switch-after SEC]
        Ctrl-C to stop (kills child processes cleanly).

  AIRCRAFT  Aircraft name as registered in the conf XML (e.g. ANTON_MFC).
            The CONF env var selects which conf file to search
            (default: conf/airframes/ENAC/conf_enac.xml, relative to PAPARAZZI_HOME).

Flags:
  --render          Enable the live TUI dashboard (default: plain debug log to stdout).
  --fg              Stream NET_FDM pose to FlightGear on the Mac host (host.docker.internal:5501).
  --switch-after N  Switch to MFC N seconds after takeoff, then back to INDI N seconds later.

Manual controller switching (run in a second terminal while the sim is running):
  python3 pprz_ctrl.py AIRCRAFT switch mfc
  python3 pprz_ctrl.py AIRCRAFT switch indi
  python3 pprz_ctrl.py AIRCRAFT block 5     # jump to a flight plan block

Takeoff sequence (flight plan blocks):
  Block 3 "Start Engine" → NavResurrect() un-kills throttle
  Block 4 "Takeoff"      → climbs to CLIMB waypoint at nav.climb_vspeed

Command delivery:
  Commands (block jumps, settings changes) are sent as pprz binary frames over UDP to
  port 4243 (the sim's primary datalink input — DOWNLINK_DEVICE=udp0, UDP0_PORT_IN=4243).

Observability:
  Layer 0 — JSBSim truth:   NPS_RATE_ATTITUDE, NPS_POS_LLH, NPS_SPEED_POS
  Layer 1 — Firmware state: STAB_ATTITUDE (INDI), STAB_MFC, ROTORCRAFT_CMD, DUAL_CTRL
  Logging: CSV written to /tmp/mfc_sim_<timestamp>.csv
  Scope:   The in-process NPS emitter (nps_scope.c in simsitl) still streams to PlotJuggler
           if it was compiled in — use --scope_host/--scope_port/--scope_decim flags
           directly on simsitl if you need that stream.
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
import xml.etree.ElementTree as ET
from collections import deque
from ivy.std_api import IvyInit, IvyStart, IvyStop, IvyBindMsg

PPRZ    = "/workspace/paparazzi"
SERVER  = f"{PPRZ}/sw/ground_segment/tmtc/server"
LINK    = f"{PPRZ}/sw/ground_segment/tmtc/link"
IVY_BUS = "127.255.255.255:2010"

# ── Parse positional AC_NAME + flags ─────────────────────────────────────────
_flags = {}
AC_NAME = None
_args = sys.argv[1:]
i = 0
while i < len(_args):
    a = _args[i]
    if a == "--switch-after" and i + 1 < len(_args):
        _flags["switch_after"] = float(_args[i + 1])
        i += 2
    elif a.startswith("-"):
        _flags[a] = True
        i += 1
    elif AC_NAME is None:
        AC_NAME = a
        i += 1
    else:
        i += 1

if AC_NAME is None:
    print(f"Usage: {sys.argv[0]} AIRCRAFT [--gdb] [--fg] [--render] [--switch-after SEC]",
          file=sys.stderr)
    sys.exit(1)

_USE_RENDER   = "--render"   in _flags
_GDB          = "--gdb"      in _flags
_USE_FG       = "--fg"       in _flags
_USE_SCOPE    = "--no-scope" not in _flags
_SWITCH_AFTER = _flags.get("switch_after")   # seconds after takeoff, or None

# ── Look up AC_ID from the conf XML ──────────────────────────────────────────
def _lookup_ac_id(pprz_home: str, conf_rel: str, name: str) -> int | None:
    conf_path = os.path.join(pprz_home, conf_rel)
    try:
        tree = ET.parse(conf_path)
        for ac in tree.getroot().findall("aircraft"):
            if ac.get("name") == name:
                return int(ac.get("ac_id"))
    except Exception as e:
        print(f"Warning: could not parse {conf_path}: {e}", file=sys.stderr)
    return None

_CONF = os.environ.get("CONF") or "conf/airframes/ENAC/conf_enac.xml"
AC_ID = _lookup_ac_id(PPRZ, _CONF, AC_NAME)
if AC_ID is None:
    print(f"Error: aircraft '{AC_NAME}' not found in {os.path.join(PPRZ, _CONF)}", file=sys.stderr)
    sys.exit(1)

SIMSITL  = f"{PPRZ}/var/aircrafts/{AC_NAME}/nps/simsitl"
FG_PORT  = 5501
try:
    FG_HOST = socket.gethostbyname("host.docker.internal")
except OSError:
    FG_HOST = "192.168.65.254"
R2D      = math.degrees(1)
SIM_HOST = "127.0.0.1"
SIM_PORT = 4243   # sim's primary datalink receive port (UDP0_PORT_IN)

# Scope: in-process NPS emitter → PlotJuggler on the Mac host (sim-time stamped JSON).
# PlotJuggler: Streaming → Start → UDP Server, port 9870, protocol JSON, "use field as timestamp" = t.
SCOPE_HOST  = FG_HOST   # same egress path as FlightGear
SCOPE_PORT  = 9870
SCOPE_DECIM = 2         # emit every Nth sim step (~500 Hz at 1 kHz sim rate)

LOG_FILE       = f"/tmp/mfc_sim_{datetime.datetime.now():%Y%m%d_%H%M%S}.csv"
DEBUG_LOG_FILE = f"/tmp/mfc_sim_{datetime.datetime.now():%Y%m%d_%H%M%S}_debug.log"

# ── pprz binary encoding ─────────────────────────────────────────────────────
STX            = 0x99
CLASS_DATALINK = 2
MSG_BLOCK      = 5   # datalink::BLOCK   — fields: block_id(u8), ac_id(u8)
MSG_SETTING    = 4   # datalink::SETTING — fields: index(u8), ac_id(u8), value(f32)

# Dual-controller constants (match control_dual_mfc_indi.h)
DUAL_CTRL_INDI = 0
DUAL_CTRL_MFC  = 1
DUAL_CTRL_IDX  = 47  # settings.h flat index for dual_ctrl_active on ANTON_MFC
                     # Verify: grep -n "dual_ctrl_active" var/aircrafts/ANTON_MFC/ap/generated/settings.h

def _pprz_frame(msg_id: int, payload: bytes) -> bytes:
    sender   = 0
    receiver = AC_ID
    comp_cls = (0 << 4) | CLASS_DATALINK
    length   = 8 + len(payload)
    header   = struct.pack("BBBBBB", STX, length, sender, receiver, comp_cls, msg_id) + payload
    ck_a = ck_b = 0
    for b in header[1:]:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return header + struct.pack("BB", ck_a, ck_b)

def pprz_block_frame(block_id: int) -> bytes:
    return _pprz_frame(MSG_BLOCK, bytes([block_id, AC_ID]))

def pprz_setting_frame(index: int, value: float) -> bytes:
    return _pprz_frame(MSG_SETTING, struct.pack("BBf", index, AC_ID, value))


# ── shared state ─────────────────────────────────────────────────────────────
state = {
    # Layer 0 — JSBSim truth (NPS_*)
    "phi": 0.0, "theta": 0.0, "psi": 0.0,
    "p":   0.0, "q":     0.0, "r":   0.0,
    "lat": 0.0, "lon":   0.0,
    "alt": 0.0, "agl":   0.0,
    "vx":  0.0, "vy":    0.0, "vz":  0.0,
    "ax":  0.0, "ay":    0.0, "az":  0.0,
    "bias_p": 0.0, "bias_q": 0.0, "bias_r": 0.0,
    "wind_n": 0.0, "wind_e": 0.0, "wind_d": 0.0,
    # Layer 1 — MFC stabilizer (STAB_MFC)
    "mfc_sp_phi":   0.0, "mfc_sp_theta":   0.0, "mfc_sp_psi":   0.0,
    "mfc_me_phi":   0.0, "mfc_me_theta":   0.0, "mfc_me_psi":   0.0,
    "mfc_err_phi":  0.0, "mfc_err_theta":  0.0, "mfc_err_psi":  0.0,
    "mfc_fk_phi":   0.0, "mfc_fk_theta":   0.0, "mfc_fk_psi":   0.0,
    "mfc_cmd_phi":  0.0, "mfc_cmd_theta":  0.0, "mfc_cmd_psi":  0.0,
    "mfc_u0": 0.0, "mfc_u1": 0.0, "mfc_u2": 0.0, "mfc_u3": 0.0,
    # Layer 1 — WLS virtual control inputs
    "wls_v0": 0.0, "wls_v1": 0.0, "wls_v2": 0.0, "wls_v3": 0.0,
    # Layer 1 — INDI stabilizer (STAB_ATTITUDE)
    "sa_att_phi":  0.0, "sa_att_theta":  0.0, "sa_att_psi":  0.0,
    "sa_ref_phi":  0.0, "sa_ref_theta":  0.0, "sa_ref_psi":  0.0,
    "sa_rate_p":   0.0, "sa_rate_q":     0.0, "sa_rate_r":   0.0,
    "sa_rref_p":   0.0, "sa_rref_q":     0.0, "sa_rref_r":   0.0,
    "sa_dacc_p":   0.0, "sa_dacc_q":     0.0, "sa_dacc_r":   0.0,
    "sa_aref_p":   0.0, "sa_aref_q":     0.0, "sa_aref_r":   0.0,
    # Layer 1 — Dual-controller status (DUAL_CTRL)
    "dual_active": -1,  # -1 = not yet received; 0 = INDI, 1 = MFC
    # Layer 1 — Firmware cmd ints (ROTORCRAFT_CMD)
    "rc_roll": 0, "rc_pitch": 0, "rc_yaw": 0, "rc_thrust": 0,
    "t":   0.0,
    "cmd": "",
}

debug_log: deque = deque(maxlen=20)

# ── Ivy callbacks ─────────────────────────────────────────────────────────────
def on_rate_attitude(agent, msg):
    parts = msg.split()
    if len(parts) < 8: return
    state["p"], state["q"], state["r"] = float(parts[2]), float(parts[3]), float(parts[4])
    state["phi"], state["theta"], state["psi"] = float(parts[5]), float(parts[6]), float(parts[7])
    state["t"] = time.monotonic()

def on_pos_llh(agent, msg):
    parts = msg.split()
    if len(parts) < 10: return
    state["lat"] = float(parts[2]) * R2D
    state["lon"] = float(parts[5]) * R2D
    state["alt"] = float(parts[7])
    state["agl"] = float(parts[9])

def on_speed_pos(agent, msg):
    parts = msg.split()
    if len(parts) < 8: return
    state["vx"], state["vy"], state["vz"] = float(parts[5]), float(parts[6]), float(parts[7])

def on_sensors(agent, msg):
    parts = msg.split()
    if len(parts) < 5: return
    state["ax"], state["ay"], state["az"] = float(parts[2]), float(parts[3]), float(parts[4])

def on_gyro_bias(agent, msg):
    parts = msg.split()
    if len(parts) < 5: return
    state["bias_p"], state["bias_q"], state["bias_r"] = float(parts[2]), float(parts[3]), float(parts[4])

def on_wind(agent, msg):
    parts = msg.split()
    if len(parts) < 5: return
    state["wind_n"], state["wind_e"], state["wind_d"] = float(parts[2]), float(parts[3]), float(parts[4])

def on_stab_mfc(agent, msg):
    parts = msg.split()
    if len(parts) < 21: return
    i = 2
    state["mfc_sp_phi"],  state["mfc_sp_theta"],  state["mfc_sp_psi"]  = float(parts[i]),    float(parts[i+1]),  float(parts[i+2])
    state["mfc_me_phi"],  state["mfc_me_theta"],  state["mfc_me_psi"]  = float(parts[i+3]),  float(parts[i+4]),  float(parts[i+5])
    state["mfc_err_phi"], state["mfc_err_theta"], state["mfc_err_psi"] = float(parts[i+6]),  float(parts[i+7]),  float(parts[i+8])
    state["mfc_fk_phi"],  state["mfc_fk_theta"],  state["mfc_fk_psi"]  = float(parts[i+9]),  float(parts[i+10]), float(parts[i+11])
    state["mfc_cmd_phi"], state["mfc_cmd_theta"], state["mfc_cmd_psi"] = float(parts[i+12]), float(parts[i+13]), float(parts[i+14])
    state["mfc_u0"], state["mfc_u1"] = float(parts[i+15]), float(parts[i+16])
    state["mfc_u2"], state["mfc_u3"] = float(parts[i+17]), float(parts[i+18])

def on_wls_v_stab(agent, msg):
    parts = msg.split()
    if len(parts) < 6: return
    v = [float(x) for x in parts[5].split(',')]
    if len(v) >= 4:
        state["wls_v0"], state["wls_v1"] = v[0], v[1]
        state["wls_v2"], state["wls_v3"] = v[2], v[3]

def on_stab_attitude(agent, msg):
    parts = msg.split()
    if len(parts) < 9: return
    att  = [float(x) for x in parts[3].split(',')]
    ref  = [float(x) for x in parts[4].split(',')]
    rate = [float(x) for x in parts[5].split(',')]
    rref = [float(x) for x in parts[6].split(',')]
    dacc = [float(x) for x in parts[7].split(',')]
    aref = [float(x) for x in parts[8].split(',')]
    state["sa_att_phi"],  state["sa_att_theta"], state["sa_att_psi"]  = att[0],  att[1],  att[2]
    state["sa_ref_phi"],  state["sa_ref_theta"], state["sa_ref_psi"]  = ref[0],  ref[1],  ref[2]
    state["sa_rate_p"],   state["sa_rate_q"],    state["sa_rate_r"]   = rate[0], rate[1], rate[2]
    state["sa_rref_p"],   state["sa_rref_q"],    state["sa_rref_r"]   = rref[0], rref[1], rref[2]
    state["sa_dacc_p"],   state["sa_dacc_q"],    state["sa_dacc_r"]   = dacc[0], dacc[1], dacc[2]
    state["sa_aref_p"],   state["sa_aref_q"],    state["sa_aref_r"]   = aref[0], aref[1], aref[2]

def on_dual_ctrl(agent, msg):
    # DUAL_CTRL: AC_ID DUAL_CTRL active committed[nb] shadow[nb] resid[nb]
    parts = msg.split()
    if len(parts) < 3: return
    try:
        state["dual_active"] = int(parts[2])
    except ValueError:
        pass

def on_rotorcraft_cmd(agent, msg):
    parts = msg.split()
    if len(parts) < 6: return
    state["rc_roll"],  state["rc_pitch"] = int(float(parts[2])), int(float(parts[3]))
    state["rc_yaw"],   state["rc_thrust"] = int(float(parts[4])), int(float(parts[5]))


# ── simsitl stdout reader ─────────────────────────────────────────────────────
def sim_stdout_reader(proc):
    with open(DEBUG_LOG_FILE, "w", buffering=1) as f:
        for line in proc.stdout:
            stripped = line.rstrip()
            debug_log.append(stripped)
            f.write(stripped + "\n")
            if not _USE_RENDER:
                print(stripped, flush=True)


# ── display ───────────────────────────────────────────────────────────────────
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
RED    = "\033[31m"

def render():
    s = state
    age = time.monotonic() - s["t"]
    stale = age > 1.0
    status = f"{GREY}stale ({age:.1f}s){RESET}" if stale else f"{GREEN}live{RESET}"
    cmd_line = f"  {YELLOW}CMD:{RESET} {s['cmd']}" if s["cmd"] else f"  {GREY}no command sent yet{RESET}"

    da = s["dual_active"]
    if da < 0:
        law_str = f"{GREY}(waiting){RESET}"
    elif da == DUAL_CTRL_MFC:
        law_str = f"{RED}{BOLD}MFC{RESET}"
    else:
        law_str = f"{GREEN}INDI{RESET}"

    lines = [
        f"{BOLD}{'─' * 60}{RESET}",
        f"  {BOLD}{CYAN}ANTON NPS  —  {AC_NAME} (ac_id {AC_ID}){RESET}   {status}",
        f"  Active law: {law_str}    {cmd_line.strip()}",
        f"{'─' * 60}",
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
        "",
        f"  {BOLD}{BLUE}LAYER 1 — INDI stabilizer  (STAB_ATTITUDE){RESET}",
        f"    {'':6s}  {'att(°)':>9s}  {'ref(°)':>9s}  {'rate(°/s)':>9s}  {'acc_ref':>9s}",
        f"    {'roll':6s}  {math.degrees(s['sa_att_phi']):+9.3f}  {math.degrees(s['sa_ref_phi']):+9.3f}  {math.degrees(s['sa_rate_p']):+9.3f}  {math.degrees(s['sa_aref_p']):+9.3f}",
        f"    {'pitch':6s}  {math.degrees(s['sa_att_theta']):+9.3f}  {math.degrees(s['sa_ref_theta']):+9.3f}  {math.degrees(s['sa_rate_q']):+9.3f}  {math.degrees(s['sa_aref_q']):+9.3f}",
        f"    {'yaw':6s}  {math.degrees(s['sa_att_psi']):+9.3f}  {math.degrees(s['sa_ref_psi']):+9.3f}  {math.degrees(s['sa_rate_r']):+9.3f}  {math.degrees(s['sa_aref_r']):+9.3f}",
        "",
        f"  {BOLD}{BLUE}LAYER 1 — MFC controller  (STAB_MFC){RESET}",
        f"    {'':6s}  {'sp(°)':>10s}  {'meas(°)':>10s}  {'err(°)':>10s}  {'F_k':>12s}  {'cmd':>10s}",
        f"    {'roll':6s}  {math.degrees(s['mfc_sp_phi']):+10.3f}  {math.degrees(s['mfc_me_phi']):+10.3f}  {math.degrees(s['mfc_err_phi']):+10.4f}  {s['mfc_fk_phi']:+12.4f}  {s['mfc_cmd_phi']:+10.4f}",
        f"    {'pitch':6s}  {math.degrees(s['mfc_sp_theta']):+10.3f}  {math.degrees(s['mfc_me_theta']):+10.3f}  {math.degrees(s['mfc_err_theta']):+10.4f}  {s['mfc_fk_theta']:+12.4f}  {s['mfc_cmd_theta']:+10.4f}",
        f"    {'yaw':6s}  {math.degrees(s['mfc_sp_psi']):+10.3f}  {math.degrees(s['mfc_me_psi']):+10.3f}  {math.degrees(s['mfc_err_psi']):+10.4f}  {s['mfc_fk_psi']:+12.4f}  {s['mfc_cmd_psi']:+10.4f}",
        f"    WLS v:  φ {s['wls_v0']:+8.2f}  θ {s['wls_v1']:+8.2f}  ψ {s['wls_v2']:+8.2f}  T {s['wls_v3']:+8.2f}",
        f"    WLS u:  NE {s['mfc_u0']:+7.0f}  SE {s['mfc_u1']:+7.0f}  SW {s['mfc_u2']:+7.0f}  NW {s['mfc_u3']:+7.0f}",
        "",
        f"  {BOLD}DEBUG{RESET}",
        *[f"    {GREY}{line}{RESET}" for line in list(debug_log)[-6:]],
        f"{'─' * 60}",
        f"  {GREY}Log → {LOG_FILE}   Ctrl-C to stop{RESET}",
    ]
    sys.stdout.write(CLEAR + "\n".join(lines) + "\n")
    sys.stdout.flush()


# ── CSV logger ────────────────────────────────────────────────────────────────
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


# ── command senders ───────────────────────────────────────────────────────────
def send_block(sock: socket.socket, block_id: int, label: str):
    frame = pprz_block_frame(block_id)
    sock.sendto(frame, (SIM_HOST, SIM_PORT))
    state["cmd"] = f"BLOCK {block_id} ({label})"

def send_switch(sock: socket.socket, law: int, label: str):
    frame = pprz_setting_frame(DUAL_CTRL_IDX, float(law))
    sock.sendto(frame, (SIM_HOST, SIM_PORT))
    state["cmd"] = f"SWITCH → {label}"
    print(f"[ctrl] switching to {label}", flush=True)

def takeoff_sequence(sock: socket.socket):
    time.sleep(1.0)
    send_block(sock, 3, "Start Engine")
    time.sleep(0.5)
    send_block(sock, 4, "Takeoff")

    if _SWITCH_AFTER is not None:
        time.sleep(_SWITCH_AFTER)
        send_switch(sock, DUAL_CTRL_MFC, "MFC")
        time.sleep(_SWITCH_AFTER)
        send_switch(sock, DUAL_CTRL_INDI, "INDI")


# ── main ──────────────────────────────────────────────────────────────────────
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

    _sim_cmd = [SIMSITL, "--norc"]
    if _USE_FG:
        _sim_cmd += ["--fg_host", FG_HOST, "--fg_port", str(FG_PORT), "--fg_fdm"]
    if _USE_SCOPE:
        _sim_cmd += ["--scope_host", SCOPE_HOST,
                     "--scope_port", str(SCOPE_PORT),
                     "--scope_decim", str(SCOPE_DECIM)]
    if _GDB:
        _sim_cmd = ["gdbserver", ":1234"] + _sim_cmd
    print("Starting NPS sim …" + (" (gdbserver :1234, waiting for debugger)" if _GDB else ""))
    sim = subprocess.Popen(
        _sim_cmd,
        env=env, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
        text=True, bufsize=1,
    )

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def shutdown(sig=None, frame=None):
        print("\nShutting down …")
        sim.terminate(); link.terminate(); server.terminate(); sock.close()
        try: IvyStop()
        except Exception: pass
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    print("Waiting for sim to start …")
    time.sleep(3)

    IvyInit("anton_monitor", "READY", None, lambda a, b: None, lambda a, b: None)
    IvyStart(IVY_BUS)

    IvyBindMsg(on_rate_attitude,  r"(\d+ NPS_RATE_ATTITUDE .*)")
    IvyBindMsg(on_pos_llh,        r"(\d+ NPS_POS_LLH .*)")
    IvyBindMsg(on_speed_pos,      r"(\d+ NPS_SPEED_POS .*)")
    IvyBindMsg(on_sensors,        r"(\d+ NPS_SENSORS_SCALED .*)")
    IvyBindMsg(on_gyro_bias,      r"(\d+ NPS_GYRO_BIAS .*)")
    IvyBindMsg(on_wind,           r"(\d+ NPS_WIND .*)")
    IvyBindMsg(on_stab_attitude,  r"(\d+ STAB_ATTITUDE .*)")
    IvyBindMsg(on_stab_mfc,       r"(\d+ STAB_MFC .*)")
    IvyBindMsg(on_wls_v_stab,     r"(\d+ WLS_V .*)")
    IvyBindMsg(on_dual_ctrl,      r"(\d+ DUAL_CTRL .*)")
    IvyBindMsg(on_rotorcraft_cmd, r"(\d+ ROTORCRAFT_CMD .*)")

    state["t"] = time.monotonic()
    threading.Thread(target=sim_stdout_reader, args=(sim,), daemon=True).start()
    threading.Thread(target=takeoff_sequence,  args=(sock,), daemon=True).start()
    threading.Thread(target=log_writer,        daemon=True).start()
    print(f"Logging → {LOG_FILE}   Debug → {DEBUG_LOG_FILE}")
    if _USE_SCOPE:
        print(f"Scope → PlotJuggler at {SCOPE_HOST}:{SCOPE_PORT} (decim {SCOPE_DECIM}, ~{1000//SCOPE_DECIM} Hz)")
    if _SWITCH_AFTER:
        print(f"Auto-switch: MFC at +{_SWITCH_AFTER}s, INDI at +{2*_SWITCH_AFTER}s")

    if _USE_RENDER:
        while True:
            render()
            time.sleep(0.1)
    else:
        while True:
            time.sleep(1)


if __name__ == "__main__":
    main()
