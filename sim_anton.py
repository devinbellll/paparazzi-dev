#!/usr/bin/env python3
"""
Launch a Paparazzi NPS simulation and print live aircraft state to the terminal.
Sends a takeoff command sequence 1 second after the sim is ready.

Usage:  python3 sim_anton.py AIRCRAFT [--gdb] [--fg] [--render] [--rc_script N]
        Ctrl-C to stop (kills child processes cleanly).

  AIRCRAFT  Aircraft name as registered in the conf XML (e.g. ANTON_MFC).
            The CONF env var selects which conf file to search
            (default: conf/userconf/ENAC/conf_mfc.xml, relative to PAPARAZZI_HOME).

Flags:
  --render          Enable the live TUI dashboard (default: plain debug log to stdout).
  --fg              Stream NET_FDM pose to FlightGear on the Mac host (host.docker.internal:5501).
  --rc_script N     Drive RC from a compiled-in NPS stick script instead of --norc
                    (0=hover 1=step_roll 2=step_pitch 3=step_yaw 4=ff; auto-takeoff for first 8 s).
                    Script 5 = FP takeoff (NAV) then ATTITUDE_Z_HOLD with a scheduled
                    roll/pitch/yaw step sequence (ANTON_MFC: AUTO1=ATTITUDE_Z_HOLD).

Interactive commands (type in the sim terminal, or pipe via pprz_ctrl.py):
  block <id>              jump to flight plan block
  setting <name> <float>  set a GCS setting by its shortname (var/aircrafts/<AC>/settings.xml)

  From a second terminal on the host:
    python3 pprz_ctrl.py AIRCRAFT block 5

Takeoff sequence (flight plan blocks):
  Block 2 "Start Engine" → NavResurrect() un-kills throttle
  Block 3 "Takeoff"      → climbs to CLIMB waypoint at nav.climb_vspeed

Command delivery:
  Commands are sent as native Ivy messages (ground/JUMP_TO_BLOCK, ground/DL_SETTING) — the
  same mechanism the GCS strip buttons and settings panel use. `server` decodes them into the
  binary pprz frames and forwards them over the sim's datalink; we never touch that wire format.

Observability:
  Layer 0 — JSBSim truth:   NPS_RATE_ATTITUDE, NPS_POS_LLH
  Layer 1 — Firmware state: ROTORCRAFT_CMD (motor commands)

  PlotJuggler feed — exactly ONE source streams to the Mac, in the shared
  aircraft-agnostic schema (root "uav", MFC_* branches — see pj_json_relay.py):
    default:     the in-process NPS scope emitter (nps_scope.c in simsitl) — ground
                 truth + firmware-registered vars at full decimated sim rate. It sends
                 to a local port; we normalize each packet and forward it on.
    --no-scope:  fall back to server's ivy UDP/JSON telemetry stream (downsampled,
                 walltime-stamped — what you'd have in real flight).
  Because both sources are normalized to the same schema, plotjuggler_mfc.xml /
  plotjuggler_indi.xml work unchanged for sim and real flight, any aircraft.
  PJ_HOST / PJ_PORT env vars override the PlotJuggler destination (default: the
  Mac host, 9870).

  Logging: server's UDP/JSON telemetry stream is always captured (normalized) to a
           .jsonl file — same schema PlotJuggler consumes, no bespoke CSV columns
           to keep in sync with the firmware.

Note on `pprzsim-launch`: paparazzi ships sw/simulator/pprzsim-launch as the canonical NPS
launcher, but it only knows how to `execv` simsitl with a handful of flags (fg/rc_script/norc/
ivy_bus) — it does not support the in-process scope emitter or a gdbserver wrap, both of which
this script needs by default. So simsitl is still invoked directly here; that is simsitl's own
native CLI, not something reinvented by this script.
"""

import datetime
import json
import math
import os
import signal
import socket
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET
from collections import deque

PPRZ = "/workspace/paparazzi"
SERVER = f"{PPRZ}/sw/ground_segment/tmtc/server"
LINK = f"{PPRZ}/sw/ground_segment/tmtc/link"
IVY_BUS = "127.255.255.255:2010"

sys.path.append(f"{PPRZ}/sw/lib/python")
sys.path.append(f"{PPRZ}/var/lib/python")
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage
from settings import PprzSettingsParser
from pj_json_relay import sanitize as sanitize_json

# ── Parse positional AC_NAME + flags ─────────────────────────────────────────
_flags = {}
AC_NAME = None
_args = sys.argv[1:]
i = 0
while i < len(_args):
    a = _args[i]
    if a == "--rc_script" and i + 1 < len(_args):
        _flags["rc_script"] = int(_args[i + 1])
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
    print(f"Usage: {sys.argv[0]} AIRCRAFT [--gdb] [--fg] [--render] [--rc_script N]",
          file=sys.stderr)
    sys.exit(1)

_USE_RENDER   = "--render"   in _flags
_GDB          = "--gdb"      in _flags
_USE_FG       = "--fg"       in _flags
_USE_SCOPE    = "--no-scope" not in _flags
_RC_SCRIPT    = _flags.get("rc_script")      # NPS compiled RC script index, or None

# ── Look up AC_ID from the conf XML ──────────────────────────────────────────
# Still needed: Ivy ground messages (JUMP_TO_BLOCK / DL_SETTING) address the
# aircraft by its numeric ac_id, which `server` uses as its aircraft-table key.
# (Launching simsitl itself never needed this — it's addressed by AC_NAME.)
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

_CONF = os.environ.get("CONF") or "conf/userconf/ENAC/conf_mfc.xml"
AC_ID = _lookup_ac_id(PPRZ, _CONF, AC_NAME)
if AC_ID is None:
    print(f"Error: aircraft '{AC_NAME}' not found in {os.path.join(PPRZ, _CONF)}", file=sys.stderr)
    sys.exit(1)

SIMSITL = f"{PPRZ}/var/aircrafts/{AC_NAME}/nps/simsitl"
FG_PORT = 5501
try:
    FG_HOST = socket.gethostbyname("host.docker.internal")
except OSError:
    FG_HOST = "192.168.65.254"
R2D = math.degrees(1)

# PlotJuggler destination (Streaming → UDP Server, protocol JSON, timestamp field
# "timestamp"). Overridable for testing / non-Mac setups.
PJ_HOST = os.environ.get("PJ_HOST") or FG_HOST   # default: same egress path as FlightGear
PJ_PORT = int(os.environ.get("PJ_PORT") or 9870)

# Scope: in-process NPS emitter (nps_scope.c) → local port → normalize → PJ_HOST.
# Routed through this process (not straight to the Mac) so every packet gets the
# shared aircraft-agnostic schema rewrite from pj_json_relay.normalize_obj.
SCOPE_LOCAL_PORT = 9871
SCOPE_DECIM = 2         # emit every Nth sim step (~500 Hz at 1 kHz sim rate)

# Telemetry JSON stream: server's own UDP/JSON emitter is the real PlotJuggler bridge
# (see pj_json_relay.py). Point it at a local port so we can tee it to a capture file;
# it is only forwarded to PlotJuggler when the scope is disabled (--no-scope), so the
# GUI never gets the same signals from two sources at once.
TELEM_JSON_LOCAL_PORT = 9870

# Logs land in /workspace/sim_logs/ (bind-mounted to the host) so they survive
# container exit. Fall back to /tmp if running outside a container.
_LOG_DIR = os.environ.get("MFC_LOG_DIR", "/workspace/sim_logs")
os.makedirs(_LOG_DIR, exist_ok=True)
_TS = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
LOG_FILE = os.path.join(_LOG_DIR, f"mfc_sim_{_TS}.jsonl")
DEBUG_LOG_FILE = os.path.join(_LOG_DIR, f"mfc_sim_{_TS}_debug.log")

# ── settings.xml (name → index), for DL_SETTING by name ─────────────────────
SETTINGS_XML = f"{PPRZ}/var/aircrafts/{AC_NAME}/settings.xml"
try:
    SETTINGS = PprzSettingsParser.parse(SETTINGS_XML) if os.path.isfile(SETTINGS_XML) else None
except Exception as e:
    print(f"Warning: could not parse {SETTINGS_XML}: {e}", file=sys.stderr)
    SETTINGS = None


# ── shared state ─────────────────────────────────────────────────────────────
state = {
    "phi": 0.0, "theta": 0.0, "psi": 0.0,
    "p":   0.0, "q":     0.0, "r":   0.0,
    "lat": 0.0, "lon":   0.0,
    "alt": 0.0, "agl":   0.0,
    "rc_roll": 0, "rc_pitch": 0, "rc_yaw": 0, "rc_thrust": 0,
    "t":   0.0,
    "cmd": "",
}

debug_log: deque = deque(maxlen=20)

# ── Ivy callbacks (PprzMessage, named field access — no manual parsing) ─────
def on_rate_attitude(ac_id, msg):
    if str(ac_id) != str(AC_ID): return
    state["p"], state["q"], state["r"] = msg["p"], msg["q"], msg["r"]
    state["phi"], state["theta"], state["psi"] = msg["phi"], msg["theta"], msg["psi"]
    state["t"] = time.monotonic()

def on_pos_llh(ac_id, msg):
    if str(ac_id) != str(AC_ID): return
    state["lat"] = msg["lat_geod"] * R2D
    state["lon"] = msg["lon"] * R2D
    state["alt"] = msg["asl"]
    state["agl"] = msg["agl"]

def on_rotorcraft_cmd(ac_id, msg):
    if str(ac_id) != str(AC_ID): return
    state["rc_roll"], state["rc_pitch"] = msg["cmd_roll"], msg["cmd_pitch"]
    state["rc_yaw"], state["rc_thrust"] = msg["cmd_yaw"], msg["cmd_thrust"]


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

def render():
    s = state
    age = time.monotonic() - s["t"]
    stale = age > 1.0
    status = f"{GREY}stale ({age:.1f}s){RESET}" if stale else f"{GREEN}live{RESET}"
    cmd_line = f"  {YELLOW}CMD:{RESET} {s['cmd']}" if s["cmd"] else f"  {GREY}no command sent yet{RESET}"

    lines = [
        f"{BOLD}{'─' * 60}{RESET}",
        f"  {BOLD}{CYAN}{AC_NAME} NPS (ac_id {AC_ID}){RESET}   {status}",
        f"  {cmd_line.strip()}",
        f"{'─' * 60}",
        "",
        f"  {BOLD}JSBSim truth{RESET}",
        f"    Lat {s['lat']:+12.6f}°   Lon {s['lon']:+12.6f}°",
        f"    Alt {s['alt']:+10.2f} m MSL   AGL {s['agl']:+8.2f} m",
        f"    Roll  {bar(s['phi'],   -45, 45, unit='°')}",
        f"    Pitch {bar(s['theta'], -45, 45, unit='°')}",
        f"    Yaw   {s['psi']:+8.2f}°",
        f"    p {bar(s['p'], -60, 60, unit='°/s')}",
        f"    q {bar(s['q'], -60, 60, unit='°/s')}",
        f"    r {bar(s['r'], -60, 60, unit='°/s')}",
        "",
        f"  {BOLD}Motor commands (ROTORCRAFT_CMD){RESET}",
        f"    roll {s['rc_roll']:+6d}  pitch {s['rc_pitch']:+6d}  yaw {s['rc_yaw']:+6d}  thrust {s['rc_thrust']:+6d}",
        "",
        f"  {BOLD}DEBUG{RESET}",
        *[f"    {GREY}{line}{RESET}" for line in list(debug_log)[-6:]],
        f"{'─' * 60}",
        f"  {GREY}Telemetry capture → {LOG_FILE}   Ctrl-C to stop{RESET}",
        f"  {GREY}Deeper MFC/INDI/WLS signals: PlotJuggler on {PJ_HOST}:{PJ_PORT}{RESET}",
    ]
    sys.stdout.write(CLEAR + "\n".join(lines) + "\n")
    sys.stdout.flush()


# ── telemetry JSON capture + relay ────────────────────────────────────────────
# server streams its UDP/JSON telemetry to TELEM_JSON_LOCAL_PORT on localhost;
# we tee every (sanitized + normalized) datagram to LOG_FILE. It is forwarded to
# PlotJuggler only when the scope emitter is off (--no-scope): in a SITL run the
# scope carries the same signals as ivy telemetry at full rate plus ground truth,
# so forwarding both would draw every curve twice from two clocks.
def telemetry_capture_relay():
    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock.bind(("127.0.0.1", TELEM_JSON_LOCAL_PORT))
    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    forward_to = (PJ_HOST, PJ_PORT)
    with open(LOG_FILE, "w", buffering=1) as f:
        while True:
            data, _ = recv_sock.recvfrom(65535)
            clean = sanitize_json(data)
            if clean is None:
                continue
            f.write(clean.decode("utf-8", errors="replace") + "\n")
            if _USE_SCOPE:
                continue
            try:
                send_sock.sendto(clean, forward_to)
            except OSError:
                pass


# ── scope normalize + relay ───────────────────────────────────────────────────
# The in-process NPS scope emitter sends raw JSON (root "<AIRFRAME> (sim)") to
# SCOPE_LOCAL_PORT; rewrite each packet to the shared "uav" schema and forward
# it to PlotJuggler.
def scope_relay():
    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock.bind(("127.0.0.1", SCOPE_LOCAL_PORT))
    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    forward_to = (PJ_HOST, PJ_PORT)
    while True:
        data, _ = recv_sock.recvfrom(65535)
        clean = sanitize_json(data)
        if clean is None:
            continue
        try:
            send_sock.sendto(clean, forward_to)
        except OSError:
            pass


# ── command senders (native Ivy ground messages, no hand-rolled frames) ─────
def send_block(ivy: IvyMessagesInterface, block_id: int, label: str):
    msg = PprzMessage("ground", "JUMP_TO_BLOCK")
    msg["ac_id"] = AC_ID
    msg["block_id"] = block_id
    ivy.send(msg)
    state["cmd"] = f"BLOCK {block_id} ({label})"

def send_setting_by_name(ivy: IvyMessagesInterface, name: str, value: float, label: str = None):
    if SETTINGS is None:
        print(f"[ctrl] no settings.xml for {AC_NAME} ({SETTINGS_XML} missing) — rebuild first", flush=True)
        return
    try:
        setting = SETTINGS[name]
    except AttributeError:
        print(f"[ctrl] unknown setting '{name}' for {AC_NAME}", flush=True)
        return
    msg = PprzMessage("ground", "DL_SETTING")
    msg["ac_id"] = AC_ID
    msg["index"] = setting.index
    msg["value"] = float(value)
    ivy.send(msg)
    state["cmd"] = label or f"SETTING {name}={value}"

def takeoff_sequence(ivy: IvyMessagesInterface):
    time.sleep(5.0)
    send_block(ivy, 2, "Start Engine")
    time.sleep(0.5)
    send_block(ivy, 3, "Takeoff")


def cmd_loop(ivy: IvyMessagesInterface):
    """Read text commands from stdin and dispatch them. Runs as a daemon thread.

    Accepts:  block <id> | setting <name> <float>
    """
    for raw in sys.stdin:
        parts = raw.strip().split()
        if not parts:
            continue
        cmd = parts[0]
        try:
            if cmd == "block" and len(parts) >= 2:
                send_block(ivy, int(parts[1]), parts[1])
            elif cmd == "setting" and len(parts) >= 3:
                send_setting_by_name(ivy, parts[1], float(parts[2]))
                print(f"[ctrl] setting {parts[1]}={parts[2]}", flush=True)
            else:
                print(
                    f"[cmd] unknown: {raw.strip()!r}  "
                    "(commands: block <id> | setting <name> <val>)",
                    flush=True,
                )
        except Exception as exc:
            print(f"[cmd] error: {exc}", flush=True)


# ── main ──────────────────────────────────────────────────────────────────────
def main():
    env = {**os.environ, "PAPARAZZI_HOME": PPRZ}

    print("Starting Paparazzi server …")
    server = subprocess.Popen(
        [SERVER, "-b", IVY_BUS, "-n", "-udp_json_stream_addr", "127.0.0.1",
         "-udp_json_stream_port", str(TELEM_JSON_LOCAL_PORT)],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )

    print("Starting Paparazzi link (UDP 4242) …")
    link = subprocess.Popen(
        [LINK, "-b", IVY_BUS, "-udp"],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )

    # RC source: a compiled-in NPS script (--rc_script N) drives the sticks
    # (script 0=hover, 1=step_roll, 2=step_pitch, 3=step_yaw, 4=ff; all auto-
    # take off for the first 8 s — see sw/simulator/nps/nps_radio_control.c).
    # Otherwise RC is disabled (--norc) and control comes from the flight plan.
    # (simsitl's own native CLI — see the pprzsim-launch note in the module
    # docstring for why we call it directly instead of through that launcher.)
    _sim_cmd = [SIMSITL]
    _sim_cmd += ["--rc_script", str(_RC_SCRIPT)] if _RC_SCRIPT is not None else ["--norc"]
    if _USE_FG:
        _sim_cmd += ["--fg_host", FG_HOST, "--fg_port", str(FG_PORT), "--fg_fdm"]
    if _USE_SCOPE:
        _sim_cmd += ["--scope_host", "127.0.0.1",
                     "--scope_port", str(SCOPE_LOCAL_PORT),
                     "--scope_decim", str(SCOPE_DECIM)]
    if _GDB:
        _sim_cmd = ["gdbserver", ":1234"] + _sim_cmd
    print("Starting NPS sim …" + (" (gdbserver :1234, waiting for debugger)" if _GDB else ""))
    sim = subprocess.Popen(
        _sim_cmd,
        env=env, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
        text=True, bufsize=1,
    )

    ivy = IvyMessagesInterface(f"sim_anton_{AC_NAME}", ivy_bus=IVY_BUS)

    def shutdown(sig=None, frame=None):
        print("\nShutting down …")
        sim.terminate(); link.terminate(); server.terminate()
        try: ivy.shutdown()
        except Exception: pass
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    print("Waiting for sim to start …")
    time.sleep(3)

    ivy.subscribe(on_rate_attitude,  PprzMessage("telemetry", "NPS_RATE_ATTITUDE"))
    ivy.subscribe(on_pos_llh,        PprzMessage("telemetry", "NPS_POS_LLH"))
    ivy.subscribe(on_rotorcraft_cmd, PprzMessage("telemetry", "ROTORCRAFT_CMD"))

    state["t"] = time.monotonic()
    threading.Thread(target=sim_stdout_reader,       args=(sim,), daemon=True).start()
    threading.Thread(target=takeoff_sequence,        args=(ivy,), daemon=True).start()
    threading.Thread(target=telemetry_capture_relay, daemon=True).start()
    threading.Thread(target=cmd_loop,                args=(ivy,), daemon=True).start()
    if _USE_SCOPE:
        threading.Thread(target=scope_relay, daemon=True).start()
    print(f"Telemetry capture → {LOG_FILE}   Debug → {DEBUG_LOG_FILE}")
    if _USE_SCOPE:
        print(f"PlotJuggler feed: NPS scope (normalized '/uav' schema) → {PJ_HOST}:{PJ_PORT} "
              f"(decim {SCOPE_DECIM}, ~{1000//SCOPE_DECIM} Hz)")
    else:
        print(f"PlotJuggler feed: ivy telemetry (normalized '/uav' schema) → {PJ_HOST}:{PJ_PORT}")

    if _USE_RENDER:
        while True:
            render()
            time.sleep(0.1)
    else:
        while True:
            time.sleep(1)


if __name__ == "__main__":
    main()
