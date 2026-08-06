#!/usr/bin/env python3
"""
Launch a Paparazzi NPS simulation headlessly, drive it, and capture the run.

This is the *programmatic* way into the sim: build, fly, watch stdout, get a CSV.
For interactive work with a GUI, use the Paparazzi control panel instead
(conf/userconf/ENAC/control_panel_mfc.xml, sessions "MFC VM SIM" /
"ANTON_MFC VM SIM") — it runs the same processes with the GCS attached, and
anything the GCS does well (strips, settings panels, live plotting) is not
reimplemented here.

Usage:  python3 sim_anton.py AIRCRAFT [flags]
        Ctrl-C to stop (kills child processes cleanly).

  AIRCRAFT  Aircraft name as registered in the conf XML (e.g. Hoops_111_MFC).
            The CONF env var selects which conf file to search
            (default: conf/userconf/ENAC/conf_mfc.xml, relative to PAPARAZZI_HOME).

Flags:
  --nav SEQ         Comma-separated flight-plan BLOCK NAMES to run, with `+N`
                    tokens to wait N seconds between them. Default:
                    "Start Engine,Takeoff" (the usual auto-takeoff). Use
                    --nav "" to send nothing and drive it yourself.
                      --nav "Start Engine,Takeoff,+15,Nav"
                    Names are resolved from var/aircrafts/<AC>/flight_plan.xml,
                    so they are exactly the names on the GCS strip buttons; an
                    unknown name lists what is available and exits.
  --set NAME=VALUE  Set a GCS setting by shortname once the nav sequence is done.
                    Repeatable. This is the "same flight, different gains" knob:
                      --set guidance_mfc_kp=2.5 --set guidance_mfc_kd=0.8
  --ic FILE         Override the JSBSim initial-conditions file (default: the
                    aircraft's compiled-in NPS_JSBSIM_INIT, or reset00.xml, both
                    on-ground). FILE is resolved relative to
                    conf/simulator/jsbsim/aircraft/, e.g. reset_inflight.xml
                    spawns 2 m AGL already hovering. This only changes the FDM's
                    physical state -- the autopilot still boots with motors
                    killed (NavKillThrottle in Wait GPS / Holding point), so
                    pair it with --nav jumping through "Start Engine" (which
                    resurrects the motors) straight into a hold/guided block,
                    skipping "Takeoff" since altitude is already there:
                      --ic reset_inflight.xml --nav "Start Engine,+0.5,Standby"
                    --nav "" alone leaves it hanging in "Wait GPS" with motors
                    dead -- looks frozen, not flying.
  --rc_script N     Drive RC from a compiled-in NPS stick script instead of --norc
                    (0=hover 1=step_roll 2=step_pitch 3=step_yaw 4=ff; auto-takeoff
                    for the first 8 s).
  --fg              Stream NET_FDM pose to FlightGear on the Mac host (:5501).
  --gdb             Wrap the sim in gdbserver :1234 and wait for a debugger.
  --no-scope        Feed PlotJuggler (and the CSV) from server's ivy telemetry
                    instead of the in-process NPS scope — i.e. exactly what a real
                    flight looks like, downsampled and without ground truth.

Command delivery:
  Blocks and settings go out as native Ivy messages (ground/JUMP_TO_BLOCK,
  ground/DL_SETTING) — the same mechanism the GCS strip buttons and settings panel
  use. `server` encodes them into binary pprz frames and forwards them over the
  sim's datalink; we never touch that wire format.

Capture (PlotJuggler feed and the on-disk log are the SAME stream):
    default:     the in-process NPS scope emitter (nps_scope.c in simsitl) —
                 ground truth + firmware-registered vars at full decimated sim
                 rate, on a local port.
    --no-scope:  server's ivy UDP/JSON telemetry stream instead.
  Whichever is selected is normalized to the shared aircraft-agnostic schema
  (root "uav", MFC_* branches — see pj_json_relay.py), forwarded to PlotJuggler,
  AND written to sim_logs/mfc_sim_<TS>.csv in the wide `/uav/...` CSV schema —
  the same format tools/sdlog2scope.py produces from a real SD flight log, so one
  layout and one analyser cover both. PJ_HOST / PJ_PORT override the PlotJuggler
  destination (default: the Mac host, 9870).

Note on `pprzsim-launch`: paparazzi ships sw/simulator/pprzsim-launch as the canonical NPS
launcher, but it only `execv`s simsitl with a handful of flags (fg/rc_script/norc/js/
ivy_bus) — it cannot express the in-process scope emitter or a gdbserver wrap, and has no
passthrough for extra arguments. So simsitl is invoked directly here; that is simsitl's own
native CLI, not something reinvented by this script.
"""

import datetime
import json
import os
import signal
import socket
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET

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
from tools.scope2csv import ScopeCsvWriter

# ── Parse positional AC_NAME + flags ─────────────────────────────────────────
_flags = {}
_sets = []
AC_NAME = None
_args = sys.argv[1:]
i = 0
while i < len(_args):
    a = _args[i]
    if a == "--rc_script" and i + 1 < len(_args):
        _flags["rc_script"] = int(_args[i + 1])
        i += 2
    elif a == "--nav" and i + 1 < len(_args):
        _flags["nav"] = _args[i + 1]
        i += 2
    elif a == "--ic" and i + 1 < len(_args):
        _flags["ic"] = _args[i + 1]
        i += 2
    elif a == "--set" and i + 1 < len(_args):
        _sets.append(_args[i + 1])
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
    print(f"Usage: {sys.argv[0]} AIRCRAFT [--nav SEQ] [--set NAME=VAL] [--ic FILE] "
          "[--rc_script N] [--fg] [--gdb] [--no-scope]", file=sys.stderr)
    sys.exit(1)

_GDB       = "--gdb"      in _flags
_USE_FG    = "--fg"       in _flags
_USE_SCOPE = "--no-scope" not in _flags
_RC_SCRIPT = _flags.get("rc_script")            # NPS compiled RC script index, or None
_NAV       = _flags.get("nav", "Start Engine,Takeoff")
_IC        = _flags.get("ic")                   # JSBSim initial-conditions file override, or None

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

# PlotJuggler destination (Streaming → UDP Server, protocol JSON, timestamp field
# "timestamp"). Overridable for testing / non-Mac setups.
PJ_HOST = os.environ.get("PJ_HOST") or FG_HOST   # default: same egress path as FlightGear
PJ_PORT = int(os.environ.get("PJ_PORT") or 9870)

# The selected feed arrives on a local port, gets normalized, and is then both
# forwarded to PlotJuggler and written to the CSV. Two ports so the scope and
# server can never collide when both happen to be running.
SCOPE_LOCAL_PORT = 9871      # in-process NPS emitter (nps_scope.c)
TELEM_LOCAL_PORT = 9870      # server.ml's UDP/JSON telemetry
SCOPE_DECIM = 2              # emit every Nth sim step (~500 Hz at 1 kHz sim rate)

# Logs land in /workspace/sim_logs/ (bind-mounted to the host) so they survive
# container exit.
_LOG_DIR = os.environ.get("MFC_LOG_DIR", "/workspace/sim_logs")
os.makedirs(_LOG_DIR, exist_ok=True)
_TS = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
LOG_FILE = os.path.join(_LOG_DIR, f"mfc_sim_{_TS}.csv")
DEBUG_LOG_FILE = os.path.join(_LOG_DIR, f"mfc_sim_{_TS}_debug.log")

# ── settings.xml (name → index), for DL_SETTING by name ─────────────────────
SETTINGS_XML = f"{PPRZ}/var/aircrafts/{AC_NAME}/settings.xml"
try:
    SETTINGS = PprzSettingsParser.parse(SETTINGS_XML) if os.path.isfile(SETTINGS_XML) else None
except Exception as e:
    print(f"Warning: could not parse {SETTINGS_XML}: {e}", file=sys.stderr)
    SETTINGS = None

# ── flight plan (block name → id) ────────────────────────────────────────────
# The build generates var/aircrafts/<AC>/flight_plan.xml with exactly what we
# need: <block name="Start Engine" no="2">. Using names instead of the numeric
# ids means a flight-plan edit can't silently retarget the sequence.
FLIGHT_PLAN_XML = f"{PPRZ}/var/aircrafts/{AC_NAME}/flight_plan.xml"


def load_blocks() -> dict:
    """Return {block name: id}, empty if the flight plan isn't generated yet."""
    try:
        tree = ET.parse(FLIGHT_PLAN_XML)
    except Exception as e:
        print(f"Warning: could not parse {FLIGHT_PLAN_XML}: {e}", file=sys.stderr)
        return {}
    return {b.get("name"): int(b.get("no"))
            for b in tree.getroot().iter("block")
            if b.get("name") is not None and b.get("no") is not None}


BLOCKS = load_blocks()


def parse_nav(spec: str):
    """'Start Engine,+15,Nav' -> [('block','Start Engine'), ('wait',15.0), ...].

    Exits with the available block names if one doesn't exist, rather than
    sending a JUMP_TO_BLOCK for a guessed id.
    """
    steps = []
    for raw in spec.split(","):
        tok = raw.strip()
        if not tok:
            continue
        if tok.startswith("+"):
            try:
                steps.append(("wait", float(tok[1:])))
            except ValueError:
                sys.exit(f"sim_anton: bad --nav wait token {tok!r} (expected e.g. +15)")
        elif tok in BLOCKS:
            steps.append(("block", tok))
        else:
            avail = "\n  ".join(sorted(BLOCKS)) or "(flight plan not generated — build first)"
            sys.exit(f"sim_anton: no flight-plan block named {tok!r} for {AC_NAME}.\n"
                     f"Available blocks:\n  {avail}")
    return steps


# ── simsitl stdout reader ─────────────────────────────────────────────────────
def sim_stdout_reader(proc):
    with open(DEBUG_LOG_FILE, "w", buffering=1) as f:
        for line in proc.stdout:
            stripped = line.rstrip()
            f.write(stripped + "\n")
            print(stripped, flush=True)


# ── feed capture + relay ──────────────────────────────────────────────────────
# One stream in, two consumers: PlotJuggler (UDP) and the run CSV. Both see the
# identical normalized packets, so what you watched live is what you can replot.
def feed_relay(local_port: int, csv_writer: ScopeCsvWriter):
    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock.bind(("127.0.0.1", local_port))
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
        try:
            csv_writer.write_packet(json.loads(clean))
        except (json.JSONDecodeError, ValueError):
            pass


# ── command senders (native Ivy ground messages) ─────────────────────────────
def send_block(ivy: IvyMessagesInterface, block_id: int, label: str):
    msg = PprzMessage("ground", "JUMP_TO_BLOCK")
    msg["ac_id"] = AC_ID
    msg["block_id"] = block_id
    ivy.send(msg)
    print(f"[nav] block {block_id} ({label})", flush=True)


def send_setting_by_name(ivy: IvyMessagesInterface, name: str, value: float):
    if SETTINGS is None:
        print(f"[set] no settings.xml for {AC_NAME} ({SETTINGS_XML} missing) — rebuild first",
              flush=True)
        return
    try:
        setting = SETTINGS[name]
    except AttributeError:
        print(f"[set] unknown setting '{name}' for {AC_NAME}", flush=True)
        return
    msg = PprzMessage("ground", "DL_SETTING")
    msg["ac_id"] = AC_ID
    msg["index"] = setting.index
    msg["value"] = float(value)
    ivy.send(msg)
    print(f"[set] {name} = {value}", flush=True)


def nav_sequence(ivy: IvyMessagesInterface, steps, sets):
    """Run the --nav steps, then apply --set values. Daemon thread."""
    time.sleep(2.0)                      # let the sim settle / GPS fix
    for kind, val in steps:
        if kind == "wait":
            time.sleep(val)
        else:
            send_block(ivy, BLOCKS[val], val)
            time.sleep(0.5)
    for spec in sets:
        name, _, value = spec.partition("=")
        if not _:
            print(f"[set] ignoring {spec!r} (expected NAME=VALUE)", flush=True)
            continue
        try:
            send_setting_by_name(ivy, name.strip(), float(value))
        except ValueError:
            print(f"[set] {spec!r}: {value!r} is not a number", flush=True)


# ── main ──────────────────────────────────────────────────────────────────────
def main():
    env = {**os.environ, "PAPARAZZI_HOME": PPRZ}
    if _IC:
        env["NPS_JSBSIM_INIT_OVERRIDE"] = _IC
    steps = parse_nav(_NAV)
    local_port = SCOPE_LOCAL_PORT if _USE_SCOPE else TELEM_LOCAL_PORT

    print("Starting Paparazzi server …")
    server_cmd = [SERVER, "-b", IVY_BUS, "-n"]
    if not _USE_SCOPE:
        # Only ask server for its JSON stream when it is the selected feed.
        server_cmd += ["-udp_json_stream_addr", "127.0.0.1",
                       "-udp_json_stream_port", str(TELEM_LOCAL_PORT)]
    server = subprocess.Popen(server_cmd, env=env)

    print("Starting Paparazzi link (UDP 4242) …")
    link = subprocess.Popen([LINK, "-b", IVY_BUS, "-udp"], env=env)

    # RC source: a compiled-in NPS script (--rc_script N) drives the sticks
    # (0=hover, 1=step_roll, 2=step_pitch, 3=step_yaw, 4=ff; all auto-take off for
    # the first 8 s — see sw/simulator/nps/nps_radio_control.c). Otherwise RC is
    # disabled (--norc) and control comes from the flight plan.
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
    sim = subprocess.Popen(_sim_cmd, env=env, stdout=subprocess.PIPE,
                           stderr=subprocess.DEVNULL, text=True, bufsize=1)

    ivy = IvyMessagesInterface(f"sim_anton_{AC_NAME}", ivy_bus=IVY_BUS)
    csv_writer = ScopeCsvWriter(LOG_FILE)

    def _debug_nav_status(ac_id, msg):
        print(f"[NAV_STATUS] ac_id={ac_id} block={msg['cur_block']} stage={msg['cur_stage']} "
              f"block_time={msg['block_time']} stage_time={msg['stage_time']} "
              f"hmode={msg['horizontal_mode']}", flush=True)
    ivy.subscribe(_debug_nav_status, PprzMessage("telemetry", "ROTORCRAFT_NAV_STATUS"))

    def shutdown(sig=None, frame=None):
        print(f"\nShutting down … ({csv_writer.n_rows} rows → {LOG_FILE})")
        sim.terminate(); link.terminate(); server.terminate()
        csv_writer.close()
        try: ivy.shutdown()
        except Exception: pass
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    print("Waiting for sim to start …")
    time.sleep(3)

    threading.Thread(target=sim_stdout_reader, args=(sim,), daemon=True).start()
    threading.Thread(target=feed_relay, args=(local_port, csv_writer), daemon=True).start()
    if steps or _sets:
        threading.Thread(target=nav_sequence, args=(ivy, steps, _sets), daemon=True).start()

    feed = "NPS scope" if _USE_SCOPE else "ivy telemetry"
    rate = f", decim {SCOPE_DECIM}, ~{1000 // SCOPE_DECIM} Hz" if _USE_SCOPE else ""
    print(f"Feed: {feed} (normalized '/uav' schema) → PlotJuggler {PJ_HOST}:{PJ_PORT}{rate}")
    print(f"Run capture → {LOG_FILE}   Debug → {DEBUG_LOG_FILE}")
    if steps:
        print("Nav: " + " ".join(f"+{v}s" if k == "wait" else f"[{v}]" for k, v in steps))

    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()
