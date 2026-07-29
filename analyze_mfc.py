#!/usr/bin/env python3
"""
analyze_mfc.py — print a concise per-axis MFC stability report to guide tuning.

Reads the canonical wide `/uav/...` CSV, which BOTH data paths now produce:

  • a sim run          — sim_anton.py writes sim_logs/mfc_sim_<TS>.csv
  • a real flight      — tools/sdlog2scope.py converts an SD log's .data

so the same metrics grade a simulation and a flight with no per-feed special
casing. Columns that a given feed doesn't carry (a flight has no /uav/TRUTH/*
unless OptiTrack is running) simply report as n/a.

Usage:
    python3 analyze_mfc.py RUN.csv [--skip SEC] [--steady]

Options:
    --skip SEC   Ignore the first SEC seconds (startup transient). Default: 5.0
    --steady     Only analyze the steady-state window (skip first + last 20 %).
"""

import csv
import math
import sys

R2D = math.degrees(1)

# ── canonical column -> internal metric key ──────────────────────────────────
# Everything below this point speaks the short internal keys. All angles in the
# schema are RADIANS (the NPS scope emits the firmware's own units), and the
# report converts to degrees where it prints them.
COLUMN_MAP = {
    "/uav/MFC_STAB/err_phi":   "mfc_err_phi",
    "/uav/MFC_STAB/err_theta": "mfc_err_theta",
    "/uav/MFC_STAB/err_psi":   "mfc_err_psi",
    "/uav/MFC_STAB/fk_phi":    "mfc_fk_phi",
    "/uav/MFC_STAB/fk_theta":  "mfc_fk_theta",
    "/uav/MFC_STAB/fk_psi":    "mfc_fk_psi",
    "/uav/MFC_STAB/cmd_phi":   "mfc_cmd_phi",
    "/uav/MFC_STAB/cmd_theta": "mfc_cmd_theta",
    "/uav/MFC_STAB/cmd_psi":   "mfc_cmd_psi",
    "/uav/TRUTH/agl":          "agl",
    "/uav/TRUTH/phi":          "phi",
    "/uav/TRUTH/theta":        "theta",
}
# WLS actuator outputs -> mfc_u0..3; their mean is the thrust proxy.
U_COLUMNS = [f"/uav/WLS_U/u/u_{i}" for i in range(4)]


def _num(v):
    if v is None or v in ("", "None"):
        return None
    try:
        return float(v)
    except ValueError:
        return None      # array cells are comma-joined strings; not a metric


# The two converters name the leading time column differently, on purpose:
# sdlog2scope.py keeps "Time" so its output stays diffable against the GCS's own
# CSV export, while scope2csv.py uses lowercase "time". Accept either.
TIME_COLUMNS = ("time", "Time")


def _load_rows(path):
    """Return internal-key row dicts from a canonical wide /uav CSV."""
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        fields = reader.fieldnames or []
        time_col = next((c for c in TIME_COLUMNS if c in fields), None)
        if time_col is None:
            sys.exit(f"analyze_mfc: {path} is not a canonical /uav run CSV (no "
                     f"{' or '.join(TIME_COLUMNS)} column). Convert it with "
                     "tools/sdlog2scope.py or tools/scope2csv.py first.")
        for src in reader:
            r = {"wall": _num(src.get(time_col))}
            for col, key in COLUMN_MAP.items():
                r[key] = _num(src.get(col))
            us = []
            for i, col in enumerate(U_COLUMNS):
                u = _num(src.get(col))
                r[f"mfc_u{i}"] = u
                if u is not None:
                    us.append(u)
            r["rc_thrust"] = sum(us) / len(us) if us else None
            rows.append(r)
    return rows

# ── parse args ────────────────────────────────────────────────────────────────
args = sys.argv[1:]
in_path  = None
skip_sec  = 5.0
steady_only = False

i = 0
while i < len(args):
    a = args[i]
    if a == "--skip" and i + 1 < len(args):
        skip_sec = float(args[i+1]); i += 2
    elif a == "--steady":
        steady_only = True; i += 1
    elif in_path is None:
        in_path = a; i += 1
    else:
        i += 1

if in_path is None:
    print(f"Usage: {sys.argv[0]} <run.csv> [--skip SEC] [--steady]", file=sys.stderr)
    sys.exit(1)
csv_path = in_path   # kept for the report header below

# ── load the canonical wide /uav CSV ─────────────────────────────────────────
rows = _load_rows(in_path)

if not rows:
    print("Capture is empty (no rows parsed).", file=sys.stderr)
    sys.exit(1)

# Reconstruct a sim-time column from the wall clock / scope timestamp
t0 = rows[0].get("wall") or 0.0
for row in rows:
    row["sim_t"] = (row.get("wall") or t0) - t0

total_sec  = rows[-1]["sim_t"]
skip_end   = skip_sec
if steady_only:
    skip_end = total_sec * 0.2

analysis   = [r for r in rows if r["sim_t"] >= skip_sec]
n          = len(analysis)

if n < 10:
    print(f"Only {n} rows after {skip_sec}s skip — not enough data.")
    sys.exit(0)

R2D = math.degrees(1)

# ── helpers ───────────────────────────────────────────────────────────────────
def _vals(key):
    return [r[key] for r in analysis if r.get(key) is not None]

def rms(vs):
    if not vs: return float("nan")
    return math.sqrt(sum(v*v for v in vs) / len(vs))

def fmt_bar(val, lo, hi, width=20):
    # A missing signal must not draw a FULL bar (nan comparisons are all False,
    # which is how "no data" used to render as "pegged at maximum").
    if val != val:
        return f"[{'·' * width}] n/a"
    frac = max(0.0, min(1.0, (val - lo) / (hi - lo)))
    filled = int(frac * width)
    return f"[{'█' * filled}{'░' * (width - filled)}]"

def count_saturated(vs, lo, hi, margin=50):
    return sum(1 for v in vs if v <= lo + margin or v >= hi - margin)

# ── attitude errors (in degrees) ──────────────────────────────────────────────
err_roll  = [v * R2D for v in _vals("mfc_err_phi")]
err_pitch = [v * R2D for v in _vals("mfc_err_theta")]
err_yaw   = [v * R2D for v in _vals("mfc_err_psi")]

rms_roll  = rms(err_roll)
rms_pitch = rms(err_pitch)
rms_yaw   = rms(err_yaw)

max_roll  = max((abs(v) for v in err_roll),  default=float("nan"))
max_pitch = max((abs(v) for v in err_pitch), default=float("nan"))
max_yaw   = max((abs(v) for v in err_yaw),   default=float("nan"))

# ── command saturation counts ─────────────────────────────────────────────────
cmd_roll  = _vals("mfc_cmd_phi")
cmd_pitch = _vals("mfc_cmd_theta")
cmd_yaw   = _vals("mfc_cmd_psi")

# Stab cmds are WLS virtual inputs (rad/s² or dimensionless) — check for
# extreme excursions rather than fixed ±9600 bounds.
def _sat_count_rel(vs, thresh_frac=0.95):
    if not vs: return 0
    abs_max = max(abs(v) for v in vs)
    if abs_max < 1e-6: return 0
    thresh = abs_max * thresh_frac
    return sum(1 for v in vs if abs(v) >= thresh)

sat_roll  = _sat_count_rel(cmd_roll)
sat_pitch = _sat_count_rel(cmd_pitch)
sat_yaw   = _sat_count_rel(cmd_yaw)

# ── F_k estimator health ──────────────────────────────────────────────────────
fk_roll  = _vals("mfc_fk_phi")
fk_pitch = _vals("mfc_fk_theta")
fk_yaw   = _vals("mfc_fk_psi")

def _range_str(vs):
    if not vs: return "n/a"
    return f"{min(vs):+.2f} … {max(vs):+.2f}"

# ── altitude / thrust ─────────────────────────────────────────────────────────
agl_vals    = _vals("agl")
thrust_vals = [int(v) for v in _vals("rc_thrust")]

agl_mean   = sum(agl_vals) / len(agl_vals) if agl_vals else float("nan")
agl_rms    = rms([v - agl_mean for v in agl_vals]) if agl_vals else float("nan")
agl_min    = min(agl_vals, default=float("nan"))
agl_max    = max(agl_vals, default=float("nan"))

thrust_min  = min(thrust_vals, default=0)
thrust_max  = max(thrust_vals, default=0)
thrust_sat  = count_saturated(thrust_vals, 0, 9600, margin=100)
# gz saturating means thrust near 9600 continuously
gz_sat_frac = sum(1 for v in thrust_vals if v >= 9500) / max(1, len(thrust_vals))

# ── WLS actuator spread ────────────────────────────────────────────────────────
u0 = _vals("mfc_u0"); u1 = _vals("mfc_u1")
u2 = _vals("mfc_u2"); u3 = _vals("mfc_u3")

def _mean(vs): return sum(vs)/len(vs) if vs else float("nan")
u_mean = [_mean(u0), _mean(u1), _mean(u2), _mean(u3)]

# ── attitude trajectory ────────────────────────────────────────────────────────
phi_vals   = [v * R2D for v in _vals("phi")]
theta_vals = [v * R2D for v in _vals("theta")]
phi_max    = max((abs(v) for v in phi_vals),   default=float("nan"))
theta_max  = max((abs(v) for v in theta_vals), default=float("nan"))

# ── crash / diverge detection ─────────────────────────────────────────────────
crashed    = any(r.get("agl", 10) is not None and r["agl"] < 0.05 for r in analysis)
diverged   = phi_max > 45 or theta_max > 45

# ── report ────────────────────────────────────────────────────────────────────
W = 60
LINE = "─" * W

def grade(rms_deg, vals=None):
    # An identically-zero channel means the controller never ran (e.g. MFC left
    # in shadow mode, or the message logged before the law was engaged). That is
    # NOT perfect tracking -- say so, instead of awarding it a green GOOD.
    if vals is not None and vals and not any(v for v in vals):
        return "no signal"
    if rms_deg != rms_deg: return "n/a"     # column absent in this feed
    if rms_deg < 1.0:  return "GOOD    ✓"
    if rms_deg < 3.0:  return "OK      ·"
    if rms_deg < 8.0:  return "MARGINAL"
    return               "BAD     ✗"

print(LINE)
print(f"  MFC Stability Report")
print(f"  Input  : {csv_path}")
print(f"  Duration: {total_sec:.1f} s   Analysis window: {skip_sec:.1f}–{total_sec:.1f} s  ({n} rows)")
print(LINE)

if crashed:
    print("  *** CRASH DETECTED (AGL < 5 cm) ***")
if diverged:
    print(f"  *** ATTITUDE DIVERGED: phi_max={phi_max:.1f}° theta_max={theta_max:.1f}° ***")
if gz_sat_frac > 0.1:
    print(f"  *** GZ SATURATED: thrust@9600 for {gz_sat_frac*100:.0f}% of run (fix gz clamp/time_trajec) ***")

print()
print("  ATTITUDE ERRORS (MFC stabilizer, after skip)")
print(f"  {'Axis':6s}  {'RMS err':>9s}  {'Max |err|':>9s}  {'Grade':12s}  {'CmdSat':>7s}")
print(f"  {'Roll':6s}  {rms_roll:>8.3f}°  {max_roll:>8.3f}°  {grade(rms_roll, err_roll):12s}  {sat_roll:>7d}")
print(f"  {'Pitch':6s}  {rms_pitch:>8.3f}°  {max_pitch:>8.3f}°  {grade(rms_pitch, err_pitch):12s}  {sat_pitch:>7d}")
print(f"  {'Yaw':6s}  {rms_yaw:>8.3f}°  {max_yaw:>8.3f}°  {grade(rms_yaw, err_yaw):12s}  {sat_yaw:>7d}")

print()
print("  F_k ESTIMATOR  (should track without saturation)")
print(f"  Roll  F_k range: {_range_str(fk_roll)}")
print(f"  Pitch F_k range: {_range_str(fk_pitch)}")
print(f"  Yaw   F_k range: {_range_str(fk_yaw)}")

print()
print("  ALTITUDE / THRUST")
print(f"  AGL mean  = {agl_mean:.2f} m   RMS dev = {agl_rms:.3f} m   range [{agl_min:.2f}, {agl_max:.2f}]")
print(f"  Thrust    range [{thrust_min:5d}, {thrust_max:5d}]  sat@9600: {gz_sat_frac*100:.0f}% of steps")
print(f"  WLS u mean: NE={u_mean[0]:+7.1f}  SE={u_mean[1]:+7.1f}  SW={u_mean[2]:+7.1f}  NW={u_mean[3]:+7.1f}")

print()
print("  ATTITUDE EXCURSION")
print(f"  Roll:  max |φ| = {phi_max:6.2f}°  {fmt_bar(phi_max, 0, 45)}")
print(f"  Pitch: max |θ| = {theta_max:6.2f}°  {fmt_bar(theta_max, 0, 45)}")

print()
print("  TUNING HINTS")
if gz_sat_frac > 0.05:
    print("  • gz is saturating → increase GZ_TIME_TRAJECTORY (makes ref smoother)")
    print("    and/or reduce GZ_KP. Check GZ u_min/u_max clamp was applied.")
if rms_roll > 5 or rms_pitch > 5:
    print("  • roll/pitch RMS high → try increasing ROLL/PITCH_KP slightly (poles faster)")
    print("    or increase TIME_TRAJECTORY (softer reference → less initial error)")
if rms_yaw > 8:
    print("  • yaw RMS high → check YAW_ALPHA matches the plant (73.5294 in XML)")
    print("    and YAW_KP. Increase INT_WINDOW slightly for a smoother F_k estimate.")
if sat_roll > n * 0.01 or sat_pitch > n * 0.01:
    print("  • stab command saturation → check ALPHA values; lower alpha = weaker actuation")
    print("    gain. May need to increase ALPHA or lower KP.")
if agl_rms > 1.0 and gz_sat_frac < 0.05:
    print("  • altitude noisy but not saturating → try reducing GZ_KP or increasing")
    print("    GZ_COMMAND_FILTER for smoother thrust output.")
if phi_max < 5 and theta_max < 5 and rms_roll < 2 and rms_pitch < 2:
    print("  • Attitude looks stable. Consider a tighter GZ setpoint step to test altitude.")
if not (gz_sat_frac > 0.05 or rms_roll > 5 or rms_pitch > 5 or rms_yaw > 8
        or sat_roll > n * 0.01 or sat_pitch > n * 0.01 or agl_rms > 1.0):
    print("  • No obvious issues. Next: test with position hold / step inputs.")

print(LINE)
