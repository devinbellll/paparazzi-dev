#!/usr/bin/env python3
"""
analyze_mfc.py — print a concise per-axis MFC stability report to guide tuning.

Accepts EITHER feed (same metrics for sim and real flight — the whole point of
the unified NPS_SCOPE schema):
  • a wide CSV from sim_anton.py            (legacy keys: mfc_err_phi, agl, …)
  • a unified scope-JSON stream / capture   (keys: mfc/roll/err, truth/agl, …)
    — i.e. a recorded NPS scope capture, or the output of tools/sdlog2scope.py
      on a downloaded SD flight log. One analyser, three feeds.

Format is auto-detected from the file extension / first byte.

Usage:
    python3 analyze_mfc.py CAPTURE.csv  [--skip SEC] [--steady]
    python3 analyze_mfc.py CAPTURE.json [--skip SEC] [--steady]   # scope ndjson

Options:
    --skip SEC   Ignore the first SEC seconds (startup transient). Default: 5.0
    --steady     Only analyze the steady-state window (skip first + last 20 %).
"""

import csv
import json
import math
import sys

R2D = math.degrees(1)

# ── scope-key -> legacy-CSV-key mapping ───────────────────────────────────────
# The rest of this script speaks the CSV keys (mfc_err_phi, …). A scope-JSON row
# ({"t":…, "truth":{…}, "mfc/roll/err":…}) is translated into the same keys here,
# preserving the CSV's UNIT conventions:
#   • errors / fk / cmd  : radians / unitless — passed through as-is
#   • phi/theta          : the script multiplies by R2D, so feed RADIANS
#                          (truth/* angles are degrees in the scope → /R2D)
#   • agl                : metres, as-is
#   • rc_thrust          : motor-output proxy = mean(mfc/act) (pprz 0..9600)
def _scope_row_to_csv(o):
    """Flatten one scope-JSON object (truth nested or flat slash-keys) to the
    legacy CSV key space used by the metrics below."""
    g = {}

    def gv(k):
        if k in o:
            return o[k]
        # truth.* may be nested under "truth"
        if k.startswith("truth/") and isinstance(o.get("truth"), dict):
            return o["truth"].get(k.split("/", 1)[1])
        return None

    r = {}
    r["wall"] = o.get("t")
    r["mfc_err_phi"]   = gv("mfc/roll/err")
    r["mfc_err_theta"] = gv("mfc/pitch/err")
    r["mfc_err_psi"]   = gv("mfc/yaw/err")
    r["mfc_fk_phi"]    = gv("mfc/roll/fk")
    r["mfc_fk_theta"]  = gv("mfc/pitch/fk")
    r["mfc_fk_psi"]    = gv("mfc/yaw/fk")
    r["mfc_cmd_phi"]   = gv("mfc/roll/cmd")
    r["mfc_cmd_theta"] = gv("mfc/pitch/cmd")
    r["mfc_cmd_psi"]   = gv("mfc/yaw/cmd")

    act = gv("mfc/act")
    if isinstance(act, list):
        for i in range(4):
            r[f"mfc_u{i}"] = act[i] if i < len(act) else None
        thr = [a for a in act[:4] if a is not None]
        r["rc_thrust"] = sum(thr) / len(thr) if thr else None
    else:
        r["rc_thrust"] = None

    agl = gv("truth/agl")
    r["agl"] = agl
    phi_deg   = gv("truth/phi")
    theta_deg = gv("truth/theta")
    # script expects radians (it applies *R2D); scope truth angles are degrees
    r["phi"]   = (phi_deg   / R2D) if phi_deg   is not None else None
    r["theta"] = (theta_deg / R2D) if theta_deg is not None else None
    return r


def _load_rows(path):
    """Return list of CSV-key row dicts from a .csv or scope-.json/.ndjson file."""
    # sniff: extension first, then first non-space byte
    is_json = path.lower().endswith((".json", ".ndjson", ".jsonl"))
    if not is_json:
        with open(path) as f:
            for ch in f.read(64):
                if ch.isspace():
                    continue
                is_json = ch in "{["
                break

    rows = []
    if is_json:
        with open(path) as f:
            for line in f:
                line = line.strip().rstrip(",")
                if not line or line in ("[", "]"):
                    continue
                try:
                    obj = json.loads(line)
                except json.JSONDecodeError:
                    continue
                rows.append(_scope_row_to_csv(obj))
    else:
        with open(path, newline="") as f:
            for row in csv.DictReader(f):
                rows.append({k: float(v) if v not in ("", "None") else None
                             for k, v in row.items()})
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
    print(f"Usage: {sys.argv[0]} <capture.csv|capture.json> [--skip SEC] [--steady]",
          file=sys.stderr)
    sys.exit(1)
csv_path = in_path   # kept for the report header below

# ── load (CSV or unified scope JSON, auto-detected) ──────────────────────────
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

def grade(rms_deg):
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
print(f"  {'Roll':6s}  {rms_roll:>8.3f}°  {max_roll:>8.3f}°  {grade(rms_roll):12s}  {sat_roll:>7d}")
print(f"  {'Pitch':6s}  {rms_pitch:>8.3f}°  {max_pitch:>8.3f}°  {grade(rms_pitch):12s}  {sat_pitch:>7d}")
print(f"  {'Yaw':6s}  {rms_yaw:>8.3f}°  {max_yaw:>8.3f}°  {grade(rms_yaw):12s}  {sat_yaw:>7d}")

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
