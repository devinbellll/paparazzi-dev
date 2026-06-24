# 2026-06-22 — Why standalone ANTON_MFC diverges but ANTON_DUAL doesn't (MFC→MFC)

## Question
ANTON_MFC (standalone `oneloop_mfc`) and ANTON_DUAL (dual wrapper) both default to
MFC→MFC, yet ANTON_MFC flies "straight up super fast" and goes unstable while DUAL
flies fine. Same MFC core — why?

## Method
Added `#ifdef SITL` printf instrumentation in the shared MFC code
(`oneloop_mfc_attitude_run` and `oneloop_mfc_guidance_vert`) and ran
`./sim.sh ANTON_MFC` vs `./sim.sh ANTON_DUAL` headless (NPS auto-takeoff).

## Findings (verified in sim)
- **ANTON_MFC diverges**: `z_meas` runs away to -88 m while `z_sp ≈ -5 m`; motors
  saturate at 9600; FDM NaN at ~t=17 s. `acts[0 0 0 0]` always, `act=0`.
- **ANTON_DUAL is stable**: altitude tracks (`z_sp=-3.977, z_meas=-3.977`); `acts`
  are non-zero and tracking; thrust sane.

**Root reason DUAL is stable:** the dual stabilization wrapper has an RC-really-lost
failsafe (`stabilization_dual_mfc_indi.c`). NPS has no RC, so on tick 1 it forces
`dual_ctrl_active = INDI`. So **DUAL runs INDI stabilization**; MFC is only a passive
shadow, fed real `actuator_state` via `oneloop_mfc_set_shadow_actuator_state()`.
**DUAL never actually runs MFC→MFC in the sim** — the failsafe masks the MFC stack.

Standalone runs the real MFC stabilizer and exposes two bugs the shadow path hid:
1. `oneloop_mfc_stab_active` defaults `false` (shadow). The standalone dispatch never
   set it, and nothing populates `mfc_shadow_act_obs`, so `actuator_state` is stuck at
   0 → broken increment/estimation → runaway.
2. Standalone guidance emitted **physical-float** thrust (`thrust_as_pprz=false`) but
   `oneloop_mfc_attitude_run` reads it with `th_sp_to_thrust_i`. `ThrustSetpoint` is a
   `union { int32_t thrust_i[3]; float thrust_f[3]; }`, so the float bits are read as
   int → garbage objective (`vthz=-552960`). DUAL's MFC shadow never hit this because
   stab=INDI ⇒ `mfc_thrust_pprz=true` ⇒ pprz-int path (the path the attitude code
   actually supports).

## Changes made (oneloop_mfc.c standalone block)
- `oneloop_mfc_stab_active = true` in `stabilization_attitude_enter()`/`run()`.
- `oneloop_mfc_guidance_run(..., true)` (pprz-int thrust) in `guidance_h_run_*`.
- Result: **NaN divergence gone**, NPS runs clean for the full duration.

## STILL NOT FLIGHT-READY (open MFC tuning work)
- MFC vertical guidance (`gz` SISO) saturates (`gz_cmd ≈ -9500`) and does not hold
  altitude — `z_meas` bounces 0–4 m instead of tracking `z_sp`. Needs gz tuning.
- The physical-float thrust path in the attitude run (`th_sp_to_thrust_i` +
  commented-out `thrust_f` line ~768) is still a latent bug if anyone sets
  `thrust_as_pprz=false`.
- Stale airframe comments (GX/GY "disabled" but `enabled=true`; `mfc_thrust_physical
  = -12` hardcode at ~1396).

## Takeaway
To actually validate/tune MFC→MFC you must run with RC present (or disable the
dual failsafe), otherwise DUAL silently falls back to INDI. ANTON_MFC standalone is
the real MFC→MFC test bed.

---

# 2026-06-22 (continued) — MFC parameter bugs fixed + run harness

## Bugs fixed this session (all in `oneloop_mfc.c` + `mfc_core.h/c`)

### 1. gz saturation at ±9600 (wrong clamp range)
`gz_cmd` feeds `thrust_pprz = gz_cmd × THRUST_PPRZ_SCALE` where `SCALE = 1000/(4×-1.5) = -166.67`.
Valid range for gz: `[MAX_PPRZ/SCALE, 0] ≈ [-57.6, 0]` (negative = up in NED).
With the old ±9600 clamp the estimator integrated against a deeply saturated output → anti-windup
never fired → gz_cmd locked at -9500 → full throttle → diverge.

**Fix:** added `u_min`/`u_max` fields to `MfcParameters` (defaulting ±9600 in `mfc_siso_init`),
set `mfc_gz.u_min = MAX_PPRZ/THRUST_PPRZ_SCALE ≈ -57.6` and `mfc_gz.u_max = 0` in guidance init.

### 2. gz reference trajectory blew up (time_trajec=1 in XML, but hardcoded too)
The second-order reference filter has discrete pole at `z = T/(T+1)`. With T=1 the pole is at 0.5
(250 rad/s BW at 500 Hz), so a step input in gz immediately creates a huge reference 2nd derivative
→ `dd_sp_traj = (sp_traj[0] - 2*sp_traj[1] + sp_traj[2]) / dt²` explodes → gz_cmd spike.

With T=250, pole at z=0.996 (~2 rad/s), giving a smooth ramp over ~120 steps.

**Fix:** `mfc_gz.time_trajec = GUIDANCE_MFC_GZ_TIME_TRAJECTORY` (now 250 in XML, not 1).

### 3. oneloop_mfc_init() ignored all XML defines — hardcoded stale values
Roll/pitch: `time_trajec=2` hardcoded (XML=50), `int_window=2` hardcoded (XML=5).
Yaw: `alpha=1.0` hardcoded (XML=73.5294 — 73× mismatch!), `kp=2.0` hardcoded (XML=4.0).
Dead `#define MFC_ATT_ALPHA 147` etc. were never used.

**Fix:** `oneloop_mfc_init()` now reads all params from `STABILIZATION_MFC_*` macros.

### 4. mfc_thrust_physical hardcoded to -12
Used by `accel_to_att_sp()` for horizontal tilt scaling. Was `mfc_thrust_physical = -12` even when
gz was actively computing thrust.

**Fix:** `mfc_thrust_physical = filt_thrust.o[0]` (filtered gz output, seeded at hover on entry).

### 5. gz startup transient: trajectory not pre-seeded
On first entry the trajectory history is zero, so `gz.setpoint_trajec ≈ 0` while `gz.setpoint` is
the current altitude (e.g. -3m NED) → immediate step input → spike.

**Fix:** `oneloop_mfc_guidance_enter()` pre-seeds `setpoint_trajec[0..2] = z_cur` and
`command[0..1] = NOMINAL_HOVER_THROTTLE` on entry.

### 6. Missing `#ifndef` defaults for guidance axis macros
No compile-time defaults for `GUIDANCE_MFC_GX_ALPHA` etc. → build failure or wrong values if XML
section is missing.

**Fix:** added `#ifndef GUIDANCE_MFC_GX/GY/GZ_*` defaults in `oneloop_mfc.c`.

## XML changes (anton_mfc.xml)
| Parameter | Old | New | Reason |
|-----------|-----|-----|--------|
| `GZ_TIME_TRAJECTORY` | 1 | 250 | prevents reference filter explosion |
| `GZ_KP` | 4.0 | 2.0 | slower pole, more damped vertical |
| `GX/GY_TIME_TRAJECTORY` | 1 | 600 | smooth horizontal setpoint ramp |
| `GX/GY_KP` | 0.8 | 0.5 | reduce horizontal aggression |
| `GX/GY_COMMAND_FILTER` | 1 | 10 | smoother horizontal cmd |
| Added `USE_TRAJECTORY_SP`, `DERIVATIVE_GAIN` defines | – | 0/1 | complete init |

## Run harness created
- `tune_mfc.sh` — builds ANTON_MFC nps, runs sim for N seconds (default 30), then calls analyze_mfc.py
- `analyze_mfc.py` — reads `sim_logs/mfc_sim_*.csv`, prints per-axis RMS error, F_k range,
  altitude stats, gz saturation fraction, and tuning hints
- `sim_logs/` — directory where CSVs persist (bind-mounted `/workspace/sim_logs/` in container)
- `sim_anton.py` updated: logs to `/workspace/sim_logs/` (host-visible) not ephemeral `/tmp/`

## Next steps
1. `./tune_mfc.sh` — first end-to-end run with all fixes applied
2. Check `analyze_mfc.py` output: gz saturation should be < 5 %, roll/pitch RMS < 3°
3. If altitude still noisy: tune `GZ_KP` (down) or `GZ_INTEGRATION_WINDOW` (up)
4. If roll/pitch drifts: tune `ROLL_KP`/`PITCH_KP` or check alpha mismatch persists
5. Consider `GZ_NOMINAL_HOVER_THROTTLE`: default -12 ≈ 7 % throttle — likely too low; real
   hover is probably around -17 to -21 (30–37 %). Update once stable hover is confirmed.
