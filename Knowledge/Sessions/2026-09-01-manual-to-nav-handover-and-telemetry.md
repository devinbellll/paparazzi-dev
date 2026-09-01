# 2026-09-01 — manual→NAV handover drop, unused-var warning, GCS "IMU UNKNOWN"

Three defects on the ANTON_FMFC / Hoops_111_MFC manual→NAV flight-test workflow.
Updated after the user flight-tested the first round of fixes in the full GCS+RC.

## 1. Airborne manual→NAV drop  (bug-312)

### Bare switch → "drops like a rock"  — EXPECTED, not a regression

`AP_MODE_NAV` executes the **current flight-plan block**, not a position hold.
After a hand launch the plan is parked in `Holding point` / `Start Engine`, whose
stage is `<attitude throttle="0">`. Switching to NAV there commands zero throttle.
Every Paparazzi rotorcraft does this; nothing the flat-traj work touched. A
position-hold block (`Hold Here`, `Standby`) **must be the active block before the
switch**. Confirmed by the user: same behaviour on stock-INDI Hoops.

### `Hold Here` first, then AUTO2 → holds

- **Vertical** now holds. `nav_set_altitude()` in `navigation.c` only copies
  `nav.fp_altitude` → `nav.nav_altitude` (what guidance_v flies) on a >0.2 m
  change vs a persistent static, so a gradual manual climb after selecting the
  block never propagated and NAV dived to the stale value. Fix (bug-307): the
  `pre_call` writes `nav.fp_altitude` **and** `nav.nav_altitude` directly every
  non-NAV tick. User-confirmed fixed.
- **Horizontal transient when moving.** The drag is exact (waypoints are local →
  `waypoint_set_xy_i`, no LLA round-trip), so the setpoint = current position at
  handover. But `guidance_h_nav_enter()` →
  `reset_guidance_reference_from_current_position()` seeds `guidance_h.ref.speed`
  with the current NED velocity while the setpoint is the stationary carrot, so
  the reference model **coasts forward** from the entry velocity (bounded by
  `GUIDANCE_H_REF_MAX_ACCEL`), overshoots by ≈ v²/(2·a_max), then returns.
  Reads as a "setpoint jump" because the WP_STDBY marker freezes the instant
  `mode==NAV` (drag stops) while the aircraft coasts past it.
  - **Fix shipped:** the `pre_call` now places WP_STDBY at a velocity-projected
    capture point — `pos + v·|v|/(2·HOLD_HERE_STOP_ACCEL)` (ENU, default
    4 m/s², a `<header>` `#define`) — via `waypoint_set_xy_i`, so a moving
    handover coasts *to* the hold point instead of past it. Hand-hover handover
    (v≈0) unchanged. Built ANTON_FMFC + Hoops_111_MFC nps clean; still needs a
    flight-test to tune `HOLD_HERE_STOP_ACCEL`.
  - Gotcha hit: an XML `pre_call=` cannot contain `&` (even `&amp;` reaches C
    literally), so `waypoint_set_xy_i` (by-value) not `waypoint_set_enu_i`
    (needs `&struct`).

### `Standby` vs `Hold Here`

`Standby` sends the aircraft to WP_STDBY's *original* (0,0,2) because it doesn't
drag the waypoint. `Hold Here` holds near the current position (drag works). Same
handover dynamics otherwise.

### FMFC specifics

The FMFC oneloop's guidance hooks are placeholders (`guidance_v_run_pos` returns
zero thrust); real thrust only when `guidance_driving` (`fmfc_linear_enabled &&
fmfc_guidance_fresh>0 && fmfc_gh && fmfc_gv`). For a `<stay>` block that path is
active (ALT vertical mode → `guidance_v_run_pos` latches). So the drop there was
the stale-altitude one, now fixed.

## 2. `nav_hold_alt` unused-variable warning  (bug-308)

`static float nav_hold_alt` in the FP `<header>` → emitted into `flight_plan.h`,
included by every module TU, used only by `auto_nav()`. Fixed with `UNUSED`.
Confirmed `tag_tracking.o` rebuilds clean.

## 3. GCS "IMU UNKNOWN" (4th strip icon red) on clean/mfc telemetry  (bug-311)

The icon is driven by **`STATE_FILTER_STATUS.state_filter_mode`**
(`UNKNOWN|INIT|ALIGN|OK|GPS_LOST|IMU_LOST|…`); the server leaves it at 0 =
`UNKNOWN` until the message arrives. `ins_ekf2.cpp:642` registers it on
`DefaultPeriodic`. `clean` and `mfc` never listed it.

**Fix:** added `STATE_FILTER_STATUS` (2.2 s) to `clean` and `mfc`. Also kept
`INS_REF` (5.1 s) + `GPS_INT` (1 s) on both and `ROTORCRAFT_FP` (0.25 s) on
`clean` — the server only builds its `nav_ref` from `INS_REF`, without which
`ROTORCRAFT_FP` is dropped and the aircraft never places on the map. Built
ANTON_FMFC nps OK.

## Build / test

- `CONF=conf/userconf/ENAC/conf_mfc.xml ./pprz.sh build ANTON_FMFC nps` — clean.
- NPS cannot reproduce the manual→NAV handover: `--norc` → ANTON_FMFC never arms
  (datalink RC), aircraft stays at the origin. Needs the full GCS+RC, or
  `--rc_script N`.

## Next

- Decide on the velocity-projected capture point for `Hold Here` and flight-test.
- Consider an upstream `nav_set_altitude_now()` helper instead of poking `nav.*`.
