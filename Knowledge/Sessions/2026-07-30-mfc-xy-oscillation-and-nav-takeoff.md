# Session: 2026-07-30 — MFC X/Y Oscillation + Headless Nav Takeoff

## What was asked
User reported ANTON_MFC's guidance stack oscillates over X/Y (stable only at very low gains,
"maybe a sign flip or a scale issue") and that `./sim.sh --fg ANTON_MFC --nav` launches but the
aircraft never takes off (suspected the vto_survey flight-plan switch).

## What changed

`sw/airborne/firmwares/rotorcraft/guidance/guidance_mfc.c` (`guidance_mfc_init`): fixed a
copy-paste bug — `mfc_gx.use_trajec_sp` and `mfc_gz.use_trajec_sp` were both reading
`GUIDANCE_MFC_GY_USE_TRAJECTORY_SP` instead of their own per-axis defines. Currently a no-op
(all default `true`), but wrong the moment someone overrides one axis differently.

`sim_anton.py`: added a debug Ivy subscription that prints the aircraft's own
`ROTORCRAFT_NAV_STATUS` downlink (block/stage/timing) whenever one arrives — used to diagnose
the takeoff issue, left in since it's harmless and useful for next time.

## What was found but NOT fixed (needs the user's input / further work)

1. **`accel_to_att_sp()` has a real scale bug** (`guidance_mfc.c`): the tilt-scaling denominator
   `mfc_thrust_physical` is hardcoded to `GUIDANCE_MFC_GZ_NOMINAL_HOVER_THROTTLE` instead of the
   actual filtered thrust — `filt_thrust.o[0]` is computed right above it and then thrown away
   (there's a literal `// TODO: Figure out if worth not using constant`). This means the
   horizontal accel→tilt conversion is always scaled as if hovering at exactly nominal weight,
   even while climbing/descending or under disturbance — a live scaling error, matching the
   "scale issue" the user suspected. Left unfixed because `git log` shows this constant
   predates the current regression (it was already there, just a different literal, when X/Y
   was reportedly tuned "pretty good" a few commits back) — so it's a pre-existing simplification,
   not obviously the new-regression cause, and flipping it changes closed-loop dynamics that
   should be verified in flight/sim, not blind.

2. **Could not confirm or rule out a phi/theta sign error.** Worked through the trig in
   `accel_to_att_sp()` (R_psi_T rotation + `float_quat_of_eulers_yxz` convention) by hand;
   it looks self-consistent (phi driven by the lateral term, corrected-second theta driven by
   the longitudinal term, signs match the expected physical response). But this is unverified
   in practice — the aircraft never left the ground in any test run this session (see below), so
   no real closed-loop X/Y data was available to check against.

3. **The aircraft never takes off in NPS regardless of flight plan.** This is the session's
   main finding — see bug-227 in `.wolf/buglog.json` for full detail. Short version: NPS-scope
   telemetry across 3 full runs shows `autopilot.mode` pinned at `AP_MODE_NAV`, but
   `nav.vertical_mode` pinned at `MANUAL` and `motors_on`/`arming`/`in_flight` pinned at 0 the
   *entire* run — the flight plan never advances past its first block ("Wait GPS", which calls
   `NavKillThrottle()`). It is **not** the vto_survey flight plan (block names/exceptions check
   out) and **not** `modules/checks/preflight_checks` (not compiled into ANTON_MFC at all).
   Adding a debug listener for the aircraft's own periodic `ROTORCRAFT_NAV_STATUS` message
   showed **zero** messages received in 40s — Ivy bus traffic isn't flowing between
   `sim_anton.py` and the firmware/server in this sandboxed Docker environment, in either
   direction. This looks like an Ivy peer-to-peer TCP callback failure under the nested
   sandbox networking, not a firmware/flight-plan bug. Not yet root-caused at the network
   level — next step would be checking (via tcpdump/strace inside the container) whether Ivy's
   post-UDP-discovery TCP handshake actually completes, or trying `--rc_script N` as a control
   (RC-driven auto-arm bypasses Ivy ground commands entirely) to see if the aircraft can fly at
   all once armed via a different path.

## Why X/Y oscillation is still unresolved
The scale bug (#1) is a legitimate, evidence-backed lead but unverified in flight. The sign
question (#2) is unverified either way. Real progress on the oscillation needs a working
takeoff first (#3) so telemetry from actual X/Y flight can be captured and checked against the
`MFC_ACC2ATT/*` and `MFC_GUIDANCE/*` NPS-scope vars (already registered, see
`guidance_mfc.c:206-235`).
