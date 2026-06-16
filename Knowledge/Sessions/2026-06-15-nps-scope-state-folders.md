# Session — 2026-06-15: NPS scope est/sensors/setpoints/modes folders

## Goal
Enrich the NPS scope (in-process JSON/UDP → PlotJuggler) so truth can be compared
against the firmware estimate, sensors, setpoints, and flight modes.

## What changed
1. **Truth NED position** — added `truth/x|y|z` (from `fdm.ltpprz_pos`) to the
   hardcoded truth block in `sw/simulator/nps/nps_scope.c`, and bumped the datagram
   buffer `4096 → 8192` (more registered vars now share the line).
2. **New rotorcraft sim-only module** `sw/airborne/modules/nps_scope/nps_scope_state.c/.h`
   + `conf/modules/nps_scope_state.xml`:
   - `nps_scope_state_periodic()` (module `<periodic freq=1000 autorun>`) refreshes a
     static float mirror every control step, in SI / deg:
     - `est/*` — firmware estimate via `stateGet{Position,Speed,Accel}Ned_f`,
       `stateGetNedToBodyEulers_f`, `stateGetBodyRates_f`.
     - `sensors/*` — NPS simulated `sensors.{accel,gyro,mag,gps}` (noisy inputs).
     - `sp/nav/*` — `nav.{target,carrot,speed,heading,climb,nav_altitude}` (ENU→NED).
     - `sp/guidance/*` — `guidance_h.sp/ref.pos` + heading, `guidance_v.z_sp/zd_sp`
       (fixed-point → SI via `POS/SPEED_FLOAT_OF_BFP`).
     - `sp/stab/*` — `stab_sp_to_eulers_f/rates_f(&stabilization.sp)` + `stabilization.cmd[]`.
     - `mode/*` — `autopilot.{mode,arming_status,motors_on,in_flight}`,
       `guidance_h.mode`, `guidance_v.mode`, `stabilization.mode`,
       `nav.{horizontal,vertical}_mode` (registered directly as step signals).
   - Registration uses the existing `NPS_SCOPE_VAR`/`NPS_SCOPE_VARN` constructor macros
     (no-op off-sim); `nps_scope.c` needed no hook API.
3. **Fleet wiring** — added `<module name="nps_scope_state"/>` inside the `nps` target
   block of all ENAC rotorcraft airframes: ANTON, MAYA, CROW, GOOSE, CobraV2, RoBoBee
   (quadrotor) and CYFOAM, FALCON_V2 (hybrid, rotorcraft fw).

## Verification
- `./build_fw.sh ANTON ... nps` → `nps.elf` ✓
- `./build_fw.sh CYFOAM ... nps` → `nps.elf` ✓ (`nps_scope_state.o` compiles)
- RoBoBee: `nps_scope_state.o` compiles, but the build then fails on `Python.h`
  (pybullet FDM) — a container env limitation, unrelated to this work.

## Found (pre-existing, NOT this work) — logged as bug-050
`ap` (hardware) target fails: `nps_v_thrust` is `#ifdef SITL`-only (stabilization_indi.c
~318) but assigned unconditionally (~751). From commit cf6041886 (yaw debug). Fix:
guard line 751 with `#ifdef SITL`. Flagged to user; left untouched.

## Learnings
- Module source must live under `sw/airborne/modules/<dir>/`; a `dir="firmwares/rotorcraft"`
  on the `<module>` did not resolve (`No rule to make target …`). Moved file to
  `modules/nps_scope/`.
- A module with a generated `<periodic>` call needs a `<header>` element so its
  prototype lands in `modules.h` (otherwise implicit-declaration warning).

## Next
- Run NPS + PlotJuggler, confirm the `truth/ est/ sensors/ sp/ mode/` tree and that
  mode steps line up with setpoint/estimate jumps.
- Optional later: EKF2 innovations (`INS_EKF2`) into an `ekf/` folder.
