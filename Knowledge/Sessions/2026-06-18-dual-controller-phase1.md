# Session — 2026-06-18: Dual-Controller Phase 1 (shadow telemetry + fidelity + flight build)

## Goal
Continue from Phase 0 (MFC+INDI co-compile, INDI active / MFC inert shadow).
Finish the Phase 1 increments from the Phase 0 session note: DUAL_CTRL
comparison telemetry, init smoke-test, and wire the shadow MFC to the real
operating point.

## What changed
- **NEW telemetry message `DUAL_CTRL` (id 194)** in
  `sw/ext/pprzlink/message_definitions/v1.0/messages.xml`: `active` (uint8) +
  `cmd_act[]` (committed/active) + `cmd_shd[]` (shadow MFC) + `resid[]`
  (shd-act). 194 was a free gap in the (nearly full, 249/255) telemetry id
  space. Required regenerating the pprzlink headers with
  `make -C paparazzi pprzlink_protocol` — `pprz.sh build` does NOT do this
  (bug-071).
- **control_dual_mfc_indi.c** — always-on `dual_committed_cmd[]` mirror
  (telemetry runs async from the control tick); `send_dual_ctrl` +
  registration in `stabilization_dual_init`; per-tick hand-off of INDI's real
  `actuator_state[]` to the shadow MFC via
  `stabilization_mfc_set_shadow_actuator_state()`; `_Static_assert
  INDI_NUM_ACT==MFC_NUM_ACT` guarding the index-by-index actuator copy.
- **conf/telemetry/default_rotorcraft.xml** — stream `DUAL_CTRL` at 10 Hz
  (ANTON_MFC's telemetry file). A registered msg only transmits if listed here.
- **stabilization_mfc.c/.h** — gate MFC's duplicate shared-message telemetry
  (EFF_MAT_STAB, STAB_ATTITUDE, WLS_*) behind `#ifndef STABILIZATION_MFC_SHADOW`
  so the active law (INDI) owns them and they aren't double-sent; keep the
  unique STAB_MFC. New `stabilization_mfc_set_shadow_actuator_state()` + a
  shadow branch in `get_actuator_state()` that COPIES the real operating point
  (like the RPM-feedback path) instead of integrating the discarded `mfc_u`.
- **stabilization_indi.h** — export `actuator_state[INDI_NUM_ACT]` (already a
  global symbol) so the wrapper can read the active law's operating point.
- **stabilization_dual_mfc_indi.xml** — `<define name="STABILIZATION_MFC_SHADOW"/>`.
- **Fixed bug-050 + bug-052** (pre-existing SITL-guard leaks blocking the
  flight build): `#ifdef SITL`-guarded `nps_th_cmd_z`/`nps_v_thrust`
  (stabilization_indi.c:731/751) and `nps_scope_z_sp`/`z_ref`
  (guidance_indi.c:523-524). ANTON_MFC now builds BOTH `nps` and `ap`.

## What I learned (see cerebrum / buglog 069-071)
- New pprzlink message ⇒ regen `var/include/pprzlink/` via `make pprzlink_protocol`.
- Telemetry id space is uint8 and nearly full; free gaps 7,13,51,57,194,195.
- `register_periodic_telemetry` allows multiple callbacks per id ⇒ double-send,
  not a crash; gate the inactive law's shared messages.
- Incremental-controller shadow fidelity needs the REAL operating point fed in
  each tick, else the shadow drifts about its own discarded commands.
- bug-050/052 are the same SITL-guard class and sit on the Phase-1→flight path.

## Verification
- `./pprz.sh build ANTON_MFC nps` and `./pprz.sh build ANTON_MFC ap` both
  produce binaries (ap.elf NOW links — first time for the dual stack).
- `nm` on simsitl: `send_dual_ctrl`, `stabilization_dual_init`,
  `stabilization_mfc_set_shadow_actuator_state` present; `actuator_state`
  appears as both `B` (INDI global) and `b` (MFC static) — no collision.
- 35 s NPS smoke run: JSBSim loads, 134 scope vars, sim steps at dt 5e-4,
  Ivy broadcasting; no assert / NaN / fault / double-registration. Behaviour
  unchanged pure-INDI by design (MFC output still discarded).
- Compile is the only automated check (CLAUDE.md); the agreement-band
  validation itself needs a human reading DUAL_CTRL/PlotJuggler in sim+flight.

## Next
- **Validate the agreement band**: fly ANTON_MFC in NPS, watch DUAL_CTRL
  `resid[]` (and dual/resid in PlotJuggler). Expect small, bounded residual in
  hover; characterise transients. Then bench/flight on real hardware (ap now
  builds).
- **Shadow integrator hygiene**: the shadow MFC's SISO integrators/adaptive
  estimator still accumulate on never-committed error; `stabilization_attitude_enter`
  re-enters both on mode change, but consider periodic shadow reset if windup
  shows in `resid` during long shadow runs.
- (Optional) guidance-level shadow (MFC guidance alongside INDI guidance).
- **Phase 2 — Handover (Option A):** promote `dual_ctrl_active` to a runtime
  selector — `dual_ctrl_set_active()` re-`enter()`s the newly active law for a
  bumpless swap; GCS dropdown + RC AUXn gate; failsafe forces INDI. The wrapper
  already routes only the active law's command to `cmd[]`, so the selector is a
  branch on which core writes the real buffer vs the shadow buffer.
