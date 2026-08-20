# 2026-08-20 — DarkO airframe port: build + SITL takeoff

## Task
Build and SITL-test the freshly ported DarkO tailsitter airframe
(`port-darko-airframe` branch, paparazzi submodule). Port was already done;
job was to make it build and take off, not redesign it. No MFC, no new
control code, no gain retuning.

## What changed (both are port fixes — renamed/misused symbols, not values)

1. **`conf/airframes/ENAC/hybrid/darko.xml` — IMU calibration define name.**
   Section `IMU` has `prefix="IMU_"`, and the define was named
   `IMU_ACCEL_CALIB`, which doubles to `IMU_IMU_ACCEL_CALIB` — `imu.c` has a
   hard `#error` guard for exactly this. Renamed to `ACCEL_CALIB` (matches
   `hoops_111_mfc.xml`'s convention). This was a build-blocking error.

2. **`conf/airframes/ENAC/hybrid/darko.xml` — two extra "dummy" DShot servos.**
   The fork's DShot block had `RM, LM, dummy1, dummy2` (4 entries) alongside
   2 Pwm elevon servos → `ACTUATORS_NB=6`. Every other DShot airframe in this
   tree (`anton_*`, `cobra*`, `hoops_*`, `maya_*`, `goose`, `robobee`,
   `falcon_v*`, `crow_indoor`, `hexa_tilted_motors`, `niquad_wind`) has
   *exactly* the real motor count in its DShot block — never padded. NPS's
   `nps_autopilot.h` falls back to `NPS_COMMANDS_NB = ACTUATORS_NB` when
   `NPS_NO_MOTOR_MIXING` is set without `NPS_USE_COMMANDS` (Darko's case), and
   `nps_autopilot_rotorcraft.c`'s command-scaling loop indexes
   `actuators_pprz[i]` for `i < NPS_COMMANDS_NB`. But `actuators_pprz[]` is
   sized `INDI_NUM_ACT+1 = 5` (`stabilization_indi.c`). With `ACTUATORS_NB=6`
   the loop read `actuators_pprz[5]` — one past the array — and simsitl
   **SIGSEGV'd right after JSBSim FDM init**, before any scope telemetry.
   The build itself even warned about it: `iteration 5 invokes undefined
   behavior [-Waggressive-loop-optimizations]` in
   `nps_autopilot_rotorcraft.c`. Removed the two dummy servos; `ACTUATORS_NB`
   is now 4, matching `INDI_NUM_ACT`.

   **Lesson for next port:** an `-Waggressive-loop-optimizations` /
   "iteration N invokes undefined behavior" warning at NPS link time is not
   noise — it is very likely a real out-of-bounds actuator array read tied to
   `ACTUATORS_NB` vs `INDI_NUM_ACT` mismatch. Chase it before flying.

## Build result
`./pprz.sh build Darko nps` — clean, links. No other symbol issues surfaced
(the two PORT NOTE items already fixed before this session —
`WLS_N_U/V`→`WLS_N_U_MAX/V_MAX`, `STABILIZATION_INDI_HYBRID` removal — held up
fine).

## SITL result: it flies, but pitch has a sustained limit-cycle oscillation

`./sim.sh Darko --no-build --nav "Start Engine,Takeoff,+15,Standby"`: takes
off, reaches Standby (AP_MODE_NAV, `ap_mode=13`), and stays airborne
indefinitely (ran 45+ sim-seconds with no crash).

Verified via a direct Ivy probe (`ROTORCRAFT_FP`/`ROTORCRAFT_STATUS`) run in a
sibling container against the sim's Ivy bus, **not** via `sim_anton.py`'s own
CSV/debug-log capture — those are known-unreliable in this sandbox (see
`Do-Not-Repeat` in cerebrum, 2026-06-25 entry). CSV file for the crashed run
was never created at all (0 packets ever reached the scope feed before the
SIGSEGV), which was itself a useful crash signal.

Steady-state telemetry (25 s window, well after Standby entry):
- `up` (altitude): dead steady at **6.86 m**, ±0.01 m — altitude hold is solid.
- `phi` (roll): small, ±3°, no obvious problem.
- `theta` (pitch): **clean, bounded sinusoid at ~1.2 Hz, amplitude ~±35°**,
  constant over the full window (not growing → not diverging in the sense of
  the task's bar, but very much not a settled hover).
- `psi` (yaw): slow continuous drift (~1.9°/s), not oscillating, not held.

This pattern — sustained fixed-amplitude bang-bang-like oscillation on one
axis while altitude locks in tight and roll stays calm — is textbook for a
closed-loop sign inversion (the controller pushes the wrong way, saturates,
error swings past through zero the other way, saturates again, repeat at a
frequency set by actuator rate limits) rather than a badly-tuned-but-correctly-
signed loop (which would usually show either slow drift or an ever-growing
oscillation, not a flat-amplitude limit cycle from the first cycle).

**Not resolved this session.** The task's own debug order (rung 3) calls for
exactly this next step — command pure roll/pitch/yaw separately via RC and
confirm response direction — but the stock NPS `--rc_script step_pitch`
tooling maps its stick input through `MODE_SWITCH_AUTO2`, which is
`AP_MODE_NAV` for Darko (not a direct-attitude mode), so it doesn't exercise
the attitude loop the way it does for airframes where AUTO2 is
ATTITUDE_DIRECT/Z_HOLD. Isolating the sign cleanly needs either a Guided-mode
Ivy stimulus or a script/mode-map change, both out of scope for this session
(no `sw/airborne`/`sw/simulator` edits without a proven firmware bug, no gain
retuning to paper over the symptom).

**Leading suspects for next session**, none confirmed:
- `G1_PITCH = {-4., 4.3, 0, 0}` and `G1_YAW = {-3.9, -3.8, 0, 0}` in
  `STABILIZATION_ATTITUDE_INDI` — the opposite-sign pitch pair vs. same-sign
  yaw pair is *consistent* with the elevons' mirrored servo travel (comments
  "min Vers le haut" / "min Vers le bas" on `ELEVON_LEFT`/`ELEVON_RIGHT`), so
  static inspection doesn't convict it — needs the pure-pitch-stick test.
- `guidance_indi_hybrid_tailsitter`'s hover-attitude convention vs. what the
  `cyclone` JSBSim model reports for `theta` — TRANSITION_MAX_OFFSET=-75°
  implies theta=0 is hover in this airframe's own convention, which the
  oscillation is centered on, so this looks less likely but wasn't ruled out.

## Report bar
Per the task's literal bar ("a SITL takeoff that does not diverge") this
technically clears it — bounded, non-growing oscillation, solid altitude,
no crash, sustained 45+ s. But it is not a clean stable hover and I said so
plainly rather than rounding up to "it flies fine."
