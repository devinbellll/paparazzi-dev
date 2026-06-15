# 2026-06-15 — Yaw oscillation: MFC guidance + INDI stabilization

## Symptom
ANTON_MFC (guidance=`mfc`, stabilization=`indi`) in NPS oscillated in yaw, even
with X/Y guidance disabled, RPY attitude setpoint `{0,0,0}`, and constant thrust.
ANTON (guidance=`indi`, stabilization=`indi`) did not. Same INDI stab module,
same INDI gains.

## The two actual causes

1. **`nav.heading` not initialised on guidance entry.**
   `guidance_mfc` hardcoded the yaw setpoint to `0.0` and never seeded
   `nav.heading`, so the stabilizer was handed a heading reference unrelated to
   the aircraft's actual heading. `guidance_indi` avoids this:
   `guidance_h_run_enter → guidance_indi_enter()` sets
   `nav.heading = stateGetNedToBodyEulers_f()->psi` on entry, and the run path
   uses `gh->sp.heading`.
   **Fix (applied, `guidance_mfc.c`):** set `nav.heading` to current psi in
   `guidance_h_run_enter`, and pass `gh->sp.heading` (not `0.0`) into
   `accel_to_att_sp`.

2. **`PERIODIC_FREQUENCY` mismatch — the dominant cause of the oscillation.**
   `anton_indi_aruco.xml` runs at **1000 Hz**; `anton_mfc.xml` was at **500 Hz**.
   INDI's discrete math (actuator-dynamics model, rate/accel estimates, filter
   sample times) is frequency-dependent, and the **yaw axis is driven by the G2
   spin-up compensation**, which is the most rate-sensitive, lowest-authority
   axis. The INDI gains were proven at 1000 Hz, so running them at 500 Hz made
   yaw limit-cycle while roll/pitch stayed fine.
   **Fix (user handling):** set `<configure name="PERIODIC_FREQUENCY" value="1000"/>`
   in `anton_mfc.xml`. See [[02 - INDI Stabilization Deep Dive]] →
   "PERIODIC_FREQUENCY is part of the INDI tuning".

## Rabbit holes (ruled out — recorded so we don't repeat them)
- Thrust→WLS magnitude/sign: `THRUST_PPRZ_SCALE × ΣBwls[3] = 1.0`, round-trips
  correctly. Not it.
- WLS priorities `{1000,1000,1,100}`: user tested pseudo-inverse too — still
  oscillated. Not it.
- `gz` filter / `time_trajectory` changes: not it.
- Absolute vs incremental thrust setpoint (`THRUST_SP` vs `THRUST_INCR_SP`):
  plausible, but ANTON forced to `{0,0,0}` attitude with its normal thrust only
  oscillated *briefly then settled* — so the setpoint/thrust path was not the
  cause. (User wants to keep the non-incremental absolute thrust regardless.)
- "Commanding flat absolute attitude while horizontal is uncontrolled couples
  into yaw": disproven by the same `{0,0,0}`-on-ANTON test.

## Key method that cracked it
Diffing the two airframe XMLs end-to-end (`anton_indi_aruco.xml` vs
`anton_mfc.xml`) surfaced the `PERIODIC_FREQUENCY` 1000→500 difference once the
code-path theories were exhausted.

## Status
- `guidance_mfc.c` heading fix: applied.
- `PERIODIC_FREQUENCY` → 1000: user applying.
- Verify by rebuilding ANTON_MFC for NPS and confirming yaw holds without
  oscillation.
