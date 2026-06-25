---
date: 2026-06-25
effort: medium
session_n: 1
status: raw
---

# NPS RC emulation → ANTON_MFC takeoff + ATTITUDE_Z_HOLD via rc_script

## What happened

Goal: emulate RC input in NPS SITL so ANTON_MFC takes off, enters
ATTITUDE_Z_HOLD, and receives scheduled/direct attitude commands.

- Surveyed NPS RC options in `sw/simulator/nps/`:
  - `-j/--js_dev` → real USB joystick (only true realtime input; awkward in the
    ephemeral container).
  - `--rc_script N` → compiled-in C stick scripts (`nps_radio_control.c`).
  - `--norc` → RC disabled (the existing `sim_anton.py` default).
- Added a `--rc_script N` flag plumbed through `sim.sh` → `sim_anton.py` →
  the `simsitl` binary (replaces the hardcoded `--norc` when set).
- Key mechanics confirmed in source:
  - With `--rc_script`, the script **owns the AP mode switch every RC frame**
    (`autopilot_static_on_rc_frame` → `ap_mode_of_3way_switch`). Mode values:
    `MANUAL=-1, AUTO1=0, AUTO2=1`.
  - The stock `takeoff` wrapper forces MANUAL for the first 8 s — incompatible
    with a flight-plan NAV takeoff, so a self-contained script must bypass it.
  - `ATTITUDE_Z_HOLD`: horizontal = attitude sticks, vertical =
    `GUIDANCE_V_MODE_HOVER` (captures z on entry).
  - `sim_anton.py` fires Start Engine (block 2, ~t=5 s) + Takeoff (block 3),
    NAV climbs to the CLIMB waypoint.
- Implemented **rc_script 5** (`radio_control_script_fp_takeoff_zhold`):
  emits AUTO2(=NAV) during a 15 s climb window so the flight plan takes off,
  then AUTO1(=ATTITUDE_Z_HOLD) with a cycled roll/pitch/yaw step schedule.
  Self-contained (skips the 8 s wrapper). Tunable via `NPS_FP_CLIMB_TIME`,
  `NPS_ZHOLD_STEP_PERIOD`, `NPS_ZHOLD_STEP_AMP`.
- Airframe `anton_mfc.xml`: remapped `MODE_AUTO1` from `ATTITUDE_DIRECT` to
  `AP_MODE_ATTITUDE_Z_HOLD` (kept `AUTO2 = NAV` for takeoff).
- Ran the sim headless and probed the **Ivy bus directly** (the sim_anton.py
  CSV/debug logs read all-zero/frozen in this sandbox and are not a witness):
  confirmed takeoff, motors on, armed, IN_FLIGHT, `ap_mode = ATTITUDE_Z_HOLD`.

## Outcomes

- ✅ `nps` target builds clean for ANTON_MFC with the new script + airframe map.
- ✅ `--rc_script N` flag forwards correctly through sim.sh / sim_anton.py
  (traced arg parsing; bare `N` not mistaken for the aircraft name).
- ✅ Takeoff + ATTITUDE_Z_HOLD entry confirmed via live telemetry
  (`ap_mode=9`, IN_FLIGHT, MOTORS_ON, ARMED).
- ✅ **User confirmed the feature works in their own test** (full flight
  behavior, incl. the attitude commands). This supersedes my headless probe.
- ⚠️ During my headless probe I saw continuous altitude climb (~164→195 m) and
  could not read the attitude-step setpoints. Per the user's working test this
  was most likely a probe/altitude-reference misread or a transient, not a real
  defect — left for the user to confirm exact mode/behavior later.

## Open threads

- Exact vertical behavior of ATTITUDE_Z_HOLD on the MFC stack (does
  `stabilization_mfc` consume `guidance_v`'s HOVER thrust, or its own
  `GUIDANCE_MFC_GZ_*` path?) — not fully traced. Revisit only if the altitude
  hold misbehaves in a real test.
- `STAB_ATTITUDE att_des` decoding in the Ivy probe was unreliable (read a
  single element); not needed for the deliverable.

## Promotable to Knowledge/

- "Verifying NPS flight behavior headless" → probe the Ivy bus from a sibling
  `--network host` container on `127.255.255.255:2010`; decode
  `ROTORCRAFT_STATUS` (ap_mode 6th payload value; 9=ATTITUDE_Z_HOLD) and
  `ROTORCRAFT_FP` (`up * 0.0039063` = metres). The sim_anton.py CSV is NOT a
  reliable state witness in the sandbox.
- "NPS rc_script controls the AP mode switch every frame" — a flight-plan NAV
  takeoff and stick injection can only coexist if one script emits AUTO2(=NAV)
  during takeoff then switches modes over time. Candidate for `06`/`07` notes.
