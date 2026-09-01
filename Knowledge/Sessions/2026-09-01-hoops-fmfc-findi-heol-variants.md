---
date: 2026-09-01
topic: Hoops_111 FMFC / FINDI / HEOL airframe variants
tags: [session, airframe, hoops, findi, fmfc, heol, nps]
---

# Hoops_111_FMFC / Hoops_111_FINDI / Hoops_111_HEOL

## Goal

Three new Hoops_111 control-stack variants, each buildable for `ap` and `nps`
and registered equally in `conf_mfc.xml` + `control_panel_mfc.xml`, mirroring
the existing `Hoops_111_MFC`.

## What was created

| file | ac_id | stack |
|---|---|---|
| `conf/airframes/ENAC/quadrotor/hoops_111_fmfc.xml`  | 178 | `oneloop_fmfc` (STABILIZATION_FMFC + GUIDANCE_FMFC) |
| `conf/airframes/ENAC/quadrotor/hoops_111_findi.xml` | 179 | `oneloop_findi` (STABILIZATION_FINDI + GUIDANCE_FINDI) |
| `conf/airframes/ENAC/quadrotor/hoops_111_heol.xml`  | 180 | `stabilization type=heol` + `guidance type=heol` (STABILIZATION_HEOL + GUIDANCE_HEOL) |

Plus:
- `conf/userconf/ENAC/conf_mfc.xml` — three `<aircraft>` entries, telemetry
  `mfc_flight_test.xml`, flight plan `flat_traj_demo.xml`, `settings/rotorcraft_basic.xml`,
  per-variant `settings_modules` list (common set + the control module(s)).
- `conf/userconf/ENAC/control_panel_mfc.xml` — per variant a `<X> VM SIM` session
  (twin of `MFC VM SIM`: joystick + simulator with `--scope_*` + datalink + server
  + PprzGCS + two `pj_json_relay.py` hops) and a `Flight Hoops_111 <X>` session
  (twin of `Flight Hoops_111 MFC`: XBee datalink + server + PprzGCS + NatNet3 on
  rigid body 111).

## Method

Cloned `hoops_111_mfc.xml` for the hardware profile (TawakiV2 board, DShot
4-in-1 servos, `ACCEL/GYRO/MAG_CALIB` arrays, `gps optitrack`+`ins ext_pose` on
ap / `ins ekf2`+`air_data`+`gps ublox` on nps, `MODEL` 0.8 / 0.0068 / 0.0068 /
0.0136 kg·m², 3S `BAT`, `NPS JSBSIM_MODEL simple_x_quad_ccw`, `actuators_pprz[]`
command_laws), then spliced in each control stack from the matching
`anton_<X>.xml`.

Key decisions:
- **G1 / G2 / ACT_FREQ stay at the Hoops_111_MFC values** ({±14, ±0.9, −0.7},
  G2 {80,−80,…}, ACT_FREQ 15) — they are identified against `simple_x_quad_ccw`,
  the NPS plant these fly, not against `anton`. Control gains, estimator windows
  and filters carry over from ANTON verbatim; the Hoops MODEL inertias equal
  ANTON's so `alpha = 2./MODEL_INERTIA_*` transfers unchanged.
- **Dropped the ANTON `STABILIZATION_<X>_COMMANDS` define.** `stabilization_heol.c`,
  `oneloop_findi.c`, `oneloop_fmfc.c` all write `actuators_pprz[i]` unconditionally;
  the `cmd[act_to_commands[i]]` remap is `#ifdef`-guarded and only needed for the
  `<commands><axis name="FR">` mixing style ANTON uses.
- **Dropped `logger_mfc_csv`** — it hard-`#include`s `stabilization_mfc.h`, so it
  cannot link without the MFC stabilizer. Kept `flight_recorder` (binary) +
  `nps_scope_state`.
- HEOL `GZ_MAX_THRUST` uses the Hoops 0.7 thrust-row factor, not ANTON's 1.5.

## Build

All six targets compile clean (arm64 ephemeral container):

```
./pprz.sh build Hoops_111_FINDI nps   # + ap
./pprz.sh build Hoops_111_FMFC  nps   # + ap
./pprz.sh build Hoops_111_HEOL  nps   # + ap
```

ap `.elf` sizes: FINDI 1 343 188 B text, FMFC 1 347 508, HEOL 1 357 072.

## Sim smoke test — and a pre-existing harness quirk

`timeout 55 ./sim.sh Hoops_111_FINDI --no-build --nav "Start Engine,Takeoff,+15,Standby"`
loads JSBSim, streams the scope, and steps through nav blocks
`Start Engine → Takeoff → Standby` cleanly (25 933 CSV rows, clean shutdown), but
`/uav/MODE/motors_on` never leaves 0 and `TRUTH/alt` never changes — it does not
take off.

**This reproduces identically on the flight-proven `Hoops_111_MFC`** with the
same command, so it is a sim-harness GPS-fix / arming-timing issue, not a defect
in the new airframes. Left as-is; the compile is the automated correctness check
per CLAUDE.md, and functional correctness needs flight test / a working sim
arming path regardless.

## Next

- Fix or document the headless `sim.sh` arming path (motors never arm under
  `--nav "Start Engine,…"` — affects every Hoops_111_* aircraft).
- Once arming works: run the isolation ladder per variant, no-noise/with-noise
  pairs, and retune the carried-over ANTON gains for the `simple_x_quad_ccw`
  plant (all three carry UNTUNED-placeholder warnings).
- Consider `contract/signals.json` branches `SITL_6DOF_HOOPS_111_{FINDI,FMFC,HEOL}`
  if these enter the cross-source comparison (structural twins of the ANTON ones).
