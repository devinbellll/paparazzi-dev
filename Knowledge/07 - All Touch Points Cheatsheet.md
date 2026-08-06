# All Touch Points Cheatsheet

Quick-reference for every file that must change per type of modification.  
`*` = always required. `?` = only if needed.

---

## Tune Gains Only (no recompile if using GCS live)

| File | Change |
|------|--------|
| `conf/airframes/ENAC/quadrotor/anton_indi_aruco.xml` * | `REF_ERR_*`, `REF_RATE_*` in `STABILIZATION_ATTITUDE_INDI` section |

---

## Change G1/G2 (Control Effectiveness)

| File | Change |
|------|--------|
| `conf/airframes/ENAC/quadrotor/anton_indi_aruco.xml` * | `G1` matrix and `G2` vector |
| *(no C file changes needed)* | G1/G2 are loaded from `generated/airframe.h` at compile time |

---

## Change Actuator Model (ACT_FREQ)

| File | Change |
|------|--------|
| `conf/airframes/ENAC/quadrotor/anton_indi_aruco.xml` * | `ACT_FREQ` in `STABILIZATION_ATTITUDE_INDI` |
| `conf/airframes/ENAC/quadrotor/anton_indi_aruco.xml` * | `THRUST_DYNAMICS_FREQ` in `GUIDANCE_INDI` (keep in sync) |

---

## Change Stabilization Algorithm (INDI → PID or new)

| File | Change |
|------|--------|
| `conf/airframes/ENAC/quadrotor/anton_indi_aruco.xml` * | `<module name="stabilization" type="NEW_TYPE"/>` |
| `conf/airframes/ENAC/quadrotor/anton_indi_aruco.xml` * | Replace `STABILIZATION_ATTITUDE_INDI` section with new section |
| `conf/airframes/ENAC/conf_enac.xml` ? | Update `settings_modules` to include/remove the right panel XML |
| `conf/modules/stabilization_NEW_TYPE.xml` ? | Create if using a custom type |
| `sw/airborne/firmwares/rotorcraft/stabilization/stabilization_NEW_TYPE.c` ? | Create source |
| `sw/airborne/firmwares/rotorcraft/stabilization/stabilization_NEW_TYPE.h` ? | Create header |

---

## Change Guidance Algorithm (INDI → PID or new)

| File | Change |
|------|--------|
| `conf/airframes/ENAC/quadrotor/anton_indi_aruco.xml` * | `<module name="guidance" type="NEW_TYPE"/>` |
| `conf/airframes/ENAC/quadrotor/anton_indi_aruco.xml` * | Replace `GUIDANCE_INDI` / `GUIDANCE_H` / `GUIDANCE_V` sections |
| `conf/airframes/ENAC/conf_enac.xml` ? | Update `settings_modules` |
| `conf/modules/guidance_NEW_TYPE.xml` ? | Create if custom |
| `sw/airborne/firmwares/rotorcraft/guidance/guidance_NEW_TYPE.c` ? | Create source |

---

## Add GCS-Tunable Parameter to Existing Module

| File | Change |
|------|--------|
| `conf/modules/stabilization_indi.xml` * | Add `<dl_setting>` entry in `<dl_settings NAME="indi">` |
| `sw/airborne/firmwares/rotorcraft/stabilization/stabilization_indi.h` * | Add `extern float my_param;` |
| `sw/airborne/firmwares/rotorcraft/stabilization/stabilization_indi.c` * | Define and initialize `float my_param = DEFAULT;` |

---

## Change Filter Frequencies

| File | Change |
|------|--------|
| `conf/airframes/ENAC/quadrotor/anton_indi_aruco.xml` * | `FILT_CUTOFF` / `FILT_CUTOFF_R` / `ESTIMATION_FILT_CUTOFF` |
| *(also tunable live via GCS)* | `stabilization_indi_filter_freq` setting (triggers re-init of filters) |

---

## Change WLS Matrix Sizes

| File | Change |
|------|--------|
| `conf/airframes/ENAC/quadrotor/anton_indi_aruco.xml` * | `WLS_N_U_MAX`, `WLS_N_V_MAX` inside the `<module name="stabilization" type="indi">` tag |
| `conf/airframes/ENAC/quadrotor/anton_indi_aruco.xml` * | Update `INDI_NUM_ACT` / `INDI_OUTPUTS` configures if adding actuators |

---

## Add a New Actuator (e.g. 5th motor)

| File | Change |
|------|--------|
| `conf/airframes/ENAC/quadrotor/anton_indi_aruco.xml` * | Add `<servo>` entry |
| `conf/airframes/ENAC/quadrotor/anton_indi_aruco.xml` * | Add `<axis>` in `<commands>` |
| `conf/airframes/ENAC/quadrotor/anton_indi_aruco.xml` * | Add `<set>` in `<command_laws>` |
| `conf/airframes/ENAC/quadrotor/anton_indi_aruco.xml` * | Set `WLS_N_U_MAX` and `INDI_NUM_ACT` to 5 |
| `conf/airframes/ENAC/quadrotor/anton_indi_aruco.xml` * | Update `G1` (add column), `G2`, `ACT_FREQ`, `COMMANDS` |

---

## Change INS / State Estimator

| File | Change |
|------|--------|
| `conf/airframes/ENAC/quadrotor/anton_indi_aruco.xml` * | Swap `<module name="ins" type="ekf2"/>` for another type |
| `conf/airframes/ENAC/conf_enac.xml` ? | Update `settings_modules` (e.g. `ins_ekf2.xml` → `ins_float_invariant.xml`) |

---

## Add Higher-Order (Jerk/Snap) Trajectory Setpoints

| File | Change |
|------|--------|
| `sw/ext/pprzlink/message_definitions/v1.0/messages.xml` * | New `GUIDED_TRAJECTORY_NED` message (jerk/snap/heading-derivative fields) |
| `sw/airborne/firmwares/rotorcraft/guidance/guidance_h.h` / `.c` * | Extend `sp`/`ref` structs, add `guidance_h_set_flat()` |
| `sw/airborne/firmwares/rotorcraft/guidance/guidance_v.h` / `.c` * | Vertical equivalent |
| `sw/airborne/firmwares/rotorcraft/guidance/guidance_h_ref.h` / `.c` * | Taylor-extrapolation reference model (`gh_update_ref_from_flat_ref`) |
| `sw/airborne/firmwares/rotorcraft/guidance/guidance_v_ref.h` / `.c` * | Vertical equivalent |
| `sw/airborne/firmwares/rotorcraft/autopilot_guided.h` / `.c` * | New `autopilot_guided_parse_GUIDED_TRAJECTORY()` |
| `sw/airborne/firmwares/rotorcraft/guidance/guidance_mfc.c` ? | Follow-up: wire `ref.speed/accel` into MFC feedforward |

See [[Plans/Flatness Trajectory Setpoints (Pos-Vel-Accel-Jerk-Snap + Psi)]] for the full design.

---

## File Path Quick Reference

```
AIRFRAME XML:
  paparazzi/conf/airframes/ENAC/quadrotor/anton_indi_aruco.xml

FLEET REGISTRY:
  paparazzi/conf/airframes/ENAC/conf_enac.xml

STABILIZATION SOURCE:
  paparazzi/sw/airborne/firmwares/rotorcraft/stabilization/
    stabilization_indi.c          ← core algorithm
    stabilization_indi.h
    stabilization_attitude_quat_indi.c  ← outer PD loop
    stabilization_rate_indi.c     ← rate-only variant
    stabilization_andi.c          ← ANDI variant

GUIDANCE SOURCE:
  paparazzi/sw/airborne/firmwares/rotorcraft/guidance/
    guidance_indi.c               ← base INDI guidance
    guidance_indi_quadrotor.c     ← quadrotor G matrix
    guidance_v.c                  ← vertical axis (classical)

MODULE XML:
  paparazzi/conf/modules/
    stabilization_indi.xml
    guidance_indi.xml
    guidance_indi_base.xml
    guidance_indi_quadrotor.xml

WLS MATH:
  paparazzi/sw/airborne/math/wls/wls_alloc.c
  paparazzi/sw/airborne/math/wls/wls_alloc.h

BUILD:
  ./pprz.sh build AIRCRAFT TARGET
  Output: paparazzi/var/aircrafts/AIRCRAFT/ap/obj/ap.elf
```

## See Also

- [[06 - Modifying ANTON Stabilization]] — worked examples
- [[02 - INDI Stabilization Deep Dive]] — algorithm internals
- [[04 - Airframe XML Configuration]] — XML reference
