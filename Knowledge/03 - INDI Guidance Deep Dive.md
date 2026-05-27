# INDI Guidance Deep Dive

## Role in the Stack

Guidance sits **above** stabilization. It converts position/velocity errors into an attitude command (and thrust) that stabilization then executes.

```
Navigation setpoint (pos/vel/heading)
        │
        ▼
guidance_indi_run_mode()           ← main entry point
        │  calls
        ▼
guidance_indi_controller()         ← computes virtual acceleration
        │  then
        ▼
guidance_indi_calcG()              ← builds 3×3 G matrix from current attitude
        │  then
        ▼
WLS / pseudo-inverse solve         ← outputs [dtheta, dphi, dT]
        │
        ▼
StabilizationSetpoint (attitude quat)
```

## Source Files

| File | Purpose |
|------|---------|
| `sw/airborne/firmwares/rotorcraft/guidance/guidance_indi.c` | Base INDI guidance logic |
| `sw/airborne/firmwares/rotorcraft/guidance/guidance_indi.h` | Public API |
| `sw/airborne/firmwares/rotorcraft/guidance/guidance_indi_quadrotor.c` | Quadrotor-specific G matrix (`guidance_indi_calcG_yxz`) |
| `conf/modules/guidance_indi.xml` | Meta-module (just points to `guidance_indi_quadrotor`) |
| `conf/modules/guidance_indi_quadrotor.xml` | Compiles `guidance_indi_quadrotor.c` |
| `conf/modules/guidance_indi_base.xml` | Compiles `guidance_indi.c`, registers settings panel |

## The G Matrix (guidance level)

Different from the stabilization G1 matrix.  
This is a 3×3 matrix relating `[dtheta, dphi, dT]` to `[ddx, ddy, ddz]`:

```c
// YXZ euler convention (used for quadrotors)
Gmat[0][0] = ctheta * cphi * T;   // dtheta → ddx
Gmat[1][0] = 0;
Gmat[2][0] = -stheta * cphi * T;
// ... etc.
```

The matrix is recomputed every cycle from the current attitude quaternion.

## Key Parameters (airframe XML under `GUIDANCE_INDI`)

| XML define | C variable | Meaning |
|-----------|-----------|---------|
| `THRUST_DYNAMICS_FREQ` | `guidance_indi_specific_force_gain` | Thrust first-order filter freq (Hz), matches ACT_FREQ |
| `RC_DEBUG` | — | Print RC debug output |

From `guidance_indi_base.xml` settings:

| XML define | C variable | Meaning |
|-----------|-----------|---------|
| `GUIDANCE_INDI_POS_GAIN` | `guidance_indi_pos_gain` | Position error → speed setpoint gain |
| `GUIDANCE_INDI_SPEED_GAIN` | `guidance_indi_speed_gain` | Speed error → accel setpoint gain |
| `GUIDANCE_H_MAX_BANK` | `guidance_indi_max_bank` | Maximum bank angle in guidance (rad) |

## Vertical Guidance (guidance_v)

Vertical axis uses a separate classical module in `guidance/guidance_v.c`.  
Parameters in airframe XML under `GUIDANCE_V`:
```xml
<section name="GUIDANCE_V" prefix="GUIDANCE_V_">
  <define name="REF_MIN_ZDD" value="-0.4*9.81"/>
  <define name="REF_MAX_ZDD" value=" 0.4*9.81"/>
  <define name="NOMINAL_HOVER_THROTTLE" value="0.30"/>
</section>
```

## Variants for Other Vehicle Types

| Module | Vehicle type |
|--------|-------------|
| `guidance_indi_quadrotor` | Standard quadrotor (ANTON, MAYA, CROW…) |
| `guidance_indi_hybrid` | Transitioning VTOL (FALCON_V2, CYFOAM) |
| `guidance_indi_fully_actuated` | Hexrotors with full 6-DOF control |
| `guidance_indi_hybrid_tailsitter` | Tailsitter VTOLs |
| `guidance_indi_hybrid_quadplane` | Quadplanes |

## See Also

- [[01 - Control System Architecture]]
- [[02 - INDI Stabilization Deep Dive]]
- [[06 - Modifying ANTON Stabilization]]
