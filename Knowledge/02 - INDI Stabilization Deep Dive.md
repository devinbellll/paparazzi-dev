# INDI Stabilization Deep Dive

## What is INDI?

**Incremental Nonlinear Dynamic Inversion** — a model-based control law that uses the measured angular acceleration and a model of actuator effectiveness to compute incremental actuator commands rather than absolute ones. It is inherently robust to model uncertainty because it only needs the *change* in control effectiveness.

Original paper: Smeur et al., "Adaptive Incremental Nonlinear Dynamic Inversion for Attitude Control of Micro Aerial Vehicles", JGCD 2016.

## Source Files

| File | Purpose |
|------|---------|
| `sw/airborne/firmwares/rotorcraft/stabilization/stabilization_indi.c` | Core INDI algorithm: filtering, G-matrix update, WLS allocation |
| `sw/airborne/firmwares/rotorcraft/stabilization/stabilization_indi.h` | Public API, extern declarations |
| `sw/airborne/firmwares/rotorcraft/stabilization/stabilization_attitude_quat_indi.c` | Outer attitude-to-rate PD loop (wraps stabilization_indi.c) |
| `sw/airborne/firmwares/rotorcraft/stabilization/stabilization_attitude_quat_indi.h` | |
| `sw/airborne/math/wls/wls_alloc.c` / `.h` | Weighted Least Squares allocator |
| `conf/modules/stabilization_indi.xml` | Module definition: compile flags, settings panel, dependencies |

## Algorithm Flow (one control cycle)

```
1. get_actuator_state()       ← read current actuator output (e.g. DShot feedback)
2. filter_rates()             ← low-pass-filter body rates p,q,r
3. Compute angular_acceleration[] ← numerical differentiation of filtered rates
4. Filter actuator states     ← Butterworth on u[] to match sensor delay
5. Attitude outer loop        ← stabilization_indi_attitude_controller()
     quat error → rate setpoint  (PD with REF_ERR_* / REF_RATE_* gains)
6. Rate inner loop            ← stabilization_indi_rate_controller()
     rate error → angular_accel_ref
7. Compute indi_v[]           ← virtual control = angular_accel_ref − angular_acceleration
8. WLS allocation             ← wls_alloc(indi_u, indi_v, Bwls, ...)
     solves:  B·Δu ≈ v  subject to u_min ≤ u ≤ u_max
9. Apply Δu to current actuator state → actuators_pprz[]
10. lms_estimation() (optional) ← adaptive G1/G2 update
```

## Key Parameters (set in airframe XML under `STABILIZATION_ATTITUDE_INDI`)

| XML define       | C variable                       | Meaning                                             |
| ---------------- | -------------------------------- | --------------------------------------------------- |
| `REF_ERR_P/Q/R`  | `indi_gains.att.p/q/r`           | Attitude error gains (outer PD loop)                |
| `REF_RATE_P/Q/R` | `indi_gains.rate.p/q/r`          | Rate error gains (inner loop)                       |
| `FILT_CUTOFF`    | `actuator_lowpass_filters[].tau` | Actuator Butterworth 2nd-order cutoff (Hz)          |
| `ACT_FREQ`       | `act_first_order_cutoff[]`       | Continuous-time actuator bandwidth (rad/s)          |
| `G1`             | `g1[INDI_OUTPUTS][INDI_NUM_ACT]` | Control effectiveness matrix                        |
| `G2`             | `g2[INDI_NUM_ACT]`               | Propeller gyroscopic/inertia coupling               |
| `WLS_PRIORITIES` | `wls_stab_p.Wv[]`                | Per-axis priority in WLS (roll, pitch, yaw, thrust) |
| `WLS_WU`         | `wls_stab_p.Wu[]`                | Per-actuator cost in WLS                            |
| `USE_ADAPTIVE`   | `indi_use_adaptive`              | Enable LMS G-matrix estimation                      |
| `ADAPTIVE_MU`    | `mu1[]`                          | LMS learning rate                                   |

## The G1 Matrix (Control Effectiveness)

```
G1 is [INDI_OUTPUTS × INDI_NUM_ACT]:
  rows: controlled axes  [roll, pitch, yaw, thrust]   (4 for ANTON)
  cols: actuators        [FR, BR, BL, FL]             (4 for ANTON)

ANTON's G1 (from anton_indi_aruco.xml):
         FR     BR     BL     FL
roll  [ -40,   -40,    40,    40 ]
pitch [  40,   -40,   -40,    40 ]
yaw   [   5,    -5,     5,    -5 ]
thrust[ -1.5,  -1.5,  -1.5, -1.5 ]

Signs follow paparazzi body frame convention.
```

## The G2 Vector (Propeller Inertia)

`G2 = {150, -150, 150, -150}` for ANTON.  
Models the counter-torque effect when a propeller spins up/down.  
Enters as: `g1g2 = g1 + g2 * d(actuator)/dt`.

## WLS Allocation

`WLS_N_U_MAX` and `WLS_N_V_MAX` are **compile-time** matrix sizes.  
They must be `≥ INDI_NUM_ACT` and `≥ INDI_OUTPUTS` respectively.  
ANTON sets them in the airframe XML:
```xml
<define name="WLS_N_U_MAX" value="4"/>
<define name="WLS_N_V_MAX" value="4"/>
```

## GCS-Tunable Parameters (runtime, no recompile)

The `stabilization_indi.xml` module registers a `dl_settings` panel.  
In the GCS you can change live:
- `indi_gains.att.*` / `indi_gains.rate.*` — PD gains
- `indi_use_adaptive` — toggle adaptive mode
- `stablization_indi_yaw_dist_limit` — yaw disturbance cap
- `stabilization_indi_filter_freq` — filter cutoff (uses handler to re-init filters)

## Adaptive Mode

When `USE_ADAPTIVE=TRUE`, the `lms_estimation()` function runs every cycle and slowly adjusts `g1_est`/`g2_est` using LMS gradient descent, bounded by `INDI_ALLOWED_G_FACTOR = 2.0` of the initial values.

## PERIODIC_FREQUENCY is part of the INDI tuning — especially for yaw (G2)

`PERIODIC_FREQUENCY` (the main AP/control loop rate, set via `<configure>` in the
airframe) is **not** a free implementation detail for INDI — the discrete-time
INDI math bakes it in. A set of INDI gains is only valid at the frequency it was
tuned at. Key dependencies in `stabilization_indi.c`:

- **Actuator dynamics model** (`:457`):
  `act_dyn_discrete[i] = 1 - exp(-ACT_FREQ[i] / PERIODIC_FREQUENCY)`.
  With `ACT_FREQ=30.5`: `0.0300` @ 1000 Hz vs `0.0592` @ 500 Hz — **~2×** off if
  the rate changes but `ACT_FREQ` doesn't.
- **Actuator-rate / angular-acceleration estimates** (`:642,643,670,675,676`):
  all derivatives are computed as `Δ × PERIODIC_FREQUENCY`.
- **Filter sample time** (`:532,552`): `1 / PERIODIC_FREQUENCY` for the
  measurement / actuator / estimation low-pass filters.

**Why yaw breaks first.** The yaw axis is driven almost entirely by the **G2**
spin-up-torque compensation (`g2_times_u = G2·indi_u/INDI_G_SCALING`, added to the
yaw objective `indi_v[2]`), and has weak direct effectiveness (`G1` yaw row ≈ ±5
vs ±40 for roll/pitch). G2 works by *predicting the actuator rate-of-change* —
exactly the quantity governed by `act_dyn_discrete` and the
`PERIODIC_FREQUENCY`-scaled derivatives. Run the loop at half the tuned rate and
that prediction is ~2× wrong; the low-authority, G2-dependent yaw axis
limit-cycles while roll/pitch (high direct authority, no G2 reliance) still look
fine. Net rule: **if you change `PERIODIC_FREQUENCY`, you must re-tune G2 /
ACT_FREQ / REF_RATE and the filter cutoffs — or expect a yaw oscillation.**

See [[Sessions/2026-06-15-yaw-oscillation-mfc-indi]] for the debugging story.

## See Also

- [[04 - Airframe XML Configuration]] for the full section syntax
- [[06 - Modifying ANTON Stabilization]] for a worked example
- [[07 - All Touch Points Cheatsheet]]
