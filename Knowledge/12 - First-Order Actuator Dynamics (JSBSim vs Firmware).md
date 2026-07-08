# First-Order Actuator Dynamics — JSBSim Model vs Firmware (ANTON / ANTON_MFC)

How the first-order motor lag is modeled in the JSBSim plant (`conf/simulator/jsbsim/aircraft/anton.xml`), how the firmware accounts for it (`stabilization_indi.c` / `stabilization_mfc.c`), and how the rotor transient enters the **yaw** axis on both sides.

---

## 1. Units: ACT_FREQ = 30.5 is **rad/s, not Hz**

`<define name="ACT_FREQ" value="{30.5, 30.5, 30.5, 30.5}"/>` is the **continuous-time corner frequency of the first-order actuator lag, in rad/s**.

Evidence, `stabilization_indi.c:228-230` (same text in `stabilization_mfc.c:293-295`), the deprecation warning for the old `ACT_DYN` define:

> *"You now have to define the continuous time corner frequency **in rad/s** of the actuators. Use `-ln(1 - old_number) * PERIODIC_FREQUENCY` to compute it from the old values."*

So the assumed motor model is

```
A(s) = ω / (s + ω),   ω = 30.5 rad/s
```

- corner frequency: 30.5 rad/s ≈ **4.85 Hz**
- time constant: τ = 1/ω ≈ **32.8 ms** (63% rise)

The JSBSim side uses the same convention: a JSBSim `<lag_filter>` implements `C(s) = c1/(s + c1)` with `c1` in rad/s. `anton.xml:129` sets `fcs/motor_lag = 30.` — so the plant lag (30 rad/s) and the firmware's assumed lag (30.5 rad/s) are essentially matched (the 0.5 rad/s difference is inconsequential; the firmware value came from real-vehicle identification).

The same number appears once more as `GUIDANCE_INDI_THRUST_DYNAMICS_FREQ = 30.5` (airframe XML), used by guidance to model the vertical-thrust transient — same units, same motor.

---

## 2. JSBSim side: how the lag is applied (`anton.xml`)

NPS writes the four normalized motor commands (0..1) directly into the JSBSim properties `fcs/ne_motor` … `fcs/nw_motor` (`NPS_ACTUATOR_NAMES` path, `nps_fdm_jsbsim.cpp:325-332`; `NO_MOTOR_MIXING=TRUE` so the firmware's per-motor outputs map 1:1).

Inside the `actuator_dynamics` flight-control block each motor command is processed **twice, in parallel**:

### a) Lag filter → thrust and steady yaw torque
```
fcs/X_motor ──► lag_filter (c1 = 30 rad/s) ──► fcs/X_motor_lag
```
`fcs/X_motor_lag` is the *physical rotor state* (normalized thrust). It drives:
- the **lift force**: `F = motor_lag × 10 N` per motor (`external_reactions`)
- via `g1_gain` (=1), the **steady drag/yaw torque** contribution.

### b) Washout filter → spin-up (transient) yaw torque
```
fcs/X_motor ──► washout_filter (c1 = 50 rad/s) ──► fcs/X_motor_d
```
A JSBSim washout is a high-pass `s·c1/(s + c1)` — i.e. a **band-limited derivative** of the motor command. This approximates `dΩ/dt`, the rotor angular acceleration. Accelerating a rotor requires torque from the airframe, so the airframe feels the **reaction (counter) torque** — the physical effect that firmware G2 models.

### c) Yaw moment = G1·lag + G2·derivative
Per motor:
```
fcs/X_yawcontrol = g1_gain · X_motor_lag  +  g2_gain · X_motor_d
```
applied as a body-Z moment of `yawcontrol × 0.5 N·m`, with sign per rotor spin direction (NE/SW +Z i.e. CCW props, NW/SE −Z). This is exactly the structure the firmware assumes: **yaw torque = steady drag term (G1 row 3) + spin-up transient term (G2)**.

Roll/pitch/thrust see only the lagged signal (forces at ±0.15 m arms); the washout/G2 path exists **only for yaw**.

---

## 3. Firmware side: how the transient is computed

The real firmware never measures the motor transient on ANTON (no RPM feedback: `STABILIZATION_*_RPM_FEEDBACK` unset). It **predicts** it open-loop with the same first-order model, discretized at `PERIODIC_FREQUENCY` = 500 Hz.

### a) Discretization at init
`stabilization_indi_init()` (`stabilization_indi.c:467`) / `stabilization_mfc_init()` (`stabilization_mfc.c:555`):
```c
act_dyn_discrete[i] = 1 - exp(-act_first_order_cutoff[i] / PERIODIC_FREQUENCY);
// = 1 - exp(-30.5/500) ≈ 0.0592
```
This is the **exact ZOH discretization** of the first-order lag (not the Euler approximation ω·Ts = 0.061).

### b) Per-tick actuator state propagation
`get_actuator_state()` (`stabilization_indi.c:980`, identical in `stabilization_mfc.c:948`):
```c
actuator_state[i] += act_dyn_discrete[i] * (u[i] - actuator_state[i]);
```
`actuator_state` (PPRZ units, 0..9600) is the firmware's estimate of where the rotors *actually are*, mirroring `fcs/X_motor_lag` in the sim. An optional `ACT_RATE_LIMIT` clamp exists but is not set for ANTON.

### c) Filtering and differentiation
Each `actuator_state[i]` is passed through 2nd-order Butterworth low-passes (control path at `FILT_CUTOFF`, estimation path at `ESTIMATION_FILT_CUTOFF` = 4 Hz), then differentiated by finite difference to get `actuator_state_filt_vectd` (≈ dΩ/dt) and `...vectdd`. These derivatives feed the adaptive LMS estimator; the yaw row uses `ddu` to estimate **G2** online (`lms_estimation()`, active only if `USE_ADAPTIVE`, which is FALSE on ANTON).

---

## 4. How the transient enters yaw control

### INDI (`stabilization_indi_rate_run`)

G2 (`{150, -150, 150, -150}`, sign = rotor spin direction, scaled by `INDI_G_SCALING` = 1000) appears in **three places**:

1. **Feedforward compensation** (`stabilization_indi.c:771,784`):
   ```c
   g2_times_u = dot(g2, indi_u) / 1000;
   indi_v[2]  = (accel_ref.r - disturbance_r) + g2_times_u;
   ```
   The yaw virtual command is *inflated* by the spin-up torque the new command will itself create, so the allocator solves for the net aerodynamic yaw acceleration actually wanted.

2. **Prediction / disturbance estimate** (`:657-662, 690-691`): the predicted angular acceleration `B·u_filt` has `g2·u_filt_prev/1000` **subtracted** on the yaw row before being compared with the measured (filtered, differentiated) gyro acceleration. The residual is the yaw disturbance estimate, bounded by `YAW_DISTURBANCE_LIMIT`. Because both prediction and measurement pass through the same Butterworth filters, the 32.8 ms actuator lag cancels in the increment — this is the core INDI trick, and it only works because `ACT_FREQ` matches the real (and JSBSim) motor lag.

3. **Allocation matrix** (`sum_g1_g2()`, `:1137-1150`): the WLS `Bwls` yaw row is `(g1[2][j] + g2[j]) / 1000` — G2 is folded into yaw effectiveness only; roll/pitch/thrust rows are `g1/1000`.

### MFC (`stabilization_mfc_attitude_run`)

MFC is model-free on the loop side: the yaw virtual command `mfc_v[2]` is the SISO MFC output directly (`mfc_core.c`), with **no explicit `g2_times_u` feedforward and no B·u prediction** — the unmodeled spin-up torque is absorbed by the MFC ultra-local disturbance estimator `F̂`. The actuator model still matters in three ways:

1. `sum_g1_g2()` is identical: the **WLS yaw row is (G1+G2)/1000**, so allocation still knows that commanding a rotor step produces extra yaw torque.
2. `get_actuator_state()` + `ACT_FREQ` drive the filtered actuator state used for the **thrust estimate** (`stab_thrust_filt`, `Bwls[3]·act_filt`) and for `cmd[COMMAND_THRUST]` reporting.
3. The actuator-state derivatives feed the (currently disabled) adaptive G1/G2 estimation.

---

## 5. Side-by-side summary

| Aspect | JSBSim plant (`anton.xml`) | Firmware (INDI / MFC) |
|---|---|---|
| Lag model | `lag_filter`, c1 = **30 rad/s** | 1st-order ZOH, ω = **30.5 rad/s** (`ACT_FREQ`) |
| Units | rad/s (JSBSim `c1`) | **rad/s** (≈4.85 Hz, τ≈32.8 ms) |
| Discrete gain @500 Hz | — (JSBSim integrates continuously) | `1−exp(−ω/fs)` ≈ 0.0592/tick |
| Transient (dΩ/dt) | washout filter, c1 = 50 rad/s | finite-diff of Butterworth-filtered predicted actuator state |
| Yaw torque structure | `g1·motor_lag + g2·motor_d` per rotor | `Bwls[2] = (G1[2]+G2)/1000`; INDI adds `+g2·u/1000` feedforward to `indi_v[2]` |
| Transient measured? | n/a (it *is* the plant) | **No** — predicted open-loop (no RPM feedback on ANTON); INDI cancels model error incrementally, MFC absorbs it in F̂ |
| G2 applied to | yaw only | yaw only (`sum_g1_g2`, `i==2`) |

**Caveats / mismatches worth knowing**
- JSBSim's washout corner (50 rad/s) ≠ firmware estimation filter (4 Hz ≈ 25 rad/s); only the adaptive estimator would care, and it's off.
- JSBSim `nw_couple` moment is applied at location `(0, −0.25, 0)` instead of `(−0.15, −0.15, 0)` — a plant-model typo, harmless for a pure Z-moment but inconsistent with the other three.
- Firmware G2 = 150/1000 = 0.15 (rad/s²)/(PPRZ/s) equivalent; JSBSim `g2_gain` = 1 on a 0.5 N·m scale — the two are calibrated independently, not derived from each other.
