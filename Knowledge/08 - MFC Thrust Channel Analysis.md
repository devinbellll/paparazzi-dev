# MFC Thrust Channel Analysis

## How `mfc_v[3]` (thrust virtual control) is set

Both `stabilization_mfc.c` and `stabilization_indi.c` compute the thrust virtual control identically:

```c
// THRUST_INCR_SP branch (used by guidance_indi)
v_thrust.z = th_sp_to_incr_f(thrust, 0, THRUST_AXIS_Z);   // guidance increment (m/s²)

stab_thrust_filt.z = 0;
for (i = 0; i < NUM_ACT; i++) {
    stab_thrust_filt.z += Bwls[3][i]                        // G1[3][i] / G_SCALING
                        * actuator_lowpass_filters[i].o[0]  // filtered actuator_state
                        * (int32_t) act_thrust_mat[2][i];   // 1 for all quad motors
}
v_thrust.z += stab_thrust_filt.z;   // absolute = increment + current estimate

mfc_v[3] = v_thrust.z;
```

`G1[3] = {-1.5, -1.5, -1.5, -1.5}`, `G_SCALING = 1000`, so `Bwls[3][i] = -0.0015 (m/s²)/PPRZ`.

At stable hover (motors ~1833 PPRZ each):  
`stab_thrust_filt.z = 4 × (−0.0015) × 1833 ≈ −11`  
Guidance increment ≈ 0 (altitude held) → `v[3] ≈ −11`

## Why `mfc_v[3] = 0` when INDI shows −11

### The circular dependency

```
mfc_v[3]
  → WLS → mfc_u[i]
  → actuator_state[i]  (first-order model: state += act_dyn × (u − state))
  → actuator_lowpass_filters[i].o[0]
  → stab_thrust_filt.z
  → mfc_v[3]
```

With the default `act_pref = {0, 0, 0, 0}`, **zero is a stable fixed point** of this loop:
- Motors off → `stab_thrust_filt.z = 0`
- Guidance increment ≈ 0 (vehicle on ground / after crash)
- `mfc_v[3] = 0` → WLS drives `u → u_pref = 0` → motors stay off

### Why INDI escapes but MFC does not

INDI escapes the zero fixed-point because the vehicle achieves stable hover:
1. Guidance sends a negative thrust increment at takeoff (`th_sp_to_incr_f < 0`)
2. `indi_v[3]` becomes non-zero → WLS spins motors
3. `actuator_state` builds toward hover level → `stab_thrust_filt.z ≈ −11`
4. Self-sustaining at the hover fixed-point

MFC does not escape because **the attitude gains carried over from the Darko hybrid are not calibrated for ANTON**, causing attitude instability. The vehicle oscillates, crashes, and motors return to zero. Once there, the zero fixed-point holds and `mfc_v[3]` is stuck at 0.

## Key parameters (ANTON_MFC airframe)

| Parameter | Value | Effect |
|-----------|-------|--------|
| `G1[3]` | `{-1.5, -1.5, -1.5, -1.5}` | Thrust effectiveness per motor |
| `G_SCALING` | 1000 | `Bwls[3][i] = G1[3][i] / 1000 = -0.0015` |
| `ACT_FREQ` | `{30.5, …}` rad/s | First-order actuator model bandwidth |
| `act_pref` | `{0.0, …}` (default) | WLS preferred motor command — zero → zero fixed-point |
| `WLS_PRIORITIES` | `{1000, 1000, 1, 100}` | Roll/pitch/yaw/thrust weights in WLS |

## Potential fix: break the zero fixed-point

Set `act_pref` to nominal hover throttle so WLS regularizes around motors-running:

```xml
<!-- in STABILIZATION_MFC section of anton_mfc.xml -->
<define name="ACT_PREF" value="{2880, 2880, 2880, 2880}"/>
```

(2880 ≈ 0.30 × MAX_PPRZ, matching `NOMINAL_HOVER_THROTTLE = 0.30`)

With this, WLS drives `u → 2880` even with zero virtual controls → `actuator_state` stays at hover level → `mfc_v[3] ≈ −17` (observable). This does not fix attitude instability but makes the thrust channel diagnosable during gain tuning.

## Observing thrust channel via telemetry

`WLS_V` telemetry (period 0.04 s in `default_rotorcraft.xml`) sends `wls_stab_p.v[]`, which equals `mfc_v[0..3]` after the WLS copy. In `sim_anton.py`, `state["wls_v3"]` captures this value. A non-zero `wls_v3` confirms motors are running and the thrust feedback loop is active.
