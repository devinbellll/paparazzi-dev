# 2026-07-07 — Porting the Paparazzi WLS allocator to MATLAB (`pprz_wls.m`)

## What was done

Created `pprz_wls.m` (workspace root): a `matlab.System` object that is a
line-for-line port of the active-set WLS control allocator
`wls_alloc()` from `paparazzi/sw/airborne/math/wls/wls_alloc.c`, configured
exactly as the MFC stabilizer (`stabilization_mfc.c`, `wls_stab_p`) sets it up
for the **ANTON_MFC** airframe
(`paparazzi/conf/airframes/ENAC/quadrotor/anton_mfc.xml`).

Usable directly in scripts (`wls = pprz_wls(); [u, iter] = wls(v);`) or in a
Simulink MATLAB System block. Input `v = [roll; pitch; yaw; thrust_z]`
pseudo-control; outputs actuator vector `u` (4×1, pprz units) and `iter`
(firmware convention: `imax+1` = ran out of iterations).

## Parameter mapping (firmware → System object)

| Firmware | Value | Source |
|---|---|---|
| `nu`, `nv` | 4, 4 | `MFC_NUM_ACT` / `MFC_OUTPUTS` |
| `gamma_sq` | 10000 | `wls_stab_p` initializer |
| `Wv` | `[1000 1000 1 100]` | `STABILIZATION_MFC_WLS_PRIORITIES` (XML) |
| `Wu` | `[1 1 1 1]` | `STABILIZATION_MFC_WLS_WU` default |
| `u_pref` | zeros | `STABILIZATION_MFC_ACT_PREF` default |
| `B` (Bwls) | `G1/1000`, yaw row `(G1_yaw + G2)/1000` | `sum_g1_g2()`, `MFC_G_SCALING = 1000` |
| `u_min` | 0 (all motors, `act_is_servo = {0}`) | `stabilization_mfc_set_wls_settings()` |
| `u_max` | 9600 (`MAX_PPRZ`) | same |
| call | `u_guess = NULL`, `W_init = NULL`, `imax = 10` | `wls_alloc(&wls_stab_p, Bwls, 0, 0, 10)` |

`u_guess = NULL` ⇒ initial `u = (u_min + u_max)/2`; empty working set.

G1/G2 (XML scaling, before /1000):

```
G1 = [-40 -40  40  40 ;   roll
       40 -40 -40  40 ;   pitch
        5  -5   5  -5 ;   yaw
     -1.5 -1.5 -1.5 -1.5] thrust
G2 = [150 -150 150 -150]  (yaw row only)
```

## Verification (no MATLAB in sandbox)

Mirrored the ported code in Python/NumPy and checked:

1. **Interior case** `v = [2, -1, 0.5, -12]`: converges in 1 iteration and
   matches the analytic unconstrained WLS solution
   (`min ‖γWv(Bu−v)‖² + ‖Wu(u−u_pref)‖²`) to machine precision.
2. **Saturating case** `v = [400, 0, 0, -12]`: two actuators pinned at
   `u_min = 0`, result stays in bounds, roll (priority 1000) achieved exactly
   while thrust (priority 100) is sacrificed — correct priority behavior.

## Notes / gotchas

- The C `qr_solve` (Householder QR least squares) and MATLAB `\` on a
  rectangular system agree for the full-rank problems this allocator sees;
  only float-vs-double precision differs from firmware.
- Lagrange-multiplier release threshold kept at `FLT_EPSILON`
  (`eps('single')`) to match the C code.
- `iter` semantics preserved: success returns the iteration count, hitting
  `imax` returns `imax+1`.
- G1/G2/`ActIsServo`/`MaxPprz`/`IMax` are nontunable (set at construction);
  `Wv`, `Wu`, `GammaSq`, `UPref` are tunable properties.

## Next

- Run the object in actual MATLAB/Simulink against logged firmware WLS_U/WLS_V
  telemetry to confirm parity on real data.
- Could add an optional `u_min`/`u_max` input port if the
  `GUIDANCE_MFC_MIN_THROTTLE` path (airspeed-dependent bounds) ever needs
  modeling.
