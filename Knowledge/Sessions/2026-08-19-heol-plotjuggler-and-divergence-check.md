# 2026-08-19 — HEOL PlotJuggler layout, post-fix SITL check, and the horizontal divergence root cause

## 1. plotjuggler_heol.xml

Added: "Attitude [rad]" (roll/pitch/yaw sp/me/TRUTH), "Attitude: torque [N*m]"
(u_ff/cmd per axis + fk), "Motors [pprz]" (u0..u3), and a third column on
"Nominal inputs" for HEOL_NOMINAL Mx*/My*/Mz*. Attitude curves come from
`MFC_STAB` (STAB_MFC id 212, reused by stabilization_heol.c the way
guidance_heol.c reuses GUIDANCE_MFC).

**Corrected a pre-existing mislabel** while doing it: the tab formerly called
"XY: accel cmd [m/s^2]" plots `cmd_x`/`cmd_y`, which are
`heol_hxy.command[0]`/`[1]`. In the 2x2 MIMO channel the OUTPUT space is (x,y)
but the INPUT space is (phi,theta), so **`cmd_x` is PHI and `cmd_y` is THETA, in
radians** — not accelerations. Renamed to "XY: tilt cmd [rad]" with the
convention spelled out. The correct pairing when reading the loop is
`err_x` (north) against `cmd_y` (theta) and `err_y` (east) against `cmd_x`
(phi); `fk_x`/`fk_y` really are m/s² in (x,y) space.

I got this wrong in my first pass at reading the log and briefly concluded the
feedback sign was inverted. It is not — once the indices are mapped correctly
the signs are right.

## 2. SITL after cfede3078: divergence is NOT gone

`cfede3078` (post-clamp estimator tap) shipped untested. Ran
`./sim.sh ANTON_HEOL --nav "Start Engine,Takeoff,+3,Standby,+1,Flat_Traj_Demo,+8,Standby"`,
confirmed the built `airframe.h` carries no `EST_PRESAT` override so the fixed
`#ifndef … false` default was live.

From `sim_logs/mfc_sim_20260819_084939.csv`: horizontal rails at ±20° bank from
t≈10 s — **during plain Standby, before the flat trajectory engages** — and
`TRUTH/y` runs to −53 m by t=24 s. `fk_y` swings −9.8 … +34.6. `TRUTH/theta`
peaks at 1.45 rad. So the estimator-tap fix was necessary but not sufficient.

## 3. Root cause (code + arithmetic, NOT yet sim-verified)

**The Q8 fixed-point position reference is differentiated raw at 500 Hz.**

`guidance_heol_horiz()` builds `ref` from `POS_FLOAT_OF_BFP(gh->ref.pos.x/y)`.
`gh->ref.pos` is `struct Int32Vect2` at `INT32_POS_FRAC = 8` → **1 LSB = 3.906 mm**.
That reference goes into `epsilon = meas - ref`, and `heol_mimo_run()` hands
`epsilon` to `mfc_mimo_run()` *as the measurement*. Inside, with
`decoupled = true` (which `heol_mimo_init()` hardcodes):

```c
float dot_err = (m->error[0][i] - m->error[1][i]) / ts;   // mfc_core_mimo.c:199
fb = m->kd * dot_err + m->kp * m->error[0][i] + ...;
```

`ts = 1/500`. One LSB of reference movement therefore produces:

| quantity | value |
|---|---|
| 1 LSB | 3.906 mm |
| `dot_err` impulse | **1.953 m/s** |
| × kd (2.8) | 5.469 m/s² |
| ÷ alpha (≈9.81) → attitude | **0.5575 rad = 31.9°** |
| bank rail | 0.349 rad = 20° |
| ratio | **1.60× the rail** |

**A single least-significant bit of reference motion overshoots the bank limit by
60%.** The kp path for the same LSB is 0.09° — this is entirely the derivative
term. And `GUIDANCE_HEOL_HXY_COMMAND_FILTER = 1`, i.e. *no output filter*, so
nothing smooths it (gz uses 8, the attitude channels use 10 — hxy is the one
channel with none).

When the reference is moving at 1 m/s it crosses an LSB every ~3.9 ms ≈ every
2 ticks, so this is a continuous ±32° impulse train → the observed bang-bang.

**The same quantization also poisons F_hat.** The estimator's `s2d2e` term
divides by `ts²`:

```c
s2d2e = (t²z0 - 2(t-ts)²z1 + (t-2ts)²z2) / (ts*ts);   // mfc_core.c:36
```

with `z0 = epsilon`, carrying the same staircase. A 1-LSB step gives
`ΔF_hat = 976.6 m/s²` pre-IIR (independent of t — the t² cancels against
`den = t²`). Through the `int_window = 500` IIR (τ ≈ 1.0 s) a single-sample
impulse lands at roughly `976.6 × ts/τ ≈ 2 m/s²` sustained — which matches the
**observed `fk_y ≈ 1.78` while hovering** to about 10%. That 1.78 alone maps to
−0.34 rad of phi, i.e. F_hat by itself is sitting on the rail.

### Why plain MFC worked and HEOL does not

Not the gains, and not `decoupled` — ANTON_MFC also ran `GX_DECOUPLED=TRUE`
with `GX_COMMAND_FILTER=1` and a *larger* `GX_KD=25`. The difference is the
**setpoint path**:

- `guidance_mfc.c` puts the quantized reference in `mfc_gx.setpoint` and passes
  the *measurement* to `mfc_siso_run()`. With `USE_TRAJECTORY_SP` defaulting to
  `true` and `GX_TIME_TRAJECTORY = 50`, the reference is smoothed by
  `mfc_iir_step` (double pole, **τ ≈ 101 ms**) before the error is formed.
  LSB kick on `dot_err`: **0.0387 m/s — 50× smaller.**
- `heol_mimo_init()` hardcodes `use_trajec_sp = false` and `setpoint = 0`, and
  the reference arrives inside `epsilon` as the *measure*. **The smoother is
  bypassed by construction**, and there is no other filter on `ref` — note
  `meas` IS Butterworth-filtered (3 Hz) and `ref` is not, so the residual is
  also phase-mismatched.

That is the regression, and it is structural to the HEOL wrapper rather than a
tuning error.

### Secondary findings

- `HXY_EST_HOLD_TIME = 0.1 s` against `int_window = 500` (τ ≈ 1.0 s): the
  estimator goes live at **0.1 of one time constant**, ~90% unconverged. The
  attitude channels use hold 0.8 s with window 20 (τ ≈ 41 ms) = 19.5 τ.
- `mfc_core_mimo.c` has **no `use_external_derivative` hook**. Commit
  `4aa6b2b37` added exactly that to the SISO core so the attitude channels
  could use gyro rate instead of a differenced angle — the same fix was never
  ported to the MIMO channel, even though NED velocity is readily available
  and `gh->ref.speed` is Q19 (LSB 1.9e-6 m/s, negligible).
- **ZYX vs YXZ inconsistency**: `heol_input_sensitivity.c` computes alpha_xy as
  the exact **ZYX** Jacobian (verified term by term against
  R = Rz(psi)Ry(theta)Rx(phi)), but `guidance_heol_horiz()` builds the setpoint
  with `float_quat_of_eulers_yxz()`. Real, and it is the "unresolved ZYX/ZXY
  question" the comments flag as harmless — which stopped being harmless when
  `187b454bf` made alpha load-bearing. But it is second-order at these angles,
  so it is not the divergence driver.
- **Ruled out**: float32 catastrophic cancellation in `mfc_est_num`. Checked
  numerically — 0.07% relative error at t=50 s. Not a factor.
- **Ruled out**: alpha going singular / falling back to identity. `det` is
  `(T/m)²` ≈ 96.2 and independent of psi, never near `MFC_MIMO_DET_MIN_ABS`.

## 4. Proposed fix (not implemented)

Two changes, because the two paths need different treatment:

1. **Derivative path** — port `use_external_derivative` from `mfc_core` to
   `mfc_core_mimo`, and feed it `stateGetSpeedNed_f() − SPEED_FLOAT_OF_BFP(gh->ref.speed)`
   (measured minus reference, per the sign note in mfc_core.h). Kills the
   differentiation entirely; mirrors what the attitude channels already do.
2. **Estimator path** — an external derivative does *not* fix `s2d2e`, since
   `z0 = epsilon` still carries the staircase. Filter `ref` through the **same**
   Butterworth already applied to `meas` (identical filters ⇒ unbiased residual,
   and it removes the existing phase mismatch as a bonus).

Then re-check `HXY_EST_HOLD_TIME` against the 1.0 s window.

Per CLAUDE.md this is a restore-or-promote situation: both are permanent
structural fixes, not diagnostics, so they go in as normal code — but the
result must be re-flown and recorded from the fixed configuration.

**Status: hypothesis.** The arithmetic above is exact and was computed this
session; the causal claim that it is *the* divergence driver has not been
tested in SITL yet.

## Method notes

- Reading the log before mapping the index convention cost me a wrong
  conclusion (inverted sign). In a MIMO loop the telemetry field names are in
  the *output* space and the commands are in the *input* space; check which is
  which before drawing any inference from a trace.
- The decisive step here was arithmetic on the quantization, not more sim time
  — same lesson as bug 2 on 2026-08-18 (the fixed-point ratchet). Fixed-point
  reference into a fast differentiator is now a **recurring** failure mode in
  this codebase; worth its own Knowledge note.
- `./sim.sh --no-build` is fast once warm; the first `nps` build is not. Don't
  run long `--nav` scenarios on a cold build.
