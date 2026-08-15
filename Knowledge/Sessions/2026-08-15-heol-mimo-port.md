# 2026-08-15 — HEOL MIMO horizontal channel: core port + golden verification

Implements `Knowledge/Plans/HEOL MIMO Horizontal Channel (Matrix Alpha Port).md`.
**Core ported and verified; wiring done; rungs 0-4 pass, rung 5 (SITL) FAILS.**

## Done

- **`mfc_core.{c,h}` factored, not rewritten.** Three shared recursion
  primitives extracted so the MIMO core runs the *same* algebra rather than a
  second transcription:
  - `mfc_iir_step(x, x1, x2, window)` — the 2nd-order IIR. It turned out to
    serve **three** call sites with three different windows: estimator
    numerator, estimator denominator, and the reference-trajectory smoother,
    which was previously written out longhand in `mfc_siso_run()`.
  - `mfc_est_num(...)` — algebraic numerator for ONE channel. Takes the
    already-formed `alpha*d2u` so the scalar product and the matrix-vector row
    both fit; takes `z0,z1,z2` as three scalars rather than an array because the
    MIMO stores history as `[history][channel]`, which is not contiguous per
    channel.
  - `mfc_est_den(time)` — scalar, and named to make the asymmetry explicit.
- **`mfc_core_mimo.{c,h}`** — n=2 MIMO core. Per-element numerator, **scalar
  shared denominator** (`estimator_den[3]`, deliberately not `[3][N]`), one
  integration window, closed-form 2x2 inverse, `alpha` scheduled once per tick
  via `mfc_mimo_set_alpha()` so the estimator's `alpha@d2u` and the command
  law's `alpha^-1` solve use the same matrix by construction.
- **`tests/mimo_golden_test.c` + `tests/run_mimo_golden.sh`** — replays the
  traces through the real firmware sources in the build container, with
  `tests/stubs/` supplying `sys_time` and `pprz_algebra_float`.

## Verification ladder — rungs 0-3 pass

```
[0] scalar core vs SISO trace   worst rel err 0.000e+00   PASS
[1] mimo_diag                   worst rel err 0.000e+00   PASS
[2] mimo_diag u1 vs SISO u      worst rel err 0.000e+00   PASS
[2] mimo_diag u2 vs SISO u      worst rel err 0.000e+00   PASS
[3] mimo_cross                  worst rel err 5.702e-06   PASS
```

Rung 0 was added beyond the plan: it re-runs the *scalar* core against the SISO
trace, so the factoring is proven not to have changed `mfc_core.c`'s behaviour.
Rung 2 (the reduction check the plan calls decisive) is bit-exact.

## The one real obstacle: single precision cannot replay the traces

The first harness run failed at rung 0 *and* rung 1 identically — same row, same
magnitude — which immediately said "common cause, not the MIMO port". It is
arithmetic precision:

| precision | worst rel err vs SISO trace |
|---|---|
| float32 storage + float32 compute | **4.571e+00** (row 33) |
| float32 storage + float64 compute | 6.944e-01 (row 10) |
| float64 throughout | 0.000e+00 |

So it is not enough to compute in double — the *stored* history must be double
too. The cause is `mfc_est_num`'s second-difference terms divided by
`sample_time^2`: at Ts = 0.01 s they cancel catastrophically during the
reference step transient.

Handled by making the working type a compile-time knob: `MFC_FLOAT_T`, default
`float` (so **the target build is byte-identical in behaviour to before**), with
the harness compiled `-DMFC_FLOAT_T=double`. `ap` recompiles clean on ARM.

**Do not over-read this.** I initially assumed it would be far worse in the
firmware's regime (Ts = 0.002 s, `time` unbounded) and checked rather than
asserting. Measured, on a smooth excitation over 25 s:

```
t=20 s   F_hat  double 1.060572e+02   float 1.063178e+02   (~0.25 %)
```

float and double agree to 3-4 significant figures. The conditioning weakness
bites on **sharp transients**, not uniformly, so this is a real but bounded
issue — it does **not** by itself explain the horizontal oscillation or the
vertical bang-bang. Whether to move the flight code to double, or reformulate
the recursion, is an open question and NOT resolved here.

## Wiring (done)

- `heol_mimo.{c,h}` — 2-vector HEOL wrapper, same invariant as `heol.h`, same
  three clamp modes. Total-command bounds are the BANK LIMITS, so the max-bank
  saturation applies to nominal+increment rather than to the increment alone.
- `guidance_heol.c` — `heol_gx`/`heol_gy` replaced by one `heol_hxy`. u* is the
  flat map's nominal ATTITUDE (`phi_ref`, `theta_ref`); `gh->ref.accel` is no
  longer a feedforward, because the acceleration the trajectory needs is already
  expressed in the nominal attitude. **`accel_to_att_sp()` deleted**, along with
  the `heol_thrust_physical` latch it needed.
- Airframe: `GX_ALPHA`/`GY_ALPHA` retired (the gain is geometry now), the two
  per-axis estimator windows collapsed into one `HXY_INTEGRATION_WINDOW`.
- **Telemetry unit change**: `cmd_x`/`cmd_y` and `sp_traj_x`/`sp_traj_y` are now
  ATTITUDES in radians, not accelerations. `plotjuggler_heol.xml`'s
  "XY: accel cmd [m/s^2]" tab is mislabelled as a result and needs renaming.

## Rung 4 — firmware loop: PASS

```
alpha_xy level/psi=0 anti-diagonal   0.000e+00   PASS
alpha_z at level = 1/m               0.000e+00   PASS
alpha_xy worst-case conditioning     |det| = 51.54 (plan min 51.5)  PASS
command == u_ff + u_fb               0.000e+00   PASS
total attitude within max bank                   PASS
```

Two bugs this rung caught that nothing else would have:

- **A type hole I introduced.** `HeolInputSensitivity` was declared `float`
  while `heol_mimo_set_alpha()` takes `mfc_float_t`. On target the two coincide
  so it compiles silently; in the double-precision harness it is a genuine
  out-of-bounds read. `heol_input_sensitivity.{c,h}` now use `mfc_float_t`.
- **My own test was wrong first.** Rung 4c originally fed an invented
  `(T, phi, theta, psi)` built from the table's per-column extremes, which is a
  point the aircraft never flies; it gave |det| = 49.76, outside the plan's
  range. Re-derived from the table: |det| really does span 51.54 .. 179.65, and
  the argmin is row 541. The plan's 51.5 .. 179.7 is exactly right.

## Rung 5 — SITL: FAILS, and not subtly

`--nav "Start Engine,Takeoff,+3,Standby,+10,Flat_Traj_Demo,+40,Standby"`:

| window | err rms x/y [m] | |
|---|---|---|
| during trajectory | 2.00 / 1.52 | |
| +5 s settle | 13.08 / 7.95 | |
| late hold (35-55 s) | 47.97 / 79.03 | diverging |

`cmd_x`/`cmd_y` sit at **exactly ±0.3491 rad**, the 20 deg bank limit, for most
of the flight. The vertical channel is fine throughout (err rms 0.039 m late).

It is broken from the moment the horizontal loop engages at t ~ 7.5 s, well
before the trajectory. The diagnostic signature: at t = 7.46 the position error
is 0.022 m — essentially zero — and `F_hat` is already -1.30 and climbing; by
t = 15 s it reaches **-143** while the position error is only a few metres. The
estimator is running away, and the command saturates chasing it.

**Do not read this as the plan's "oscillation persists, look at the attitude
loop" branch.** That branch assumes a like-for-like comparison, and this is
worse than the ~30 m oscillation recorded for the scalar loop, on a longer
window, with no matched baseline flown. The honest status is: the port is
verified correct against the reference traces, and something about how it is
driven in firmware is wrong. Unexplained and still open.

Candidates not yet eliminated, roughly in order:

1. `F_hat` startup: `est_hold_time` blanks only 0.1 s, after which `den` is
   still tiny and `num/den` is large. Same in the scalar path, but the scalar
   path had a 5x mismatched alpha absorbing it.
2. The single-precision conditioning issue above, which is far worse here than
   in the traces: firmware runs Ts = 0.002 s with `time` unbounded, and this
   flight ran 400 s.
3. `HXY_INTEGRATION_WINDOW = 500` was inherited from the old per-axis GX/GY
   value; it has not been re-justified for the matrix form.
4. The ZYX/ZXY convention, now load-bearing rather than academic.
