# 2026-08-15 — HEOL MIMO horizontal channel: core port + golden verification

Implements `Knowledge/Plans/HEOL MIMO Horizontal Channel (Matrix Alpha Port).md`.
**Core ported, wired, and committed. Rungs 0-4 pass at plain float; rung 5
(SITL) FAILS and is undiagnosed after eliminating two hypotheses.**

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

## The precision detour — a wrong turn, recorded because it cost real time

The first harness run failed at rung 0 *and* rung 1 identically: same row, same
magnitude. That correctly said "common cause, not the MIMO port". The cause I
then assigned to it was wrong.

I modelled the algorithm in numpy at float32 and got a worst relative error of
**4.571e+00** against the SISO trace, versus 0.0 at float64. On that basis I
concluded single precision could not replay the traces, and introduced an
`MFC_FLOAT_T` typedef across `mfc_core`, `mfc_core_mimo`, `heol_mimo` and
`heol_input_sensitivity` so the harness could build the cores in double.

**The numpy model was not the C.** It rounded every intermediate to float32;
the compiled code does not. Measured on the actual firmware sources at plain
float, the worst disagreement with the traces is:

| check | worst rel err at float |
|---|---|
| scalar core vs SISO trace | 7.3e-4 |
| mimo_diag, and the reduction check | 7.3e-4 |
| mimo_cross | 1.2e-3 |

Four orders of magnitude better than the model predicted, and an unremarkable
single-precision result. All the harness ever needed was a tolerance matched to
the arithmetic: `TOL = 5e-3`, documented against those numbers. A genuine
porting error shows as O(1), several orders above, so the checks still catch
one. The typedef is reverted in full (54662c7a) and everything passes at float.

**Lesson worth keeping: do not build on a model of the code when the code
itself is available to measure.** The harness that would have falsified this in
one run already existed; I inferred instead of running it. The typedef then
propagated a second error — it forced float mirrors into `stabilization_mfc.c`
and `guidance_heol.c` for the by-address telemetry, which I argued for keeping
as a "latent bug fix" when it was an artifact of my own change.

## Diagnosing the rung-5 divergence: two hypotheses eliminated

Both tested against the same profile
(`Start Engine,Takeoff,+3,Standby,+10,Flat_Traj_Demo,+40,Standby`):

| build | late-hold err rms x/y | max \|F_hat\| |
|---|---|---|
| float, sensor noise ON (baseline) | 48.0 / 79.0 m | 143 |
| float, **all NPS sensor noise zeroed** | 46.9 / 76.0 m | 1343 |
| **cores in double**, noise ON | 47.4 / 76.8 m | 1362 |

Identical divergence in all three; `cmd_x` pinned at the ±0.3491 rad bank limit
throughout. So it is **neither sensor noise nor arithmetic precision**.

The noise result matters beyond this repo: the Simulink reference is reported
stable with noise off and unstable with noise on, so the obvious guess was that
the firmware was showing the same thing. It is not — the firmware diverges with
perfect sensors. They are different failures that look alike, and a noise fix on
the Simulink side should not be expected to fix this.

One trap on the way: the first double build produced a log full of `3e38`
values that looked like a diverging controller. It was telemetry reading half a
double as a float — the scope and telemetry take `float *` while the structs had
become `mfc_float_t`. The flight was fine; only the log was wrong. Worth
remembering before reading any log from a build whose types have changed.

## Schema tie-in — the full nominal set (7f8fe7c9)

`struct GuidanceFlatNominal` now carries the complete flat-map nominal input set
for all six HEOL channels, not just the three the position loops use:
`thrust`, `phi`, `theta`, `Mx`, `My`, `Mz`, plus the reference body rates. One
latch, because the six describe one instant of one trajectory — splitting them
would let the attitude loop and the position loops fly different samples. The
setter takes the whole record rather than a parameter list, since the channel
set is still growing.

Nothing consumes the moments or rates yet; they are published to the scope so
the attitude port starts from verified data. Verified against the table:

  Mx logged -0.02937 .. 0.11502   table -0.02944 .. 0.11502
  My logged -0.11502 .. 0.16266   table -0.11502 .. 0.16266
  Mz logged -0.03702 .. 0.02848   table -0.03703 .. 0.02848

(extremes differ only where 20 Hz nav sampling misses the table's peak row.)

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

## Commits

| repo | commit | what |
|---|---|---|
| paparazzi | `187b454bf` | horizontal channel becomes one 2x2 MIMO loop |
| paparazzi | `7f8fe7c97` | full flat-map nominal set onboard, all six channels |
| paparazzi | `54662c7af` | remove `mfc_float_t`; float-appropriate harness tolerance |
| paparazzi | `352e9ce15` | regenerated `flat_traj_demo_data.h` (12 new columns) |
| super | `66b0e2b`, `a11228b`, `bc65aba`, `09f27f1` | bumps + `tests/` + notes |

`352e9ce1` unblocked the build: the three firmware commits before it read columns
that until then existed only in the working tree. Confirmed with a from-scratch
`rebuild` of `ap` plus `nps`, harness still green.

The generated table still carries no GPL boilerplate -- the only `.h` in
`sw/airborne/modules/nav/` without one. Deliberately not fixed here: the
banner's "do not hand-edit" is correct and a hand-added header would be
destroyed by the next regeneration. Not a new gap either -- the file has been in
the tree unlicensed since `5185b674`; `352e9ce1` updates it rather than
introducing it. Tracked on the `Generic_Quad` side.

## Next, in order

1. **The rung-5 divergence.** Noise and precision are out. Cheapest remaining
   probe is `HXY_INTEGRATION_WINDOW`: it was inherited verbatim from the old
   per-axis GX/GY value of 500 and never re-justified for the matrix form, and
   the golden traces only ever validated a window of 10. The harness can sweep
   it offline against the traces with no sim run at all -- a far faster loop
   than the ~4 minute flights this session used. Then `est_hold_time`, then
   whether `heol_hxy` is fed the reference it expects.
2. **The attitude HEOL port.** Data is onboard and verified (7f8fe7c9). The loop
   is still plain MFC: three scalar `MfcParameters` doing setpoint tracking with
   tuned alphas (`2/Ixx`, `2/Iyy`, `3/Izz`) and no `u_ff`. The port means
   `HeolParameters` driven on epsilon, `alpha = 1/I`, and `M*_ref` as
   feedforward -- and the same 2-3x input-gain change that destabilised the
   vertical loop when `alpha_z` went live. Yaw's continuous-angle unwrapping
   (`mfc_psi_continuous`) must survive intact. Do it AFTER (1): two loops
   changing at once makes a regression unattributable.
3. **ZYX vs ZXY.** No longer academic -- `alpha_xy` is the horizontal input
   gain now, so the convention is load-bearing. Confirm what the generator
   actually emits.
4. Minor: `plotjuggler_heol.xml`'s "XY: accel cmd [m/s^2]" tab is mislabelled;
   that channel emits radians since the MIMO port.
