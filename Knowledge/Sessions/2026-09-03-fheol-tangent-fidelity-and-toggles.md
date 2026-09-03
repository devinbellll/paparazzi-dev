---
date: 2026-09-03
topic: FHEOL Tangent fidelity, and XML-toggleable filters and clamps
tags: [session, fheol, fmfc, tangent, oneloop, nps, filters, clamps]
---

# FHEOL Tangent fidelity + XML-toggleable filters & clamps

Brief: `ThAIsis/Tasks/brief-fheol-tangent-fidelity.md`.
Commits: `8d3850f8e` (Part A), `0e049b769` (Parts B and C), on
`sitl-estimator-lib`, submodule `paparazzi` of `paparazzi_dev`.

## Goal

Bring `oneloop_fheol` to the fidelity of the Simulink Tangent variant, and
lift every filter and clamp in the FMFC/FHEOL loop to airframe- and
GCS-controllable state on both stacks.

## Part A — what was actually wrong

The 0fb345ae2 port swapped `alpha_fi` for the live 3x3 Tangent form and
changed nothing else. But the Tangent variant changes the linear bracket's
**input space**: `u` is `[d1, d2, dT]` — radians of tilt-plane increment about
the reference body x/y axes, newtons of collective increment — not a force in
NED. The old code handed that straight to `flatness_quad_force_transform`, so
two of three slots were radians read as newtons.

The chain now, per tick, matching `system_5673.xml`:

1. `R_ref` from `float_rmat_of_eulers_312(nom.phi, nom.theta, sp.heading)`;
   rows are `xb, yb, zb` in NED. `T_ref = -nom.thrust`.
2. `alpha = (alpha_fi/m) [T*yb, -T*xb, -zb]`  (unchanged from the port)
3. `u* = [0, 0, T_ref]`
4. bracket → `du = [d1, d2, dT]`
5. `du_saturate` — radial tilt clamp and thrust interval, both toggleable and
   both off by default; the **positive thrust floor is always on**
6. `zb_cmd = zb_tip(triad, d1, d2)` — exact Rodrigues, no small angle
7. `fi_c = -(T_ref + dT) * zb_cmd` → flatness transform

### Two departures from the brief, both measured

**Step 8 (`quad_du_from_fi` as `u_prev`) is implemented but defaults OFF.**
In `fmfc_quad.slx` the Tangent subsystem's `du_app` → `Mux_du_app` chain ends
in an **unterminated line**: `Mux_du_app` (SID 5708) out:1 carries `Points`
and no `Dst`. The estimator's `u_prev` port (`Subsystem` 5710 in:2) is fed by
`LPF2(Memory(delta_u_fi))` — the *commanded*, pre-saturation increment, which
is what the firmware already did. The `du_app` path is staged in the model,
not active in it. Turning `fheol_use_du_app` on makes the firmware do
something the ground truth does not. (`LPF` (5702), the `fc_g` filter on the
estimator drive, is likewise dangling in that subsystem.)

**Nominal dropout synthesises a nominal rather than holding the bracket in
reset.** The coupled branch also runs during Takeoff and Standby, where
`nom_valid` is false. Resetting there leaves the linear loop with pure
feedforward and no feedback: measured `z` stuck at **+0.39 m** through takeoff,
`a_c_z` wound to **160 m/s²**, `fi_c_z` spiking to **-84 N** at engagement,
divergence at t = 22.8. So the dropout path now builds a level reference at the
thrust `guidance_v` implies, floored positive. That keeps **one
parameterisation for the whole flight**, which is the constraint the brief was
protecting — `u` is `[rad, rad, N]` under Tangent and `[N, N, N]` under
Force_Setpoint, and alternating them makes the estimator's `u_prev` history
meaningless.

## The root cause of the original crash was not in the C

`anton_fheol.xml` shipped `FILT_CUTOFF = 250.0` in **both** the
`STABILIZATION_FHEOL` and `GUIDANCE_FHEOL` sections, against
`PERIODIC_FREQUENCY = 500`. That is *exactly Nyquist*, where a 2nd-order
Butterworth attenuates essentially nothing. A 250 Hz alternation passed
through `accel_ned_lowpass` and closed a loop via `ka_z = 0.892` into `a_c_z`,
giving a sustained two-cycle: `fi_c_z` alternating **-1.6 / -15.1 N** every
sample. The mean is hover, so `z`, `vz` and every reference looked clean.

Detector: mean `|x[k]-x[k-1]|` over mean `|x[k]-x[k-2]|` on `fi_c_z`.

| cutoff | ratio |
|---|---|
| 250 Hz | 712 |
| 50 Hz  | 0.53 |

The file's own comment already said "FILT_CUTOFF is 50 Hz here", and
`flat_mfc_quad_params.m` runs `fc_g = 50`. Fixed to 50.

**`anton_fmfc.xml` still carries 250.0 with the same contradicting comment.**
The Force_Setpoint stack tolerates it and still tracks the circle at RMS
0.22 m, so it was left alone rather than changed under a brief that forbids
tuning around a miss. **This is a decision for the author.**

## Parts B and C

Seven filters, each with its own cutoff define and its own runtime enable, on
both stacks — `FC_ACCEL`, `FC_RATE`, `FC_POS`, `FC_ZETA_E`, `FC_ACT_FI`,
`FC_ACT_M`, `FC_ACT_OBS`. Every default comes from the shared cutoff it
replaced. Cutoffs are live: a bank is re-initialised in place when its cutoff
moves, carrying its current output across so a GCS change does not step the
signal. An enable set false is a **passthrough, never a zero**, and the filter
keeps being stepped so flipping back is bumpless.

Two signals needed more than a plain passthrough:

- `dw` is a **difference of two filter states**, not a filter output, so its
  bypass differences the raw rate against its own previous sample.
- `u_prev` must stay **one tick delayed** when unfiltered, so the previous
  tick's raw `du` is shadowed. Reading the current tick's raw `du` would close
  the estimator's delayed-command lane with no delay at all — a different loop,
  not an unfiltered version of the same one.

Part C restores the correction rails and the tilt clamp from Stage 2's
unconditional `#if 0` to compiled-in code behind runtime guards, **all
defaulting false**, so the default build still behaves as Stage 2. FHEOL adds
the `quad_du_saturate` radial clamp and thrust interval. The actuator hard
clamps at the commit block are untouched and not exposed — pin-level
protection, not a controller choice.

No airframe overrides were added: every default reproduces existing behaviour.

## Acceptance

**ANTON_FHEOL**, Flat circle4 @ 4 m/s, one lap, t = 23.5..27.5
(`sim_logs/mfc_sim_20260903_213239.csv`):

| metric | measured | accept |
|---|---|---|
| horizontal RMS | 0.230 m | < 0.35 |
| tilt peak | 45.3° | < 50 |
| `u_sat_frac` | 0 | 0 |

Stable for the full 91 s; hover holds z = -2.00 m.

**ANTON_FMFC regression**, same window:

| build | RMS [m] | tilt [°] | `u_sat_frac` |
|---|---|---|---|
| pre-change | 0.2212 | 44.70 | 0 |
| post, run 1 | 0.2212 | 44.70 | 0 |
| post, run 2 | 0.2212 | 44.70 | 0 |
| post, run 3 | 0.2214 | 44.70 | 0 |
| post, run 4 | 0.2212 | 44.70 | 0 |

Well inside the 0.05 m / 2° band. `ANTON_FHEOL`, `ANTON_FMFC` and
`Hoops_111_FMFC` all build `nps`.

## Traps hit

- **NPS is not bit-reproducible run to run.** Two runs of the *same* binary
  differ in 42421 of 43430 common timestamps. Regressions must be compared as
  metrics inside a band, never as traces. A trace diff earlier in this session
  appeared to show a bit-exact match and was wrong.
- **A new `dl_setting var=` needs an `extern` in the module `.h`.** The
  generator emits `settings.h` referencing the bare symbol, and the failure
  reads as `'fmfc_fc_accel' undeclared` *in generated code*. No `Xml_error`
  was involved this session; `xml.dom.minidom.parse` on the module XML is a
  cheap pre-check for the malformed-XML class.
- **`oneloop_fheol.c` at HEAD does not compile.** It writes
  `fheol_log.fheol_fi_star` while `struct FlatLog` has `fmfc_fi_star`; the
  working tree carried the uncommitted rename. `git stash` to get a "clean
  baseline" therefore yields a tree that fails to build for an unrelated
  reason. Build baselines with `git show HEAD:<file>` into a scratch copy.

## Left for the author

1. **`anton_fmfc.xml` `FILT_CUTOFF = 250` at `PERIODIC_FREQUENCY = 500`** —
   same latent Nyquist defect, same contradicting comment. Not changed.
2. **Uncommitted doc reverts in the working tree, not from this session.**
   `conf/airframes/ENAC/quadrotor/anton_fheol.xml` and
   `conf/modules/oneloop_fheol.xml` both carry pending edits that revert their
   FHEOL/Tangent descriptions back to the FMFC constant-diagonal text, which
   now contradicts the code. Both commits here staged **only** their own hunks
   and left those edits pending.
3. **`Flat circle4` completes one lap (~4.2 s) and returns to hover.** That is
   the trajectory's own length, not a fault, but it means the acceptance
   window is short. A longer trajectory would test tracking better.
4. **The `paparazzi` submodule pointer was NOT bumped.** `paparazzi_dev` HEAD
   still records `29c773861`, which is 11 commits behind the submodule's
   `sitl-estimator-lib` HEAD — 9 of those predate this session (Stages 1–4 and
   the standalone estimator library) and were never bumped either. Bumping now
   would sweep that pre-existing work into a commit from this session, so it
   was left for the author's next deliberate `Bump paparazzi:` commit. The two
   commits from this session, `8d3850f8e` and `0e049b769`, are safe in the
   submodule's own history.
