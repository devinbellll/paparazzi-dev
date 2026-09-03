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

**`anton_fmfc.xml` carried the same 250.0.** It was measurably firing there
too, just not destructively — see below. Fixed the same way in `8b60bf90d`.

### Follow-up: 250 Hz is not a passthrough, it is a Nyquist oscillator

The author's intent for 250 was "make this filter a passthrough", which is what
the reference does — `LPF` (SID 5702), the `fc_g` filter on the estimator
drive, is dangling in the Tangent subsystem. But `init_second_order_low_pass`
builds `K = tan(pi*fc/Fs)`, so `fc = Fs/2` gives `K = tan(pi/2)` and the
coefficients degenerate to `b = [1,2,1]`, `a = [2,1]` — a **double pole exactly
at `z = -1`**. The zeros cancel it, so from a consistent state the signal does
pass; but the homogeneous mode is `(A + B n)(-1)^n`, which grows linearly and
flips sign every sample. One rounding error seeds it and nothing damps it.

Measured on the difference equation, single 1e-7 perturbation:

| sample | error, fc = 250 | error, fc = 50 |
|---|---|---|
| 6   | -2.0e-07  | +5.0e-02 |
| 50  | -4.6e-06  | +1.2e-10 |
| 200 | -1.96e-05 | ~0 |
| 399 | +3.95e-05 | ~0 |

So the fix is the **enable flag**, which is a real bypass, not a cutoff at
Nyquist. `anton_fheol.xml` now states
`USE_FC_ACCEL/RATE/POS/ZETA_E/ACT_OBS = FALSE` with the cutoffs left at 50 as
the value a re-enabled filter would use, and `USE_FC_ACT_FI/ACT_M = TRUE`
(the one filter the reference actually wires, `LPF2`). Commit `bb37ebe96`.

Re-measured in that configuration: RMS **0.226 m**, tilt peak **45.0 deg**,
`u_sat_frac` **0**, alternation ratio **0.72**. The loop is fine without that
filtering — it was the degenerate filter, not the absence of one.

### Clamp state, confirmed

No clamp engages anywhere in the 91 s run. `|fi_c|` stays in 2.94 .. 15.68 N
against the always-on positive floor of 1e-6 N; the only actuator rails are on
the ground before arming and 105 samples during the takeoff climb, none while
tracking. Peak tilt is 45 deg against `MAX_BANK = 30`, so enabling
`USE_TILT_CLAMP` **would** clip this trajectory.

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

1. ~~`anton_fmfc.xml` `FILT_CUTOFF = 250`~~ — **fixed** in `8b60bf90d`, same
   way as FHEOL. It was oscillating at ±0.76 N tick to tick (ratio 38.9 in
   hover, 11.7 in the circle); after the fix, 0.71 / 0.72. Tracking is
   unchanged inside the band (RMS 0.2205–0.2212 over three runs against 0.2212
   before) and peak actuator command drops ~8%, from 3268 to ~3000.
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


## Addendum: the same defect on ANTON_FMFC

`anton_fmfc.xml` carried `FILT_CUTOFF = 250` at `PERIODIC_FREQUENCY = 500` as
well, and it was **not** benign there either — it just never grew enough to
destabilise the Force_Setpoint loop:

| window | before | after |
|---|---|---|
| hover, t = 16..22 | 38.93 | 0.71 |
| circle, t = 38..42 | 11.65 | 0.72 |

±0.76 N of tick-to-tick oscillation in hover, against the ±7 N (ratio 712) that
tumbled the Tangent loop. Fixed the same way — cutoffs at 50 as the value a
re-enabled filter would use, `USE_FC_ACCEL/RATE/POS/ZETA_E/ACT_OBS = FALSE` for
a true bypass, `USE_FC_ACT_FI/ACT_M = TRUE`. Three runs after: RMS
0.2205/0.2208/0.2212 m, tilt 44.90/44.91/44.70°, no saturation, and peak
actuator command down from 3268 to ~3000 — consistent with removing an
oscillation rather than with a gain change. Commit `8b60bf90d`.

**Both airframes are now clean of the Fs/2 cutoff.** Worth grepping any new
airframe for `FILT_CUTOFF` against its `PERIODIC_FREQUENCY` before first flight;
this one is invisible in the mean and does not announce itself.

## Addendum: filters enabled, and Hoops_111_FHEOL

### Filters turned back on (`5b2c2c3b5`)

`USE_FC_ACCEL / RATE / POS / ZETA_E / ACT_OBS` set TRUE on `anton_fheol.xml`
and `anton_fmfc.xml`, cutoffs unchanged at 50 Hz. This reverses the passthrough
setting from `bb37ebe96` / `8b60bf90d`. Both configurations track, and they sit
within run-to-run spread of each other:

| aircraft | filters on | bypassed |
|---|---|---|
| ANTON_FHEOL | RMS 0.2295 m, tilt 45.04° | 0.226 m, 45.0° |
| ANTON_FMFC | RMS 0.2234 m, tilt 45.00° | 0.2205 m, 44.9° |

No alternation either way (ratio 0.54). The filters are not what makes or
breaks this loop — the degenerate 250 Hz filter was. Comment blocks in both
airframes were rewritten to describe the enabled state; leaving them would have
repeated exactly the code/prose contradiction flagged in the doc reverts.

### Hoops_111_FHEOL (`b08e1ed79`)

New airframe, `ac_id 181`, registered in `conf_mfc.xml` and given two control
panel sessions (`Flight Hoops_111 FHEOL`, NatNet rigid body 111 → 181, and
`FHEOL VM SIM`). Both `nps` and `ap` targets build.

**Hover is clean** (t = 25..45 s, settled): z −2.001 m with 13 mm spread,
93 mm horizontal drift, 0.31° tilt, zero saturation.

**The 4 m/s circle does not meet ANTON's numbers**, but neither does its FMFC
sibling on the same vehicle, gains and trajectory:

| aircraft | RMS [m] | tilt [°] | u_sat_frac |
|---|---|---|---|
| Hoops_111_FHEOL | 0.5044 | 54.85 | 0.0125 |
| Hoops_111_FMFC | 0.4795 | 53.66 | 0 |

So it is the Hoops gain set being marginal at 4 m/s, not a Tangent defect.
FHEOL is ~5% worse on RMS, 1.2° more tilt, and unlike FMFC it touches the
actuator rails for 1.25% of the window.

**Gain provenance, twice removed.** These are FINDI GA values carried to FMFC
and now to FHEOL, measured on neither, with every GA bound set on the
Force_Setpoint loop before the Tangent rework. Nothing in this airframe is a
tuned point on this structure. **Hover first tomorrow; do not open with the
4 m/s circle.**

### One more NPS trap

A failed build plus `sim.sh --no-build` silently runs the stale `.elf` and
reports plausible numbers for the wrong aircraft. It happened here: an
ANTON_FHEOL build died on the `gen_aircraft.ml` assertion and the run that
followed reproduced an earlier ANTON_FMFC result exactly (0.2212 / 44.70 /
max u 2988). Caught only because the figures were byte-identical to a previous
run. **Always confirm the build printed `==> Done` before trusting a run**, and
treat a repeated metric across supposedly different aircraft as a red flag.
The `gen_aircraft.ml` assertion is a failed `mv` of the temp makefile that
leaves a half-built aircraft dir; `./pprz.sh clean <AC> nps` clears it.
