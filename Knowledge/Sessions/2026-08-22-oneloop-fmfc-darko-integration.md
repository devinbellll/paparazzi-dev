# 2026-08-22 — `oneloop_fmfc_darko`: the HEOL/MFC brackets on the Darko FINDI spine

## Task

Build `oneloop_fmfc_darko` for the DarkO tailsitter: the MIMO HEOL/MFC brackets
on the already-flying Darko FINDI spine. Keep the entire spine; replace only the
two INDI increments with 3-channel model-free brackets on `mfc_core_mimo` at
n = 3, with the nominal command added outside the correction.

**INTEGRATION, NOT EVALUATION.** The Simulink reference `fmfc_darko.slx` was
itself untuned and not flying as of 2026-08-18, so there is no tuned gain set to
inherit and no reference behaviour to compare against. The deliverable is: it
builds, it is structurally correct, it passes analytic property checks, and it
arms in SITL. **No performance number is reported and nothing was tuned.**

## Verdict, up front

- **The port is done and it is structurally checked.** 109 new analytic property
  checks pass, and the module builds clean for `nps` and for `ap` with no
  warnings on either.
- **It arms, takes off and stays up, and the vertical channel holds** — 80 s at
  6.4–7.6 m with the vertical residual inside ±0.62 m.
- **It does not hold position.** The commanded tilt sits on the 30° limiter
  **100 % of the settled window**, and the horizontal residual saturates around
  15–25 m and stays there. It is a bounded orbit, not a divergence, and not a
  hover.
- **That reproduces with every NPS noise source zeroed**, so it is structural.
  With noise on the configuration is **marginal**: of three noise-on runs, one
  stayed airborne for the full 80 s and two departed and landed within 20–40 s.
- **The flagged inner gain is not obviously the fix.** Two in-flight A/Bs to
  `findi_darko`'s measured (25, 7.0) both went inverted within ~5 s of the
  change. Both applied it during a transient, so the comparison is not clean —
  but it does not support "just put the FINDI gains back", and per the brief
  nothing was retuned. **Reported, not tuned.**

Gate: the structure is built and verified; the behaviour question is open and
belongs to whoever tunes the Simulink side. When that lands, the firmware side
is a header edit.

## The `TRANSFORM_V_SCALE = 0` caveat — stated explicitly

`darko_fmfc.xml` ships `GUIDANCE_FMFC_DARKO_TRANSFORM_V_SCALE = 0.`, and every
SITL number in this note comes from a build with it.

That is **not the published law.** At 0 the flatness force transform reduces to
pure thrust vectoring — the attitude that points the commanded force, with no
wing lift credited. It is hover-only, plant-agnostic, and it cannot transition.

The reason is inherited verbatim from `findi_darko` and is not something this
session could fix or should have tried to: **`darko.xml` simulates JSBSim
`cyclone`, which is not DarkO.** Mass 1.089 vs 0.492 kg, hover roll/yaw inertia
~2.9×, and — the one that matters — wing lift at 5 m/s of **0.74 N against the
DarkO model's 8.96 N, a factor of 12.** The force transform is an *open-loop
aerodynamic inversion* sitting downstream of both loops, so no increment and no
estimator protects it: `F_hat` can only correct what the loop closes around.
With the published law (`= 1`) `findi_darko` diverged past 110 m, **with the
noise sources zeroed as well as with them on.** The transform reads `‖v‖`, so
the takeoff climb alone triggers it.

**Consequence, said plainly: the smoke test below exercises the two brackets,
the attitude law and the sequential allocation. It does NOT exercise the
aerodynamic half of the flatness inversion, in either direction.** Nothing here
says the published law fails on the real DarkO; it says this simulator cannot
test it. Building a JSBSim DarkO model was explicitly out of scope.

## What was built

New files:

- `sw/airborne/firmwares/rotorcraft/oneloop/oneloop_fmfc_darko_law.h` — the PURE
  half: the 3-channel HEOL/MFC bracket, the two constant diagonal α matrices,
  the two nominal flat inputs. Includes `oneloop_findi_darko_law.h` and reuses
  its frame maps, attitude helper, actuator maps and tilt limiter **verbatim** —
  those are properties of the airframe, not of the control law, and duplicating
  them is how the two stacks would drift.
- `sw/airborne/firmwares/rotorcraft/oneloop/oneloop_fmfc_darko.{c,h}` — the
  module.
- `conf/modules/oneloop_fmfc_darko.xml`
- `conf/airframes/ENAC/hybrid/darko_fmfc.xml` — twin of `darko_findi.xml`,
  identical outside the control stack.
- `tests/oneloop_fmfc_darko_test.c` + `tests/run_oneloop_fmfc_darko.sh` — 109
  analytic property checks.
- `tests/stubs_darko/mcu_periph/sys_time.h` — clock stub only.
- `DARKO_FMFC` (ac_id 29) added to `conf/userconf/ENAC/conf_mfc.xml`. Purely
  additive; a second worker was adding the quad `oneloop_fmfc` entry to the same
  file concurrently.

Nothing outside that list was modified. The quad `oneloop_fmfc.*`,
`anton_heol.xml`, `guidance_heol.c`, `mfc_core_mimo.*` and `heol_mimo.*` were
**not touched**.

## The delta, precisely

The entire FINDI spine is kept — applied wrench, flatness force transform,
quaternion attitude error, sequential allocation, mirrored-elevon commit, the
oneloop dispatch. Only the two increments change:

```
FINDI linear   f_i^c = f_i^prev + m (a_c - a_tilde)
FMFC  linear   f_i^c = f*       + alpha_fi^-1 (-F_hat_fi - fb_fi)

FINDI angular  m_c   = J (Omdot_c - Omdot_lpf) + m_lpf
FMFC  angular  m_c   = m*       + alpha_m^-1  (-F_hat_m  - fb_m)
```

with

```
alpha_fi = (1/m) I3        f* = m (a_ref - g e_z)
alpha_m  = inv(J)          m* = J Omdot_ref        (= 0 at hover)
```

Both α matrices are **constant, diagonal and plant-derived**. They are set once
at init, routed through the core's own `mfc_mimo_set_alpha()` so the
determinant guard and the closed-form 3×3 cofactor inverse are exercised, and
they are not tuning parameters. There is no conditioning question — unlike the
HEOL horizontal channel's anti-diagonal Jacobian, these are fixed positive
diagonals, because the flatness map sits *outside* this loop.

The applied wrench is **still computed every tick** but no longer enters either
command: `delta` is a real input to the force transform, and `m_lpf` against
`m_c` is the one log signal that separates "the estimator is wrong" from "the
loop is wrong".

## Six things that were not obvious

### 1. The angular bracket's own PD **is** eq. (tsPD) — so the tsPD helper is not called

With `eps = -zeta_e` and `d(eps)/dt = Om_lpf - Om_ref`, the decoupled core's
feedback term is

```
-fb = -(kp eps + kd d(eps)/dt) = kp zeta_e + kd (Om_ref - Om_lpf)
```

which is exactly `Omdot_c = K_xi zeta_e + K_Om (Om_ref - Om_lpf)` for
`kp = K_xi`, `kd = K_Om`. So `findi_darko_tspd()` is deliberately **not**
called: computing the same quantity in two places is how two places drift.

This is not a coincidence — it is the whole reason an MFC bracket can be dropped
onto the FINDI attitude law without changing the law. The harness asserts it
numerically against `findi_darko_tspd()` on three axes, with a nonzero `Om_ref`,
and with a discriminating case that shows an inverted derivative sign breaks it.

**The law is the single PD, not a cascade.** The canonical `.tex` prints a
two-stage cascade and it is wrong (author's decision, 2026-08-21).

### 2. `ka` has no slot in an MFC bracket, and that is structural

The FINDI outer loop has three feedback paths:
`a_c = a_ref + kx² e_p + 2 zeta_x kx e_v + ka e_a`. The decoupled MFC core has
two (kp, kd) plus an integral. `kp = kx²` and `kd = 2 zeta_x kx` map straight
across; **the acceleration-error path does not.**

That is not an omission. `F_hat` *is* the algebraic estimate of the residual
acceleration, and `-F_hat` enters the same right-hand side `ka e_a` used to.
Feeding `ka` in as well would put two estimators of one quantity in series.

**No `ka` knob is defined anywhere in this stack.** `flat_mfc_darko_params.m`
carries `ka = 0.2062`; if the Simulink reference genuinely keeps an
acceleration-error path, that is a **structural difference to reconcile**, not a
gain to copy across. Flagged as an open question for the Simulink side.

### 3. Gravity belongs in `f*`, not in `F_hat`

Both "work" — an algebraic estimator can absorb a constant. But with gravity in
`f*` the estimator's true value in the nominal plant is **identically zero**,
which is a property a test can assert and a log can be read against. With
gravity in `F_hat` it would carry 9.81 m/s² of known physics and the residual it
exists to find would be a couple of percent of its own magnitude. Checked both
ways in the harness: `f*` reproduces `a_ref` exactly through the nominal plant,
and `m*` reproduces `Omdot_ref`.

### 4. `est_use_presat_command` is set explicitly on both brackets

`false` on both — the estimator is fed the command the plant actually received,
after the clamp. That is also the core's default, and the harness asserts the
core default separately so a change there fires here rather than in flight.

Setting it at the call site is not defensive style: a module-XML define
shadowing the core default is what hid the HEOL divergence defect for three days
(`cfede3078`). It is now visible in three places — the airframe XML
(`M_EST_PRESAT` / `FI_EST_PRESAT`), the `#ifndef` block, and `init_brackets()`.
Grepped: nothing else in the tree defines either name.

The harness **discriminates** it rather than merely reading it back: it drives a
channel into a clamp that actually bites, asserts the clamp bit, and asserts the
two taps then differ. A check that cannot tell them apart would be vacuous.

### 5. The clamp bounds the TOTAL, and it is applied inside the core

`sat(u* + du)` over `[lo, hi]` is `u* + sat(du)` over `[lo - u*, hi - u*]`, and
the window is rewritten every tick before `mfc_mimo_run()` — because that is the
only place the applied-command tap and the integral freeze can observe it in the
same sample it bites. Same argument and the same code shape as `heol_mimo.c`.

**Finding, small but worth recording:** the `lo > hi` "collapse" branch in
`heol_mimo.c` (and in the copy here) is **unreachable**. It reduces to
`u_min > u_max`, not to "u_ff is outside the range" as its comment claims — a
`u_ff` outside the bounds simply produces a shifted window that forces the
correction to pull it back in, which is already correct. Kept as a guard against
a caller that inverts its own bounds, with the comment corrected and a harness
check recording that the normal path gets the answer right without it.

### 6. The "a value offset is absorbed" argument is **weaker** under MFC than under INDI

`W_FULL_CMD` and `D_FULL_CMD` are calibrated by slope, not by value, and
`findi_darko`'s justification is that "a value offset is absorbed because the
increment integrates on the command". **That reasoning does not carry over.** An
MFC bracket does not integrate on the command; it leaves the offset in `F_hat`,
where it is *carried* rather than cancelled. The slope is what matters either
way, but the absorption claim is now flagged as INDI-specific in three places.

This is visible in the log: `F_hat_lin,z` settles at **0.6–3.1 m/s²** in the
noise-off run rather than near zero, which is the 2.2× mass mismatch showing up
in the estimator instead of in an increment. `findi_darko` flew the same plant
mismatch with its measured gains untouched and needed no retuning; this
controller has to hold it in `F_hat`. **That is a real structural difference
between the two stacks and it is a plausible contributor to what follows.**

## Bring-up ladder

### Rung 1 — analytic property checks: 109, all pass

`./tests/run_oneloop_fmfc_darko.sh`. There are no golden traces and none are
coming — no MATLAB here, and the Simulink reference was untuned and not flying,
so there would be no reference behaviour even with it. Every check is
**structural**: true of the controller regardless of what the gains turn out to
be.

| group | checks | what it discriminates |
|---|---|---|
| [1] bracket | 17 | `epsilon = MEASURE − REFERENCE` (opposite to FINDI's outer loop), the nominal added outside, the estimator tapping the correction alone, and a zero residual passing `u_ff` through exactly |
| [2] presat tap | 6 | the core default, that `bracket_init` does not overwrite an explicit choice, and — with a clamp that actually bites — that the two taps really differ |
| [3] α matrices | 15 | `I/m` isotropic and `inv(J)` anisotropic, through the core's own setter, exactly diagonal, det far from the guard |
| [4] nominal inputs | 15 | `f*`/`m*` reproduce `a_ref`/`Omdot_ref` exactly in the nominal plant; `m*` linear in `Omdot_ref` with no gyroscopic term |
| [5] the tsPD identity | 14 | the bracket reproduces `J · findi_darko_tspd()` on all three axes, survives a nonzero `Om_ref`, and breaks under an inverted derivative sign |
| [6] clamp | 5 | the bound is on the TOTAL, both rails, and the window is never inverted |
| [7] estimator | 10 | the measured windows (500 / 20) and hold times (0.5 / 0.5), `F_hat` pinned before the hold time and live after, and the per-element-numerator / **shared-scalar-denominator** asymmetry (three channels driven 1:2:4 come out 1:2:4) |
| [8] n = 2 unaffected | 5 | a 2-vector trace is **bit-identical** with a 3-vector channel live in the same program — `memcmp`, verified not assumed — plus the anti-diagonal `alpha` the horizontal channel actually runs |
| [9] hover chain | 14 | `f*` → transform → attitude error → bracket → allocate reproduces the trim rotor speed and zero flaps |
| [10] AERO frame | 8 | the residual is fed in the AERO frame; a hover-ROLL error comes out as differential THRUST on rotor 1 (LEFT), a hover-PITCH error as common-mode flap |

They caught nothing during the port, which is the point: it meant the SITL
behaviour below could be attributed to control within minutes rather than hours.

### Rung 2 — builds

Clean for `nps` and for `ap`, **no warnings on either**, re-verified with the
sources touched so the compiles actually reran. `ap` links at 372 816 text /
11 216 data / 490 152 bss.

### Rung 3+ — SITL smoke test

Five runs this session, all `DARKO_FMFC`, JSBSim `cyclone`,
`TRANSFORM_V_SCALE = 0`, flight plan `rotorcraft_basic`. Logs under
`sim_logs/fmfc_darko/`.

| run | gains | noise | outcome |
|---|---|---|---|
| `fmfc_shipped_noise` | (4, 6) shipped | on | **airborne the full 80 s.** Motors on t = 6.5 s, climbed to 7 m, altitude held 6.3–7.5 m for the rest of the run |
| `fmfc_shipped_nonoise` | (4, 6) shipped | **off** | **airborne the full 80 s**, altitude 6.4–7.6 m — the same picture as above |
| `fmfc_setlate_noise` | (4,6) → (25,7) at t ≈ 40 | on | descended and landed at t ≈ 38, *before* the setting arrived. A/B void |
| `fmfc_setmid_noise` | (4,6) → (25,7) at t ≈ 20 | on | went inverted and landed at t ≈ 18, *before* the setting arrived. A/B void |
| `fmfc_k25_7_from_climb_noise` | (25, 7) from t ≈ 8 | on | inverted within ~5 s of the change and landed by t ≈ 13 |

#### What the two airborne runs show

Settled windows, noise-off run (`fmfc_shipped_nonoise`), 25–75 s:

- altitude −6.4 to −7.6 m; vertical residual `|eps_z|` ≤ **0.62 m** throughout
- horizontal residual `|eps_h|` **11.5–15 m rms, peak 20.3 m** — flat across
  every 10 s window, neither growing nor shrinking
- **commanded tilt on the 30° limiter 100 % of samples**
- roll band ±0.43 rad, pitch band ±0.40 rad, both steady
- pitch moment command on its ±1 N·m bound 14 % of samples
- pitch attitude error `|zeta_e_y|` peaks at 1.3 rad (75°)
- slow wander: ~0.20 Hz in pitch, ~0.08 Hz in roll

The noise-on run is the same to within run-to-run scatter (rms 12.6–16.4 m,
tilt railed 100 %, `m_c_y` railed 14 %).

**Read: the vertical channel works and the horizontal one is saturated open.**
The attitude loop cannot follow the attitude the transform commands — 75° of
peak pitch error — so the horizontal force never points where the outer loop
wants it, position error grows until the tilt limiter saturates, and the whole
thing settles into a bounded 15–25 m orbit. It is not a divergence and it is not
a hover.

**Noise is an axis, and it was tested, not assumed.** The picture reproduces
with every stochastic source zeroed, so it is **structural**. What noise adds is
run-to-run variability: with noise on, the configuration sometimes tips into a
departure inside 20–40 s.

#### The ±1 N·m moment clamp is not the cause

`m_c_y` sits on it 14 % of the time, but the clamp is inert as a *limit*: the
flaps deliver ~0.09 N·m at `d_max`, so a 1 N·m demand is already an order of
magnitude past the airframe's pitch authority and the allocator saturates long
before the clamp does. Raising it would change nothing about the applied moment.
It exists as a runaway guard on `F_hat` (which an incremental law does not need)
and as the thing that makes the `M_EST_PRESAT` choice observable at all.

#### Pitch is not attributable, and was not chased

The DarkO baseline under stock INDI carries an unresolved **~1.2 Hz ±35° pitch
limit cycle** that reproduces noise-off. Pitch is also exactly the axis where
the unmodelled `PHI_mv` wing pitching moment lives. The ~0.2 Hz pitch wander
seen here is a *different* frequency, so it is probably not the same phenomenon
— but per the brief the baseline limit cycle was not investigated and no pitch
number here is offered as attributable.

## Gains — all placeholders, and the one that moved

From `flat_mfc_darko_params.m`, used unchanged, **not searched, not tuned**:

```
outer   kx = 1.8898   zeta_x = 1.1093        (ka = 0.2062 has no slot — see above)
inner   k_xi = 4      k_om = 6               -> wn 2.00 rad/s, zeta 1.50
```

**FLAGGED:** `findi_darko`'s inner loop is the **measured** (25, 7.0) = wn 5,
zeta 0.7, from a 2026-08-12 hand sweep whose usable band is narrow:

| (k_xi, k_om) | wn | zeta | result |
|---|---|---|---|
| (12, 1.5) | 3.46 | 0.217 | **diverged**, peak \|e\| 93 m |
| (25, 7.0) | 5.00 | 0.700 | peak \|e\| 0.0204 m, rms 0.0083 m |
| (121, 16.5) | 11.0 | 0.750 | bounded, large pitch limit cycle |

FMFC's (4, 6) is slower and much more damped than any row of that table. It is
the first thing to suspect, and the peak 75° attitude error is consistent with
an inner loop that cannot keep up — **but the two in-flight A/Bs to (25, 7.0)
both went inverted within ~5 s**, and both applied the change during a transient
(a climb, or an already-departing aircraft), so neither is a clean comparison.

So: **the flagged gain is a live suspect and not a demonstrated fix.** Per the
brief it was reported and not tuned. Nothing in the committed configuration
changed; the A/Bs were live GCS settings on an unmodified build.

The estimator windows and hold times are **not** placeholders — they are
measured constants used verbatim:

```
FFilter_fi = 500   F_hold_time_fi = 0.5     linear/force bracket
FFilter_m  = 20    F_hold_time_m  = 0.5     angular/moment bracket
```

Nothing was inherited from the HEOL horizontal channel — different plant,
different units, different α.

## What is next

1. **The Simulink side has to tune first.** The structure is now in firmware and
   verified; searching gains against it here would produce a number with no
   reference behind it. When `fmfc_darko.slx` tunes up, the firmware side is a
   header edit — `STABILIZATION_FMFC_DARKO_K_XI/K_OM` and
   `GUIDANCE_FMFC_DARKO_KX/ZETA_X` in `darko_fmfc.xml`.
2. **Resolve `ka`.** Either the Simulink model has no acceleration-error path
   (in which case `ka` is vestigial in `flat_mfc_darko_params.m`) or it has one
   and the firmware bracket is structurally short a term. This is a question for
   the model, not for the firmware.
3. **The mass-error-in-`F_hat` question.** `findi_darko` absorbed the 2.2× plant
   mass mismatch in its increment and flew on measured gains untouched; this
   controller carries it in `F_hat` (observed 0.6–3.1 m/s²). Worth checking
   whether the horizontal saturation survives on a plant whose mass matches the
   model — which needs the same JSBSim DarkO model item 4 needs.
4. **A JSBSim DarkO model** remains the precondition for testing the
   aerodynamic half of the inversion at `TRANSFORM_V_SCALE = 1`, on either
   stack. Unchanged from the `findi_darko` session.
5. **`fmfc_darko_linear_enabled = false`** (attitude-only, rungs 2–3) is wired
   and untested. With the datalink RC and no stick script the attitude-only path
   gets a zero thrust setpoint and simply sits; drive it with `--rc_script 0`.
   Given that the failure here is plausibly in the attitude loop, this is
   probably the *first* thing the next session should do.

## Environment notes

- **`WORKSPACE_DIR` must point at `paparazzi_dev/`.** `export WORKSPACE_DIR=$PWD`
  from the repo root. Set `CONF=conf/userconf/ENAC/conf_mfc.xml` too — the
  default fleet XML does not contain `DARKO_FMFC`.
- **Two concurrent sims corrupt each other's logs** (hard-coded NPS scope port
  9871, IVY 2010, `--network host`). A second worker was active in this repo all
  session; every run here waited on
  `docker ps --filter ancestor=paparazzi-build:latest` being empty first, and
  every CSV was checked for time backsteps before being read (0 in all five).
- **`--set` is applied AFTER the whole `--nav` sequence completes.** Useful as a
  live in-flight A/B if the sequence is kept short — `--nav "Start Engine,
  Takeoff"` lands the setting at t ≈ 8 s, since the Takeoff block auto-advances
  to Standby. It cannot configure anything *before* the flight; that needs the
  airframe XML and a rebuild.
- **`--ic` is known broken. Do not use it.**
- `tests/stubs/` cannot be used for anything that links the Darko spine: it
  stubs `math/pprz_algebra_float.h` down to a single `float_vect_zero()` and
  would shadow the real quaternion algebra. `tests/stubs_darko/` stubs the clock
  and nothing else.

## Diagnostic modifications — all reverted

One diagnostic build this session: `NPS_NOISE_SCALE` flipped `1.` → `0.` in
`darko_fmfc.xml` for the noise-off run, then **flipped back to `1.` and
rebuilt**. The committed airframe is the one the noise-on runs used. No other
term was commented out, zeroed or bypassed, and no gain in any committed file
was changed.
