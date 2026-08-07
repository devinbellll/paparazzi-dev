# Plan — MFC Decoupled Core Port (HEOL stage 1)

> **Scope.** Stage 1 of getting ANTON flying HEOL control with a flatness
> feedforward trajectory. This note is written to be executed by a session scoped
> to `paparazzi_dev/` alone. The MATLAB/Simulink model this is ported *from*
> (`MFC_SISO`) lives outside that mount, so everything load-bearing from it — the
> equations, the sign convention, the gain semantics — is restated here rather
> than cited. Paths are relative to the `paparazzi_dev/` root; firmware source is
> under `paparazzi/sw/airborne/`.
>
> Created 2026-08-06. Not yet executed.

## Context

The goal is to fly ANTON in HEOL control — flatness feedforward plus a model-free
correction — using the trajectory setpoint path added on `feature-diff-flatness`
(see [[Flatness Trajectory Setpoints (Pos-Vel-Accel-Jerk-Snap + Psi)]]). Three
things block that; this stage clears the first.

The upstream Simulink SISO model has been effectively rewritten since June and
now supports six controller variants.
`paparazzi/sw/airborne/firmwares/rotorcraft/stabilization/mfc_core.c`
(`mfc_siso_run`) implements exactly one of them: **2nd-order, coupled,
algebraic, no `Ki`**. So a tuning found in the model cannot be entered into the
firmware at all — which is why the model tuning was never validated on SITL, and
SITL still shows large x/y command oscillations.

### The one concept this whole change rests on

**"Decoupled" does not mean per-axis.** The firmware already has six independent
`MfcParameters` instances with their own gains and clocks. Decoupled means the
*estimator is decoupled from the feedback gains*:

| | **coupled** (what the firmware does today) | **decoupled** (what is being added) |
|---|---|---|
| estimator driven by | the tracking error `e` | the measurement `y` |
| what `F̂` means | plant dynamics **plus** the closed-loop polynomial — a hybrid quantity, not a disturbance estimate | the plant's own lumped disturbance — physically meaningful, plottable, comparable to an analytic value |
| `kp`, `kd` | **folded into** `F̂` as known coefficients | **explicit** in the feedback law |
| `ki` | explicit (never folded, in either structure) | explicit |

Coupled hands the estimator `ë = F + αu + a·ė + b·e` with `a = −kd`, `b = −kp`
as *known*, so the command needs no explicit P or D term. Its genuine advantage:
`kd` is applied without ever forming `ė`, so no raw differentiation of a noisy
error signal. Its cost: `F̂` is not `F`, and it moves when you retune.

**The classic failure mode** is applying `kp` twice — once folded into `F̂`, once
in an explicit feedback law — so the loop runs at double the proportional gain
you think you set. Nothing errors; it just behaves oddly. The `decoupled` flag
must gate *both* the drive signal and the folding, together, or this happens.

**Sign contract, non-negotiable:** `error = measure − setpoint` (measurement
minus **filtered** setpoint), and the command law **subtracts** `fb`. The folded
`−kp`, `−kd` are derived against this convention; feeding `setpoint − measure`
inverts the folded poles and the loop diverges. The firmware already gets this
right at `mfc_core.c:71` — do not "fix" it.

### HEOL is not in this firmware

`grep -rni heol` over the whole repo returns zero hits. HEOL exists only as a
Simulink variant subsystem in the model repo. Its structure, for reference:

```
ε    = ŷ − r                     (tracking residual around the feedforward)
u_fb = (−F̂ − PID(ε)) / b
u    = u_ff + u_fb
```

with the estimator fed **`ε` and `u_fb`** — the feedback component only, *not*
the total `u`. Building it is stage 2, as a **separate library with `mfc_core`
as a dependency** — see the sketch at the end. It needs the decoupled path first.

The intended outcome of *this* stage: a decoupled tuning from the model can be
typed into `anton_mfc.xml` or a GCS slider and mean the same thing on both sides,
with the coupled path preserved as a flight-tested fallback.

## Decisions already taken

1. **Adopt the model's gain convention.** `kp`/`kd` become the raw coefficients
   of `s² + kd·s + kp`. `use_Kd` is deleted. All existing gains converted once.
2. **`decoupled` is a per-axis runtime flag**, default `FALSE` (coupled), exposed
   as a `dl_setting`. The upstream model treats structure as a block choice, not
   a parameter; a flight controller that needs an in-flight A/B against a
   flight-tested tuning cannot afford that. Deliberate divergence.
3. **Scope: `mfc_core` + the standalone path.** `oneloop_mfc` gets converted
   gains so it keeps working, but no new settings.

## Critical files

| File | Role |
|---|---|
| `paparazzi/sw/airborne/firmwares/rotorcraft/stabilization/mfc_core.h` | `struct MfcParameters` — add fields, delete `use_Kd` |
| `paparazzi/sw/airborne/firmwares/rotorcraft/stabilization/mfc_core.c` | `mfc_siso_run()` — the whole change (130 lines today) |
| `paparazzi/sw/airborne/firmwares/rotorcraft/stabilization/stabilization_mfc.c` | `:533-558` roll/pitch/yaw init |
| `paparazzi/sw/airborne/firmwares/rotorcraft/guidance/guidance_mfc.c` | `:290-327` gx/gy/gz init |
| `paparazzi/sw/airborne/firmwares/rotorcraft/oneloop/oneloop_mfc.c` | `:587-615`, `:1380-1412` — convert, no new settings |
| `paparazzi/sw/airborne/firmwares/rotorcraft/guidance/guidance_indi.c` | `:266-267` — 4th consumer, `THRUST_MFC` path |
| `paparazzi/conf/modules/{stabilization_mfc,guidance_mfc,oneloop_mfc}.xml` | defines + settings panels |
| `paparazzi/conf/airframes/ENAC/quadrotor/{anton_mfc,hoops_111_mfc,anton_oneloop,anton_mfc_thrust}.xml` | gain conversion |

`grep -rl mfc_core.h paparazzi/sw/airborne/` confirms exactly four consumers —
`stabilization_mfc`, `guidance_mfc`, `oneloop_mfc`, `guidance_indi`. All four
must be converted; missing one silently mistunes that axis by orders of
magnitude.

## 1. `mfc_core.h` — struct changes

Delete `use_Kd`. Add:

```c
  /* Estimator structure. FALSE (default): coupled — the estimator is driven by
   * the tracking error and the closed-loop polynomial s^2 + kd*s + kp is folded
   * into F_hat, so the feedback path carries only ki. TRUE: decoupled — the
   * estimator is driven by the measurement with no folding, so F_hat is the
   * plant's own lumped disturbance, and an explicit iPD(I) law stabilizes.
   * This flag gates BOTH the drive signal and the folding — splitting them
   * applies kp twice and silently doubles the proportional gain. */
  uint8_t decoupled;

  float   z[3];          /* estimator drive history: error (coupled) or measure (decoupled) */
  float   int_err;       /* trapezoidal integral of the tracking error */
  float   ki;            /* integral gain; explicit in BOTH structures, never folded */
  float   est_hold_time; /* blank the estimate for this long after reset [s] */
```

Retype the `kp`/`kd` doc comment: they are now **polynomial coefficients of
`s² + kd·s + kp`**, not `wn`/`zeta`. `error[3]` stays — it is the tracking error,
needed for `fb` even in the coupled case (`ki`), and it is what telemetry logs.
In coupled mode `z` duplicates `error`; that is 12 bytes per axis (72 total)
bought for clarity, preferable to aliasing two meanings onto one buffer.

Update the file-header comment: the law is now `u = (−F̂ + ff − fb)/α`, and
"placing the closed-loop poles at s = −kp (double pole)" is no longer true.

`mfc_siso_init()`: set `decoupled = false`, `ki = 0.f`, `est_hold_time = 0.1f`.
`mfc_siso_reset()`: also zero `z` and `int_err`.

## 2. `mfc_core.c` — `mfc_siso_run()`

Five edits, in place. The reference filter (step 1) and the two IIR smoothers
(step 3) are **untouched** — they already match the model's smoother in the same
`(W² + 2W + 1)` denominator form, so that part of the port is already done.

Likewise the estimator numerator at `mfc_core.c:89-90` is already
character-for-character the model's 2nd-order algebraic kernel. Only its *input*
changes. Do not rederive it.

**(a) Drive signal.** After `error[0]` is computed at `:71`:

```c
  mfc_stt->z[0] = mfc_stt->decoupled ? mfc_stt->measure : mfc_stt->error[0];
```

**(b) Pole coefficients** — replace `mfc_core.c:75-76`:

```c
  /* Coupled: hand the estimator the closed-loop polynomial as known coefficients
   * (edd = F + alpha*u + a*ed + b*e).  Decoupled: nothing to fold. */
  float a = mfc_stt->decoupled ? 0.f : -mfc_stt->kd;
  float b = mfc_stt->decoupled ? 0.f : -mfc_stt->kp;
```

**(c) Estimator moments** — `mfc_core.c:78-90`: substitute `z[0..2]` for
`error[0..2]` throughout (`sde`, `s2d2e`, `sd2e`, `de`, `d2e`, `num`). `d2u`
keeps using `command[1]` — the command that actually reached the plant.

**(d) Hold time** — `mfc_core.c:104`: `(mfc_stt->time > mfc_stt->est_hold_time)`.
The `!= 0.0f` denominator guard on the same line stays.

**(e) Feedback, anti-windup, command** — replace `mfc_core.c:109-114`:

```c
  /* Trapezoidal integral of the tracking error (candidate — may be rolled back). */
  float int_err_cand = mfc_stt->int_err +
    0.5f * (mfc_stt->error[0] + mfc_stt->error[1]) * mfc_stt->sample_time;

  float fb;
  if (mfc_stt->decoupled) {
    float dot_err = (mfc_stt->error[0] - mfc_stt->error[1]) / mfc_stt->sample_time;
    fb = mfc_stt->kd * dot_err + mfc_stt->kp * mfc_stt->error[0] + mfc_stt->ki * int_err_cand;
  } else {
    /* kp and kd are already inside F_k — applying them here too would run the
     * loop at double gain. Only the integral is never folded. */
    fb = mfc_stt->ki * int_err_cand;
  }

  /* error = measure - setpoint, and the law SUBTRACTS fb. Inverting either
   * inverts the folded poles and the loop diverges. */
  float u = (-F_k + dot_dot_setpoint_trajec - fb) / mfc_stt->alpha;
  u = (u + (mfc_stt->command_filter - 1.f) * mfc_stt->command[1]) / mfc_stt->command_filter;

  /* Clamp, and freeze the integral in the same sample the clamp bites. */
  bool frozen = false;
  if (u > mfc_stt->u_max) { u = mfc_stt->u_max; frozen = true; }
  if (u < mfc_stt->u_min) { u = mfc_stt->u_min; frozen = true; }
  if (!frozen) { mfc_stt->int_err = int_err_cand; }

  mfc_stt->command[0] = u;
```

Add `z[2] = z[1]; z[1] = z[0];` to the history shift block at the end.

**Behaviour preservation:** with `decoupled = false`, `ki = 0`,
`est_hold_time = 0.1`, and converted gains, this is arithmetically identical to
today — `fb` collapses to 0, and `a`,`b` reproduce the old `use_Kd` expressions.

Leave `if (in_flight) {}` at `:116` alone; it is not this change's business.

## 3. Gain conversion (mechanical, must be complete)

Old: `a = −2·kd·kp` (`use_Kd = TRUE`) or `−2·kp` (FALSE); `b = −kp²`.
New: `a = −kd`; `b = −kp`. Therefore:

> **`kp_new = kp_old²`**  and  **`kd_new = 2·kd_old·kp_old`** (`use_Kd = TRUE`)
> or **`kd_new = 2·kp_old`** (`use_Kd = FALSE` or absent)

| Site | axis | old kp, kd, use_Kd | → new kp, kd |
|---|---|---|---|
| `anton_mfc.xml:216-234` | roll, pitch, yaw | 2, 1.5, TRUE | **4.0, 6.0** |
| `anton_mfc.xml:278-289` | gx, gy | 0.8, 1.5, TRUE | **0.64, 2.4** |
| `anton_mfc.xml:295-297` | gz | 4, 1.5, TRUE | **16.0, 12.0** |
| `hoops_111_mfc.xml:273-291` | roll, pitch, yaw | 2, 1.5, TRUE | **4.0, 6.0** |
| `hoops_111_mfc.xml:346-357` | gx, gy | 0.8, 1.5, TRUE | **0.64, 2.4** |
| `hoops_111_mfc.xml:363-365` | gz | 4, 1.5, TRUE | **16.0, 12.0** |
| `anton_oneloop.xml:193-201` | roll, pitch | 6, —, FALSE | **36.0, 12.0** |
| `anton_oneloop.xml:207` | yaw | 1, —, FALSE | **1.0, 2.0** |
| `anton_oneloop.xml:308-314` | gx, gy | 0.8, —, FALSE | **0.64, 1.6** |
| `anton_oneloop.xml:320` | gz | 4, —, FALSE | **16.0, 8.0** |
| `anton_mfc_thrust.xml:217` | thrust | 4, —, FALSE | **16.0, 8.0** |
| `stabilization_mfc.c:61-67` | roll | 6, 1.5, TRUE | **36.0, 18.0** |
| `stabilization_mfc.c:90-96` | pitch | 8, 5, TRUE | **64.0, 80.0** |
| `stabilization_mfc.c:119-125` | yaw | 1, 0.1, FALSE | **1.0, 2.0** |
| `guidance_mfc.c:58-64` | gx | 0.8, 0, FALSE | **0.64, 1.6** |
| `guidance_mfc.c:86-92` | gy | 0.8, 0, FALSE | **0.64, 1.6** |
| `guidance_mfc.c:121-127` | gz | 4, 0, FALSE | **16.0, 8.0** |
| `guidance_indi.c:266-267` | thrust | 2, 0 (hardcoded literal) | **4.0, 4.0** |
| `oneloop_mfc.c:82-368` `#ifndef` fallbacks | all six | per its own block | convert each |

`guidance_indi.c:266` hardcodes `kp = 2` with the airframe define commented out
— convert the literal, leave the commented define alone.

Delete every `*_USE_KD` define, `dl_setting`, and struct assignment across all
four module XMLs and all four `.c` consumers. `grep -rn use_Kd paparazzi/` must
return zero.

**Widen the setting ranges.** Existing bounds are `kp ∈ [0,20]`, `kd ∈ [0,10]`;
under the new convention pitch alone needs `kd = 80`. Set `kp ∈ [0,200]`,
`kd ∈ [0,200]`, `ki ∈ [0,50]`, step `0.1`.

## 4. Module XML wiring

For `stabilization_mfc.xml` and `guidance_mfc.xml`, per axis:

- New defines `<AXIS>_DECOUPLED` (default `FALSE`), `<AXIS>_INTEGRAL_GAIN`
  (default `0.`), `<AXIS>_EST_HOLD_TIME` (default `0.1`).
- New `dl_setting`s for `mfc_<axis>.decoupled`
  (`values="COUPLED|DECOUPLED"`, `type="uint8"`), `.ki`, `.est_hold_time` — all
  `persistent="true"`, matching the surrounding convention.
- **Rewrite the `KP`/`KD` `description=` strings.** They currently read
  *"natural frequency wn=kp"* and *"Damping ratio (zeta), used only when
  …_USE_KD=TRUE"*, which becomes actively wrong and will mislead whoever tunes
  next. Use *"closed-loop polynomial coefficient: s² + kd·s + kp"*.

`oneloop_mfc.xml` gets converted defaults and `use_Kd` removal only — no new
settings, per the scope decision.

`anton_mfc.xml`: add the three new defines per axis explicitly (even at their
defaults) so the structure choice is visible in the airframe rather than implied.

## 5. Out of scope, deliberately

- **`oneloop_mfc.c:1420-1421`** sets `mfc_gz.u_max = 0.f` and
  `u_min = MAX_PPRZ / GUIDANCE_MFC_THRUST_PPRZ_SCALE` = **+9600** with the
  default scale `1.f`. So `u_min > u_max`, the clamp order pins the gz command
  to +9600 every tick, and the inline comment `/* ≈ -57.6 */` does not match the
  expression. Real bug, but oneloop-only (`guidance_mfc.c:334-335` is correct)
  and ANTON_MFC does not use oneloop. **Log it to `.wolf/buglog.json` and leave
  it** — fixing it changes ANTON_ONELOOP flight behaviour, a separate decision.
  Note in the log that the new anti-windup makes it slightly worse: that axis
  now also freezes its integral permanently.
- `mfc_thrust_physical` hardcoded to nominal hover (`guidance_mfc.c:521`).
- 1st-order model, sliding-window estimator, live/matrix `α` — the model's other
  variants. Not needed for HEOL; add when a use appears.

## Verification

**1. Compile — the only automated correctness check this repo has.** From the
`paparazzi_dev/` root:

```bash
./pprz.sh build ANTON_MFC ap
./pprz.sh build ANTON_MFC nps
./pprz.sh build Hoops_111_MFC ap
./pprz.sh build ANTON_ONELOOP ap      # scope regression: must still link
./pprz.sh build ANTON ap              # guidance_indi THRUST_MFC path
```

**2. Numerical equivalence against the model's golden traces (optional but
strongly recommended).** `mfc_core.c` has only two external dependencies
(`get_sys_time_float`, `float_vect_zero`), so it compiles standalone on the host
with trivial stubs. A harness under `tools/` can then replay a reference trace
and compare `command[0]` and `estimator`.

The traces live **outside this repo**, in the MFC_SISO model repo at
`tests/golden/` — `2nd_decoupled_alg.csv` and `2nd_coupled_alg.csv` are the two
that matter. **Ask the user to copy them to `tools/mfc_golden/`**; do not attempt
to reach outside the mount. If they are unavailable, skip this step and lean
harder on step 3.

Match the config exactly or it will not agree: `use_trajec_sp` ↔ the model's
`use_ref_filter`, `time_trajec` ↔ `ref_filter_window`, `int_window` ↔
`est_filter_window`, `command_filter = 1`, `est_hold_time = 0.1`, `ki = 0`,
saturation wide open. Expect agreement to float rounding.
Diagnostics: if coupled matches and decoupled does not, the drive signal or the
`a`/`b` zeroing is wrong. If both drift *late*, suspect the `time` origin — the
firmware uses a wall clock latched in `mfc_siso_reset`, the model a sample
counter.

**3. Coupled regression in SITL — this is what proves the conversion table.**

```bash
./sim.sh                 # ANTON_MFC, default (coupled) settings
```

Fly the same profile as the last recorded run and compare against `sim_logs/`.
Trajectories should be indistinguishable from pre-change. If they are not, the
gain conversion is wrong, not the core — recheck §3 before touching `mfc_core.c`.

**4. Decoupled A/B in SITL.** Flip `mfc_gx.decoupled` / `mfc_gy.decoupled` to
DECOUPLED via the GCS settings panel and enter the model's tuning directly. The
specific thing to watch is the **x/y command oscillation** that motivated this
task. Log `mfc_gx.estimator` via `nps_scope`/PlotJuggler (`plotjuggler_mfc.xml`,
see [[08 - NPS Simulation Telemetry]]): in decoupled mode it should read as a
physical disturbance in m/s², roughly constant in steady hover, and — the real
test — **it should not move when `kp`/`kd` are retuned**. If it does, the
estimator is still error-driven somewhere.

**5. OpenWolf upkeep** (`.wolf/OPENWOLF.md`, mandatory): update `anatomy.md` for
changed files, append to `memory.md`, record the gain-convention change and the
coupled/decoupled semantics in `cerebrum.md` (Decision Log + Key Learnings), log
the oneloop gz clamp bug in `buglog.json`, and write
`Knowledge/Sessions/<date>-mfc-decoupled-core-port.md`.

---

## Following stages (sketch — not for execution yet)

**Stage 2 — HEOL as its own library, with `mfc_core` as a dependency.** HEOL is
not "MFC with a feedforward bolted on", and must not be built by widening
`mfc_core`. Recall the structure from the Context section: the estimator is fed
`ε` and **`u_fb`** — the feedback component only, not total `u`. That is what
makes a bolt-on wrong: `mfc_core`'s `d2u` term uses `command[1]`, so the
estimator sees the correct signal exactly when `mfc_core` *is* the `u_fb` block
and nothing else.

The consequence is that **stage 1's core needs no further change to support
HEOL.** A new `heol` module owns `u_ff` and the residual, and drives a decoupled
`MfcParameters` instance with `setpoint = 0`, `measure = ε` — giving
`error[0] = ε`, `z[0] = ε` (decoupled drives on `measure`), an internal
feedforward of zero (constant setpoint ⇒ zero double-difference), and
`command[0] = u_fb`. Every wiring invariant falls out of stage-1 semantics
rather than needing new flags. That is the reason to do stage 1 properly first.

Shape: `paparazzi/sw/airborne/firmwares/rotorcraft/control/heol.c/.h` +
`paparazzi/conf/modules/heol.xml` with `<depends>mfc_core</depends>`, mirroring
how `mfc_core.xml` is already a dependency-only library module with no settings
of its own. Exposes a per-axis `struct HeolParameters` embedding an
`MfcParameters`, and stays guidance/stabilization-agnostic so one library serves
both.

**Stage 3 — wire HEOL into ANTON's guidance.** `u_ff` comes from the flat
reference, which the firmware already carries but discards: accel/jerk/snap
propagate into `gh->ref.accel/jerk/snap` and `gv->zdd_ref/j_sp/s_sp`
(`guidance_h_ref.c:117-152`, `guidance_v_ref.c:108-131`), while
`guidance_mfc.c` reads only `gh->ref.pos` (`:566-567`) and `gv->z_ref` (`:505`).
That reader belongs in the HEOL wiring, not in `mfc_core`. Open question to
settle then: whether `u_ff` is the full flatness-inverted thrust/torque or just
`α⁻¹ẍ_ref` — the latter is nearly free once the reference is read and may be
enough for the quadrotor.

**Stage 4 — trajectory plumbing.**
`paparazzi/sw/airborne/modules/nav/flat_traj_demo_data.h:57` has
`FLAT_TRAJ_DEMO_ORDER 0`, so the demo runs position-only despite the table being
fully populated with derivatives; bump to 4 once stage 3 consumes them. Mind
that sample 0 carries nonzero snap (±165.93) and heading accel (2.094) — a t=0
discontinuity against a hovering aircraft. The header's own comment block still
calls the array an all-zero placeholder, which is stale. Then split the raw data
into its own header so the MATLAB generator can overwrite it directly, making a
trajectory swap "regenerate the header, change one `#include`".

## See also

- [[Flatness Trajectory Setpoints (Pos-Vel-Accel-Jerk-Snap + Psi)]] — the
  setpoint path this builds on
- [[02 - INDI Stabilization Deep Dive]], [[03 - INDI Guidance Deep Dive]] — the
  stack MFC sits alongside
- [[05 - Module System]], [[07 - All Touch Points Cheatsheet]] — module XML and
  per-change file checklists
