# Simulink MFC Quad ↔ ANTON_MFC Firmware — Parameter Correspondence

Written 2026-08-17. Supersedes the `"Values match ANTON_MFC airframe firmware"`
claim in `scripts/controllers/mfc_quad_params.m`, which is stale in the sense
that it was written against the **pre-2026-08-07 firmware gain convention**.

## Provenance — read this before using the table

The firmware column is derived from files read this session:

- `sw/airborne/firmwares/rotorcraft/stabilization/mfc_core.c` (the law)
- `sw/airborne/firmwares/rotorcraft/stabilization/stabilization_mfc.c`
- `sw/airborne/firmwares/rotorcraft/guidance/guidance_mfc.c`
- `conf/airframes/ENAC/quadrotor/anton_mfc.xml`
- `conf/modules/stabilization_mfc.xml`
- `git show be655818a` (the 2026-08-07 convention change)

**The Simulink column is not.** `scripts/controllers/mfc_quad_params.m`, the
`Generic_Quad` repo, and any `.slx` do not exist anywhere on this workspace's
filesystem. The Simulink values below are the ones quoted in the task brief.
Everything derived from them is marked as an inference, and the one line of the
`.m` that would confirm it is named in §3.

## 1. The firmware parameterization, stated exactly

`mfc_core.c` implements, per axis:

```
error       = measure - setpoint_trajec          # sign contract, non-negotiable
z           = decoupled ? measure : error        # estimator drive signal
a, b        = decoupled ? (0, 0) : (-kd, -kp)    # folded into F_hat when coupled
fb          = decoupled ? kd*edot + kp*e + ki*∫e : ki*∫e
u           = (-F_hat + rddot - fb) / alpha
```

Either structure closes the loop on the **same** polynomial:

```
s^2 + kd*s + kp
```

So `kp` and `kd` are **raw polynomial coefficients**, not `wn`/`zeta`:

```
wn   = sqrt(kp)
zeta = kd / (2*sqrt(kp))
```

`decoupled` does not change the target polynomial — it changes whether the poles
live inside `F_hat` (coupled) or in an explicit iPD(I) term (decoupled).

### What each axis actually controls

| Axis | `measure` | `u` (virtual command) | Structure |
|---|---|---|---|
| roll / pitch | Euler angle [rad], `stateGetNedToBodyEulers_f()` | torque [N·m] | coupled |
| yaw | unwrapped psi [rad] | torque [N·m] | coupled |
| gx / gy | NED position [m], Butterworth-filtered | **force [N]** (see §5) | decoupled |
| gz | NED z [m], down positive | thrust [N], hover ≈ −m·g | coupled |

## 2. Where the numbers actually live

The task brief attributed several values to `anton_mfc.xml` that are in fact
**module defaults** in `conf/modules/stabilization_mfc.xml`. The airframe
overrides every attitude gain, so the module defaults never reach ANTON_MFC.

| Define | Module default | anton_mfc.xml (what flies) |
|---|---|---|
| `ROLL_PROPORTIONAL_GAIN` | 36. | **4.** |
| `ROLL_DERIVATIVE_GAIN` | 18. | **6.** |
| `PITCH_PROPORTIONAL_GAIN` | 64. | **4.** |
| `PITCH_DERIVATIVE_GAIN` | 80. | **6.** |
| `YAW_PROPORTIONAL_GAIN` | 1. | **4.** |
| `YAW_DERIVATIVE_GAIN` | 2. | **6.** |
| `ROLL_ALPHA` | 2. | `2. / MODEL_INERTIA_XX` = 294.1 |
| `PITCH_ALPHA` | 2. | `2. / MODEL_INERTIA_YY` = 294.1 |
| `YAW_ALPHA` | 0.2 | `3. / MODEL_INERTIA_ZZ` = 220.6 |

**Consequence:** roll and pitch are *not* asymmetric on ANTON_MFC — both are
`kp 4 / kd 6`. The 36/18-vs-64/80 asymmetry is real but lives in the module
defaults, i.e. it applies to any airframe that does not override them
(Hoops_111_MFC came from that tuning). It is not an ANTON_MFC baseline problem.

## 3. The Simulink `kp`/`kd` convention — inferred, 3-for-3

Hypothesis: **Simulink `kp` is `wn` [rad/s] and Simulink `kd` is `2*zeta`
[dimensionless]**, so the closed-loop polynomial is
`s^2 + (kd*kp)*s + kp^2`, giving

```
kp_firmware = kp_simulink^2
kd_firmware = kd_simulink * kp_simulink
```

This is the same conversion the 2026-08-07 commit applied to the airframes, once
you note that the *old* firmware `kd` was `zeta` while Simulink's is `2*zeta`
(`kd_new = 2*kd_old*kp_old` ≡ `kd_sim * kp_sim`).

Three independent checks, all exact:

| Axis | Simulink | Predicted firmware | Actual `anton_mfc.xml` |
|---|---|---|---|
| roll | `kp_phi=2`, `kd_phi=3` | 4, 6 | **4, 6** ✅ |
| pitch | `kp_theta=2`, `kd_theta=3` | 4, 6 | **4, 6** ✅ |
| thrust | `kp_T=4`, `kd_T` not quoted | 16, `4*kd_T` | **16, 12** ⇒ implies `kd_T=3` ✅ |

Three axes agreeing exactly, with the thrust axis *predicting* an unquoted
Simulink value (`kd_T = 3`, the same `2*zeta = 3` as roll and pitch), is strong.
It is not proof.

**To confirm, read one thing in `mfc_quad_params.m`:** the line where `kp_*` /
`kd_*` are consumed — or any comment stating the closed-loop form. If it reads
`s^2 + kd*kp*s + kp^2` (equivalently `wn = kp`, `zeta = kd/2`), the hypothesis
holds and **roll, pitch and the vertical axis are already identical between the
two models**. If instead `kp`/`kd` are raw polynomial coefficients like the
firmware's, then Simulink roll/pitch is `wn = 1.414, zeta = 1.061` against the
firmware's `wn = 2, zeta = 1.5`, and the two genuinely differ.

## 4. Per-axis correspondence table

`I_xx = I_yy = 0.0068`, `I_zz = 0.0136`, `m = 0.8` — all **placeholders** (see §6).

### Attitude

| Simulink | Firmware define | Same quantity? | Conversion / note |
|---|---|---|---|
| `kp_phi = 2` | `ROLL_PROPORTIONAL_GAIN = 4.` | No — different parameterization | `kp_fw = kp_sim^2`. Under §3, **identical loops**. |
| `kd_phi = 3` | `ROLL_DERIVATIVE_GAIN = 6.` | No | `kd_fw = kd_sim * kp_sim`. Under §3, **identical**. |
| `kp_theta = 2` | `PITCH_PROPORTIONAL_GAIN = 4.` | No | as roll. **Identical** under §3. |
| `kd_theta = 3` | `PITCH_DERIVATIVE_GAIN = 6.` | No | as roll. **Identical** under §3. |
| `alpha_phi = 1/I(1)` ≈ 147.1 | `ROLL_ALPHA = 2./I_xx` = 294.1 | Same quantity, **factor 2 apart** | See §5. Does not move the poles. |
| `alpha_psi` (comment `YAW_ALPHA = 3`) | `YAW_ALPHA = 3./I_zz` = 220.6 | Same quantity, **factor 3 apart** if Simulink is `1/I_zz` = 73.5 | See §5. |
| — | `ROLL/PITCH/YAW_DECOUPLED = FALSE` | — | Attitude is coupled; guidance gx/gy is decoupled. Mixed, see §7. |
| `FFilter_*` (comment `= 5`) | `ROLL/PITCH/YAW_INTEGRATION_WINDOW = 5.` | Same knob, **sample-rate dependent** | See §8. |
| — | `ROLL/PITCH/YAW_TIME_TRAJECTORY = 50.` | — | Sample-rate dependent, see §8. |
| — | `ROLL/PITCH/YAW_COMMAND_FILTER = 10.` | — | Was `1.` before 2026-08-07. |

Resulting firmware attitude loop: **`wn = 2.0 rad/s`, `zeta = 1.5`** on all three
axes. That is a slow attitude loop for a 0.8 kg quad — but it is a faithful
conversion of the flight-tested `kp=2, kd=1.5, USE_KD=TRUE` tuning, not a
conversion error.

### Vertical (gz)

| Simulink | Firmware define | Same quantity? | Conversion / note |
|---|---|---|---|
| `kp_T = 4` (comment `GZ_KP = 4`) | `GZ_KP = 16.` | No — parameterization | `16 = 4^2`. Comment names the **pre-2026-08-07** value. **Identical** under §3. |
| `kd_T` (not quoted; §3 implies 3) | `GZ_KD = 12.` | No | `12 = 3*4`. **Identical** under §3 if `kd_T = 3`. |
| — | `GZ_ALPHA = 5./m` = 6.25 | Physical would be `1/m` = 1.25 | Factor 5. See §5. |
| `command_filter_T = 1` (comment `= 8`) | `GZ_COMMAND_FILTER = 8.` | Same knob | **Real difference**: Simulink has no command filter on thrust (1 = none), firmware smooths with 8. Sample-rate dependent (§8). |
| — | `GZ_TIME_TRAJECTORY = 4.`, `GZ_INTEGRATION_WINDOW = 4.` | — | Much tighter than the attitude axes' 50/5. |
| — | `GZ_DECOUPLED = FALSE` | — | Coupled, like attitude. |

Resulting firmware vertical loop: **`wn = 4.0 rad/s`, `zeta = 1.5`**.

### Horizontal (gx / gy)

| Simulink | Firmware define | Same quantity? | Conversion / note |
|---|---|---|---|
| `kd_pos_x = 2` | `GX_KD = 25` | **No, and not a conversion** | The `2 → 25` gap is not explained by §3. GX/GY were **hand-retuned** on 2026-08-07 to `kp=2, kd=25` in the decoupled structure to kill an oscillation (commit message of `be655818a`). This is a deliberate divergence from the model. |
| — | `GX_KP = 2` | — | Note `kp=2` here is a *polynomial coefficient*, unlike Simulink's `kp = wn`. |
| `FFilter_pos_x = 100` (comment `= 5`) | `GX_INTEGRATION_WINDOW = 600.` | Same knob | Was `10.` before 2026-08-07. Comment's `= 5` points at the **attitude** window, not this one — the comment is mis-targeted. Sample-rate dependent (§8). |
| — | `GX_ALPHA = 15./m` = 18.75 | Physical would be `1/m` = 1.25 | Factor 15. See §5. |
| — | `GX/GY_DECOUPLED = TRUE` | — | The only axes using the explicit-iPD structure. |
| — | `GX/GY_COMMAND_FILTER = 1.` | — | No smoothing. |

Resulting firmware horizontal loop: **`wn = 1.41 rad/s`, `zeta = 8.84`** — heavily
overdamped, effectively a first-order lag. This is the axis with the largest and
least-explained gap to the model.

## 5. ALPHA — same quantity, different constant, and it does not move the poles

`alpha` is the assumed input gain in `ëe = F + alpha*u`. The firmware's alpha is a
tuning constant divided by the physical inertia/mass; the Simulink alpha (as
quoted) is the pure `1/I`:

| Axis | Simulink | Firmware | Ratio |
|---|---|---|---|
| roll | `1/I_xx` = 147.1 | `2./I_xx` = 294.1 | 2 |
| pitch | `1/I_yy` = 147.1 | `2./I_yy` = 294.1 | 2 |
| yaw | `1/I_zz` = 73.5 (inferred) | `3./I_zz` = 220.6 | 3 |
| gz | `1/m` = 1.25 (physical) | `5./m` = 6.25 | 5 |
| gx/gy | `1/m` = 1.25 (physical) | `15./m` = 18.75 | 15 |

**A constant alpha error is self-cancelling.** The estimator absorbs it into
`F_hat` (`F_hat → (alpha_true - alpha)*u`), so the closed-loop polynomial stays
`s^2 + kd*s + kp` regardless. Confirmed empirically on the HEOL side 2026-08-14.

What alpha *does* change:
1. **Estimator-error amplification.** `u = (-F_hat + …)/alpha` — a larger alpha
   divides estimator noise by more. The firmware's alpha being 2–15× larger than
   physical makes it *less* noise-sensitive than the model, not more.
2. **Nothing about the clamps** — those are computed separately (§6).

So the alpha column is a genuine numeric difference that is **not** a control
difference. Do not "fix" it by rescaling the PD; that was tried on HEOL and made
things worse.

The SI-units port (2026-07-16) is what created the split: it kept the old
pre-SI tuning constants (2, 2, 3, 5, 15) as numerators and divided by the
physical constant. Simulink evidently never carried those numerators.

## 6. Clamps and the placeholder-mass caveat

Computed in code, not configured:

| Axis | Clamp | ANTON_MFC value |
|---|---|---|
| roll | `±0.5 * MAX_PPRZ * Σ|g1g2[0][i]|` | ±5.22 N·m |
| pitch | same, row 1 | ±5.22 N·m |
| yaw | same, row 2 | ±1.31 N·m |
| gz | `[-GZ_MAX_THRUST, m*g]` | [−46.08, +7.85] N |
| gx/gy | `±0.7 * 9.81 * m * sin(MAX_BANK)` | ±1.879 N (= 0.7·20° of bank) |

`g1g2` is built in `sum_g1_g2()` as `I_axis * G1_row / 1000` (torque rows) and
`m * G1_row / 1000` (thrust row). G2 is deliberately excluded (MFC allocates
absolute commands).

**`MODEL_MASS = 0.8`, `INERTIA_XX/YY = 0.0068`, `INERTIA_ZZ = 0.0136` are
explicitly marked `TODO: measure` in the airframe.** They are the JSBSim
`anton` model's own constants. In SITL that is self-consistent — the numbers
describe the aircraft that is actually flying — which is the only reason this is
tunable at all right now. **It also means every SITL gain carries a scaling
assumption that a real ANTON will not honour.** Because alpha error is absorbed
(§5), a mass/inertia error will not destabilise the loop, but it *will* move
every clamp and the hover normalisation. Bench-measure before flight.

Also note the `0.7` in the gx/gy clamp: an undocumented hard-coded 70 % derating
of `GUIDANCE_H_MAX_BANK`. Nothing states why.

## 7. The mixed DECOUPLED configuration

| Axis | `DECOUPLED` |
|---|---|
| roll, pitch, yaw | FALSE (coupled) |
| gx, gy | **TRUE** |
| gz | FALSE |

The gx/gy `TRUE` is documented in `be655818a`: the decoupled structure plus a
hand-tuned `kp=2/kd=25` was what killed the horizontal oscillation. The other
five axes were left coupled because coupled is the flight-tested fallback
(Decision Log, 2026-08-06). So the mix is justified — but only the gx/gy half of
it was written down, which is why it reads as arbitrary.

## 8. Sample-rate dependence — the trap in `int_window`, `time_trajec`, `command_filter`

All three are **dimensionless discrete-window parameters**, not time constants:

- `command_filter`: `u ← (u + (W-1)*u_prev) / W` — a one-pole IIR over *samples*.
- `int_window`, `time_trajec`: the `W` in `mfc_iir_step`'s
  `(W^2 + 2W + 1)` smoother — again over *samples*.

ANTON_MFC runs at `PERIODIC_FREQUENCY = 500`. **If the Simulink model runs at a
different rate, none of these three transfer numerically** — they must be scaled
by the rate ratio to preserve the same physical smoothing.

### The conversion rule

`mfc_iir_step` is `(x + (2W^2+2W)*x1 - W^2*x2) / (W^2+2W+1)`. Its characteristic
polynomial is `z^2 - (2W^2+2W)/(W+1)^2 * z + W^2/(W+1)^2`, a **double pole at
`z = W/(W+1)`**. So

```
tau = -1 / (f_s * ln(W/(W+1)))  ~=  W / f_s     (W >> 1)
```

W is therefore a count of **samples**, and the physical time constant is
`W / f_s`. To carry a window between models: `W_b = W_a * f_b / f_a`.

At `f_s = 500`:

| Define | W | tau |
|---|---|---|
| attitude `TIME_TRAJECTORY` | 50 | 0.10 s (double pole) |
| attitude `INTEGRATION_WINDOW` | 5 | 0.010 s |
| `GZ_TIME_TRAJECTORY` / `GZ_INTEGRATION_WINDOW` | 4 | 0.008 s |
| `GX/GY_INTEGRATION_WINDOW` | 600 | 1.20 s |

`command_filter` is the simpler one-pole `u <- (u + (W-1)*u_prev)/W`, pole at
`(W-1)/W`, `tau ~= W / f_s`: attitude `10 -> 0.020 s`, `GZ 8 -> 0.016 s`.

Note the attitude reference filter's 0.10 s double pole is **not** negligible
against a `wn = 2 rad/s` (0.5 s) closed loop — it materially shapes the step
response, so a measured attitude rise time is the reference filter and the loop
together, not the loop alone.

### Applying it

This gives a testable explanation for `FFilter_pos_x = 100` vs
`GX_INTEGRATION_WINDOW = 600`: if the Simulink model runs at 100 Hz, its
`tau = 1.0 s` against the firmware's `1.20 s` — near-agreement, where the raw
numbers look 6x apart. **Do not treat this as established.** Read the model's
sample time from `mfc_quad_params.m`, convert with `W_b = W_a * f_b / f_a`, and
either confirm it or discard the idea.

Until the Simulink sample rate is known, `int_window`, `time_trajec` and
`command_filter` cannot be compared across the two models at all, and any
"difference" reported for them is meaningless.

## 9. Summary — is there a real transfer gap?

| Axis | Verdict |
|---|---|
| roll | **Parameterization artefact.** Identical loops under §3. |
| pitch | **Parameterization artefact.** Identical loops under §3. |
| yaw | Not comparable — no Simulink yaw `kp`/`kd` quoted. Firmware is `wn=2, zeta=1.5`. |
| gz | **Parameterization artefact** for `kp`/`kd`. **Real difference** in `command_filter` (1 vs 8). |
| gx / gy | **Real difference.** Deliberate: hand-retuned to a different structure (decoupled) and a much more damped tuning than the model. |
| alpha, all axes | Real numeric difference (2×–15×), **not** a control difference (§5). |
| window/filter params | **Not comparable** until the Simulink sample rate is known (§8). |

The headline: the two models are much closer than the raw numbers suggest. Every
attitude and vertical `kp`/`kd` disagreement in the brief's list is explained by
one convention difference (`kd_sim = 2*zeta` vs `kd_fw = 2*zeta*wn`) plus the
2026-08-07 conversion. The genuinely divergent axis is **horizontal**, and that
divergence is documented and intentional.

---

# Addendum, 2026-08-17 (later) — verified against the measured Simulink configuration

The kickoff `Knowledge/Tasks/kickoff-mfc-quad-sitl-position.md` supplied the
Simulink side **measured** on 2026-08-17 from `models/controllers/mfc_quad.slx`
at `Ts = 0.002 s`. ANTON_MFC runs `PERIODIC_FREQUENCY = 500`, so **both sides are
at 500 Hz and every sample-count window compares directly** — the unit trap in
the kickoff's §1 does not bite here.

This replaces the kickoff's "unverified" §1 table. Firmware column read from
`anton_mfc.xml`, `mfc_core.c` and `guidance_mfc.c`.

## Verified correspondence

| | Simulink | Firmware (before) | Firmware (now) |
|---|---|---|---|
| **structure** φ/θ/ψ | coupled | coupled ✅ | coupled |
| **structure** z | **decoupled** | **coupled** ❌ | coupled — deliberately, see §A2 |
| **structure** x/y | decoupled | decoupled ✅ | decoupled |
| `est_window` φ/θ/ψ | 5 | 5 ✅ | 5 |
| `est_window` z | 50 | **4** | **4** — see §A2 |
| `est_window` x/y | 100 | **600** | 600 |
| `ref_window` all | 50 | attitude **250**, z 4, x/y 50 | attitude **50** ✅, z 4, x/y 50 |
| `command_filter` | 1 (off) everywhere | attitude **10**, z **8**, x/y 1 | attitude **1** ✅, z 8, x/y 1 |
| `Ki` | 0 | 0 ✅ | 0 |
| `est_hold_time` | 0 att / 0.01 z,pos | 0.1 everywhere | 0.1 — longer is safer, left alone |
| **estimator command tap** | **post-EMA, post-clamp** | **pre-saturation** ❌ | **post-clamp** ✅ |
| `Kp`/`Kd` φ, θ | 4 / 12 | 4 / **6** | **4 / 12** ✅ |
| `Kp`/`Kd` ψ | 4 / 6 | 4 / 6 ✅ | 4 / 6 |
| `Kp`/`Kd` z | 16 / 5.6 | 16 / **12** | 16 / 12 — see §A2 |
| `Kp`/`Kd` x, y | 75 / 150 | **2 / 25** | 2 / 25 — see §A3 |
| `alpha` x/y | 18.75 (`15/m`) | 18.75 ✅ | 18.75 |
| `alpha` z | 1.25 (`1/m`) | 6.25 (`5/m`) | 6.25 |
| `alpha` φ/θ, ψ | 147.06, 73.53 (`1/I`) | 294.1, 220.6 (`2/I`, `3/I`) | unchanged |

Because x/y share both the **structure** (decoupled) and the **alpha** (18.75),
their `Kp`/`Kd` are directly comparable with no reconciliation — that is the one
axis pair where a bare gain comparison is legitimate.

## A0 — §3's inference was half wrong; here is the correction

§3 above inferred, from the *brief's quoted* Simulink values, that
`kd_firmware = kd_simulink * kp_simulink`. The kickoff's **measured** applied
coefficients contradict the `kd` half:

| axis | Simulink applied Kp | Simulink applied Kd |
|---|---|---|
| φ, θ | 4 | **12** |
| ψ | 4 | 6 |
| thrust | 16 | 5.6 |

§3 predicted φ/θ `kd = 3 * 2 = 6`. The measured value is **12**. So the correct
conversion carries a factor of 2:

```
kp_firmware = kp_simulink^2
kd_firmware = 2 * kd_simulink * kp_simulink        # NOT kd_sim * kp_sim
```

i.e. **Simulink `kd` is `zeta`, not `2*zeta`** — the same convention the *old*
pre-2026-08-07 firmware used. Consistency check on the other axes: ψ applied 6
⇒ `kd_sim = 6/(2*2) = 1.5`; thrust applied 5.6 ⇒ `kd_sim = 5.6/(2*4) = 0.7`.
Both plausible damping ratios, so the corrected rule is self-consistent across
all three.

**`kp` is unaffected** — `kp_fw = kp_sim^2` still holds on all three axes, which
is why §3's three-for-three `kp` agreement was real.

The practical consequence is the roll/pitch `Kd`: firmware was running **6**
where the reference runs **12**, a genuine 2× under-damping, not a
parameterization artefact. That is fixed below.

## A1 — H1: answered, and it was true

`mfc_core.c` set `est_use_presat_command = true`, i.e. the estimator was fed the
**pre-saturation** model-inversion output, where Simulink's `mfc_siso.step` ends
`state.u_km1 = u` with `u` post-EMA and post-clamp.

```c
mfc_stt->command_est = mfc_stt->est_use_presat_command ? mfc_stt->command_presat
                                                       : mfc_stt->command[0];
```

Nothing in `stabilization_mfc.c` or `guidance_mfc.c` overrode it, so **all six
MFC axes ran the pre-saturation tap**. (Only `guidance_heol.c` exposed a knob —
which is how the same defect came to be diagnosed on HEOL first.)

**Changed the default to `false`.** This is the correct anti-windup tap and it
matches the reference. It is the one H that was both true and load-bearing.

## A2 — H2: FALSE for firmware, and the reasoning inverts

H2 predicted the z estimator window (4 samples, 8 ms vs Simulink's 50) was
starving the estimator and causing "z noisy and choppy". **Tested and refuted.**

Raising `GZ_INTEGRATION_WINDOW` 4 → 50 **broke the vertical loop**: altitude
swung 10–30 m against a 3 m setpoint with `cmd_z` bang-banging both clamps.

The reason is structural, and it is the key correction to the kickoff:

> **In the coupled structure, `int_window` is not a noise filter — it is the
> estimator bandwidth, and in coupled mode the estimator *is* the feedback path**
> (the closed-loop poles are folded into `F_hat`). Slowing it 12× detunes the
> controller itself.
>
> Simulink can run 50 on z because **its z axis is decoupled** — there the poles
> live in an explicit iPD and the estimator only cancels disturbance, so the
> window genuinely is a noise filter. H2 is a decoupled-channel argument applied
> to a coupled channel.

For x/y — decoupled on both sides — H2's reasoning *does* apply. But it points
the other way there: firmware's 600 is 6× **longer** than Simulink's 100, so the
horizontal estimator is heavily smoothed, not starved.

Measured with the native fast config (`est_window = 4`, coupled, no measurement
filter), vertical alone, noise on: **err_z rms 1.7 mm**, `cmd_z` smooth in
[−10.1, −7.6] N with no clamping. The z channel was never the problem.

**F̂ rms on the GZ channel (deliverable): 57.5** (rung-3 run, t = 25–140 s, noise
on). For reference the kickoff quotes Simulink at 11.3 m/s² on z — not the same
quantity, since firmware's gz command is in Newtons and carries `alpha = 6.25`.

### Filtering the z measurement also fails, for the same reason

Extending the x/y 3 Hz Butterworth to z (the obvious response to "z is noisy")
**also produced a limit cycle between both thrust clamps**. A 3 Hz Butterworth is
~60 ms of lag, against a channel whose reference filter and estimator window are
both 8 ms. z is deliberately left on the raw measurement, and
`guidance_mfc.c` now says so and says why.

**General rule this produced:** before filtering an MFC measurement, compare the
filter's lag with that axis's `int_window` and `ref_window` in milliseconds. If
the lag is comparable or larger, the filter will destabilise the loop. Slow the
whole channel coherently instead.

## A3 — the horizontal gains were left alone, on purpose

Firmware x/y run `Kp 2 / Kd 25` against Simulink's `75 / 150` — a genuine 37×/6×
difference, same structure, same alpha. **The firmware values were kept.**

With the H1 fix and the attitude damping fix in place, firmware holds hover
position at **x/y rms 4.8 / 4.2 cm with its command at 5 % of the ±1.879 N rail**.
The Simulink loop holds 2.5 cm but with its command **on the rail 73–92 % of the
time**, and the kickoff itself warns its noise model is optimistic and its rail
fraction is not a target. Adopting `75/150` would buy ~2 cm of hover error at the
cost of running the bank command into saturation — the wrong trade for a vehicle
that has to fly.

The `est_window` 600 vs 100 difference was likewise left: the horizontal loop is
quiet and unsaturated as it stands.
