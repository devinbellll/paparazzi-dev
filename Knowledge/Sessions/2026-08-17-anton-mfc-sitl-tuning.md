# Session — 2026-08-17: ANTON_MFC SITL tuning, parity map, and the reference-filter bandwidth fault

Goal: tune the MFC quad controller in SITL on ANTON_MFC, and first establish
whether ANTON_MFC and the `Generic_Quad` Simulink MFC model are the same
controller.

**Rung reached: 2 (attitude), with rung 1 verified.** Rungs 3–5 not reached.

> [!warning] Part 1's fix and its `kd` inference are SUPERSEDED by Part 2 (below)
> Part 1 cured the 65 % attitude overshoot by slowing the reference filter
> (`TIME_TRAJECTORY` 50 → 250). Part 2, working from the *measured* Simulink
> configuration, found the real cause: **roll/pitch `Kd` was under-damped by
> exactly 2×** (6 where the reference runs 12). With `Kd = 12` the overshoot
> drops to 34 % *and keeps the 0.175 s rise time*, so `TIME_TRAJECTORY` is back
> at its flight-validated **50**.
>
> Part 1's §3 inference `kd_fw = kd_sim * kp_sim` is also wrong by a factor of 2;
> the correct rule is `kd_fw = 2 * kd_sim * kp_sim` (Simulink `kd` is `zeta`, not
> `2*zeta`). The `kp` half — `kp_fw = kp_sim^2` — stands. See §A0 of the
> correspondence note.
>
> Everything else in Part 1 (rung 1, the noise switch, the include-order bug, the
> wn 2–6 stability ceiling, the methodology notes) stands unchanged.

## Headline

The attitude axes carried a **reference-filter / closed-loop bandwidth
mismatch**: the reference-trajectory filter is ~5× faster than the loop it
drives, so the `rddot` feedforward overdrove every attitude step. Measured 65 %
overshoot with ~9× more control effort than needed. **Reproduced identically
with every noise source zeroed** — structural, not noise.

## Blocker: the Simulink model is not in this workspace

`scripts/controllers/mfc_quad_params.m`, the `Generic_Quad` repo, and any
`.slx` do not exist anywhere on this filesystem (searched `/`). Only two session
notes mention `Generic_Quad`, both saying work was "tracked on the Generic_Quad
side".

So the correspondence table's firmware column is derived from source read this
session; its Simulink column is the values quoted in the task brief. The one
question that needs the `.m` is named explicitly in the note (§3) — it is a
single line about what `kp`/`kd` mean.

## Deliverable 1 — correspondence table

`Knowledge/15 - Simulink MFC Quad ↔ ANTON_MFC Firmware Correspondence.md`.

Key result: **the two models are far closer than the raw numbers suggest.** The
inferred Simulink convention is `kp = wn`, `kd = 2*zeta`, so

```
kp_firmware = kp_simulink^2
kd_firmware = kd_simulink * kp_simulink
```

Three independent checks agree exactly (roll `2,3 -> 4,6`; pitch `2,3 -> 4,6`;
thrust `4 -> 16` with the unquoted `kd_T = 3` *predicted* as `12`). Under this
reading roll, pitch and the vertical axis were already **identical** between the
two models before this session.

## Premise corrections found in the brief

1. **The disputed gains are not all in `anton_mfc.xml`.** ROLL 36/18, PITCH
   64/80, `ROLL_ALPHA=2`, `YAW_ALPHA=0.2` are **module defaults** in
   `conf/modules/stabilization_mfc.xml`. The airframe overrides every attitude
   gain with `kp 4 / kd 6`. So **roll and pitch are NOT asymmetric on
   ANTON_MFC** — red flag #2 dissolves. The asymmetry is real but lives in the
   module defaults, which ANTON_MFC never uses.

2. **The attitude gains were converted correctly** at the 2026-08-07 convention
   change (`be655818a`): `kp=2, kd=1.5, USE_KD=TRUE` → `4, 6`, exactly `kp^2`
   and `2*kd*kp`. Not a conversion error.

3. **`NPS_*_NOISE_STD_DEV_*` are not `#ifndef`-guarded.** `nps_sensors.h:6`
   includes `nps_sensors_params_default.h` unconditionally when
   `NPS_SENSORS_PARAMS` is unset. Airframe defines cannot override them; the
   airframe must select a params header instead.

4. **`--rc_script 1/2/3` is not an attitude instrument on this airframe.** Those
   scripts set `MODE_SWITCH_AUTO2`, and `anton_mfc.xml` maps AUTO2 to
   `AP_MODE_NAV` — full guidance, sticks ignored. The attitude step instrument
   for ANTON_MFC is **`--rc_script 5`** (`fp_takeoff_zhold`): NAV climb for 15 s,
   then AUTO1 = `AP_MODE_ATTITUDE_Z_HOLD` with ±0.3 stick steps cycling
   pitch → roll → yaw, 4 s each. Yaw is stick-**rate** commanded, so it ramps
   rather than steps — a step analyser finds no yaw edges, correctly.

## Rung 1 — unit contract and hover thrust

Verified by reading `mfc_core.c`, `stabilization_mfc.c`, `guidance_mfc.c`, then
confirmed in flight.

- Attitude: measure = Euler angle [rad], command = torque [N·m], coupled.
- gz: measure = NED z [m], command = thrust [N].
- Clamps computed at init: roll/pitch ±5.22 N·m, yaw ±1.31 N·m,
  gz [−46.08, +7.85] N, gx/gy ±1.879 N.

**Hover `gz` command = −9.02 N mean against −m·g = −7.848 N — a 14.9 %
deviation** (baseline hover run, 147 s, `mfc_sim_20260817_094606.csv`). Per the
airframe's own comment this measures G1-thrust-row / mass identification error:
the JSBSim motors are ~13 % *less* effective than the identified `−1.5` thrust
row claims (implied true row ≈ −1.305). Absorbed by the F-estimator; the
vertical loop tracked to 0.012 m RMS.

Note `MFC_GUIDANCE/fk_z` sits at ~+55 mean, and `−F_hat/alpha = −8.88 N` is
essentially the entire hover command. That is the MFC structure working as
designed (the estimator absorbs gravity), not a runaway — `err_z` RMS is 0.012 m.

### Unit-contract errors found

- **`guidance_mfc.c` gx/gy commands are forces [N], not accelerations.** They are
  divided by `mfc_thrust_physical` [N] in `accel_to_att_sp()` and clamped by
  `9.81 * MASS * sin(MAX_BANK) * 0.7` [N]. The `anton_mfc.xml` comment calling
  them "NED acceleration [m/s^2] … clamped to ±g·sin(MAX_BANK)" is wrong on both
  counts. Internally consistent either way (both clamp and denominator carry the
  same `MASS`), so no behavioural bug — a documentation error only.
- The `0.7` in that clamp is an undocumented hard-coded 70 % derating of
  `GUIDANCE_H_MAX_BANK`.
- `mfc_thrust_physical` is still hard-wired to the nominal hover constant
  (`guidance_mfc.c:547`, pre-existing `// TODO`); `filt_thrust` is computed and
  discarded. Correct in level hover, drifts in climb/descent.

## Rung 2 — attitude, and the fault

Instrument: `--rc_script 5`, ±0.3 stick = ±13.5° steps (`SP_MAX_PHI = 45°`).

All runs below are **verified single-feed** (500 rows/s, zero duplicate
timestamps, zero backward time steps) — see the methodology note.

| Run | Config (`wn`/`ζ` of `s²+kd·s+kp`) | t63 | t90 | peak (1.0 = no overshoot) | \|cmd\|max [N·m] | \|fk\|max |
|---|---|---|---|---|---|---|
| R1 | baseline `kp 4 / kd 6` (wn 2, ζ 1.5), traj 50, **noise on** | 0.222 s | 0.264 s | **1.65** | 0.139 | 39.3 |
| R2 | baseline, **noise off** | 0.222 s | 0.264 s | **1.65** | 0.139 | 39.3 |
| R3b | baseline gains, **traj 250** via `--set`, noise off | 0.824–0.848 s | 2.12–2.17 s | **0.977–1.028** | 0.0149 | 3.9 |
| R4b | `kp 100 / kd 20` (wn 10, ζ 1.0), traj 50, noise off | — | — | **diverged** | **5.2224 = clamp** | 45028 |
| R5 | `kp 36 / kd 18` (wn 6, ζ 1.5) via `--set`, noise off | — | — | **diverged** | **5.2224 = clamp** | 46294 |
| R6 | `kp 36 / kd 18` **baked in XML, from boot**, noise off | — | — | **diverged** | **5.2224 = clamp** | 45540 |
| **R8** | **final config** (traj 250 in XML, from boot), noise off | 0.848 s | 2.13 s | **0.975** | 0.0150 | 3.99 |
| **R10** | **final config**, **noise on** | 0.826–0.872 s | 2.11–2.18 s | **0.975–1.005** | 0.0149 | 3.95 |

Roll and pitch agree to 3–4 significant figures, and every 24 s cycle repeats
identically.

CSVs: R1 `…_095444`, R2 `…_095952`, R3b `…_100656`, R4b `…_100851`,
R5 `…_100959`, R6 `…_101146`, R8 `…_101345` (partial, 21 s — enough to show the
first pitch step), R10 `…_102006`. All verified single-feed.

### The two noise pairs

| Pair | noise on | noise off | verdict |
|---|---|---|---|
| baseline (R1 / R2) | peak 1.65, cmd 0.139, fk 39.3 | peak 1.65, cmd 0.139, fk 39.3 | **identical** |
| final config (R10 / R8+R3b) | peak 0.975–1.005, cmd 0.0149, fk 3.95 | peak 0.975–1.028, cmd 0.0149–0.0150, fk 3.9–3.99 | **identical** |

Both configurations are insensitive to sensor noise at this amplitude. The
baseline pair is what proved the fault was structural; the final pair confirms
the fix is not a noise-dependent result either.

### Methodology failure worth recording

The first attempts at R3 and R4 were run back-to-back and **both streams landed
in one CSV**. `sim.sh`/`sim_anton.py` do not self-terminate; a `timeout -s INT`
leaves the capture relay alive on `127.0.0.1:9871`, so a second sim started
while the first relay is still listening gets ingested into the *first* run's
file. The tell is the row rate: the contaminated file ran at 1000 rows/s in the
overlap region against the scope's real ~500 rows/s, and showed a physically
impossible state (`|phi|` pinned at 3.14 rad while `agl` held steady at 2.1 m).

Both runs were discarded and re-run strictly serialized (`docker rm -f` +
settle before and after each sim, CSV name taken from that run's own stdout
rather than `ls -t`). **Do not use `ls -t` to attribute a sim log** — parse the
filename the run itself printed.

### Why the noise pair decided the session

**R1 and R2 are numerically identical.** The 65 % overshoot and the ringing
survive perfect sensors, so they are structural. No filter or noise-side work
would have touched this — and the first lever the brief suggested reaching for
(the `FILT_CUTOFF` family) would have been wasted effort. Running the no-noise
case before any gain work is what made the rest of the session short.

### Root cause

The reference-trajectory filter and the closed loop are ~5× apart in bandwidth:

- `mfc_iir_step` has a **double pole at `z = W/(W+1)`**, so `tau ≈ W / f_s`.
  At `W = 50`, `f_s = 500` → `tau = 0.10 s` ≈ 10 rad/s.
- The designed closed loop is `s^2 + 6s + 4` → `wn = 2 rad/s`, poles at
  −0.764 and −5.236.

`mfc_core.c` feeds forward `dot_dot_setpoint_trajec / alpha`. A 0.235 rad step
through a 0.10 s reference produces `rddot ≈ 22.6 rad/s²` — computed by hand
from the filter recurrence and **matching the logged first command sample to
3 decimals** (0.0077 predicted vs 0.0080 logged, after the `W=10` command
filter). That feedforward, not the feedback, drives the whole motion; the
wn = 2 loop cannot arrest it, so the axis overshoots 65 % and rings at ~7 rad/s
after the reference has already settled.

R3b confirms it: slowing the reference to `tau ≈ 0.5 s` with **gains unchanged**
removes the overshoot entirely (peak 0.978), and cuts control effort 9× and the
estimator excursion 10×.

So this was never a gain-search problem. It is a configuration mismatch between
two knobs that have to be chosen together.

### The other end of the fix does not exist — there is a bandwidth ceiling

The obvious alternative is to speed the *loop* up to the reference instead
(wn 2 → 10 rad/s), which for a 0.8 kg quad is where an attitude loop normally
lives. It does not work:

| Gains | wn, ζ | Result |
|---|---|---|
| `kp 4 / kd 6` | 2, 1.5 | stable |
| `kp 36 / kd 18` | 6, 1.5 | **diverges** — command pinned at the ±5.22 N·m clamp, `fk` to 4.6e4 |
| `kp 100 / kd 20` | 10, 1.0 | **diverges** — same signature |

`kp 36 / kd 18` was tested **twice**: applied in flight via `--set` (R5), and
baked into the airframe XML and flown from boot after a full rebuild (R6). Both
diverge within ~4 s of `ATTITUDE_Z_HOLD` entry, so this is a real stability
limit and not a bumpless-transfer artefact of changing a folded gain mid-flight.
Worth having checked — in the coupled structure `kp`/`kd` live inside the
estimator, so a mid-flight change is not obviously benign.

**Interpretation:** raising `kp`/`kd` in the coupled structure raises the
*estimator's* own gain on `e` and `edot`. The folded-pole construction assumes
the estimator is much faster than the loop; with `int_window` `tau = 0.01 s`,
the command filter `tau = 0.02 s` and `ACT_FREQ = 30.5 rad/s`, that separation
is gone by wn = 6 rad/s. The ceiling lies between wn 2 and wn 6 and was **not**
bracketed more finely this session.

Note the module defaults (roll wn 6, pitch wn 8) sit *above* this ceiling. They
presumably work on Hoops_111_MFC, which has different inertia, effectiveness and
filter settings — but nothing should adopt them for ANTON_MFC.

## Configuration changes made

All promoted, none diagnostic-only. Working tree is clean of temporary edits.

### New — `conf/simulator/nps/nps_sensors_params_anton_mfc.h`

States every stock noise default explicitly and multiplies it by a scale, so
`NPS_NOISE_SCALE = 1` is identical to the previous implicit configuration.
Selected from the airframe SIMULATOR section:

```xml
<define name="SENSORS_PARAMS" value="nps_sensors_params_anton_mfc.h" type="string"/>
<define name="NOISE_SCALE"    value="1."/>
```

Per-source overrides `NPS_NOISE_SCALE_ACCEL/_GYRO/_MAG/_BARO/_GPS/_SONAR` each
default to `NPS_NOISE_SCALE`. "All noise off" is one line.

**The stock gyro white noise is already zero** — the gyro's only stochastic
content is the 0.5 °/s bias random walk in `nps_sensors_params_common.h`, which
is what `NOISE_SCALE_GYRO` acts on.

### Bug fix — `nps_sensor_accel.c`, `nps_sensors.c`

Both included `nps_sensors.h` **without** `generated/airframe.h` first. Since
that header picks the params file with `#ifndef NPS_SENSORS_PARAMS`, and
`NPS_SENSORS_PARAMS` is an airframe define, those two TUs silently fell back to
`nps_sensors_params_default.h` while `nps_sensor_gyro/mag/gps/baro/sonar.c`
(which do include it) used the airframe's file — **different noise parameters in
different translation units of the same binary.**

Confirmed by preprocessing before and after:

```
before:  nps_sensor_accel -> nps_sensors_params_default.h
         nps_sensor_gyro  -> nps_sensors_params_anton_mfc.h
after:   all three        -> nps_sensors_params_anton_mfc.h
```

Verified in flight: accelerometer noise drops 122× (first-difference std
31.77 → 0.26) between `NOISE_SCALE 1` and `0`. Without the fix the accelerometer
would have kept full noise in every "noise off" run.

This is a **latent upstream bug** — it bites any airframe that sets
`NPS_SENSORS_PARAMS`, not just ANTON_MFC.

### Fix — `conf/modules/guidance_mfc.xml`

`gx/gy/gz_cfilt` sliders were `min="0." max="1."`, but `command_filter` is a
divisor where 1 = no smoothing and the flown `GZ_COMMAND_FILTER` is **8** —
outside the slider's own range. Dragging that slider would have silently
disabled the filter, and `min=0` is a division hazard. Changed to
`min="1." step="1." max="50."`, matching the stabilization panel.

### Final recorded configuration — `anton_mfc.xml`

The only control-law change kept is the reference filter:

```xml
<define name="ROLL_TIME_TRAJECTORY"  value="250."/>   <!-- was 50. -->
<define name="PITCH_TIME_TRAJECTORY" value="250."/>   <!-- was 50. -->
<define name="YAW_TIME_TRAJECTORY"   value="250."/>   <!-- was 50. -->
```

`tau = W / f_s = 250/500 = 0.5 s`, matched to the `wn = 2 rad/s` loop.

**Gains are unchanged at `kp 4 / kd 6` on all three attitude axes.** The
`kp 36 / kd 18` experiment was reverted in the XML before R8/R10 were flown, and
those two runs — which are the ones recorded above — are from the restored
configuration. `git diff` on the airframe shows only the three
`TIME_TRAJECTORY` lines and the noise block; no gain lines appear.

Yaw was changed alongside roll and pitch for consistency of the reference
filter. **Yaw was not independently step-tested** — the stick commands a rate
(`SP_MAX_R`), so `rc_script 5` ramps it rather than stepping it, and the step
analyser correctly reports zero yaw edges. Yaw tracked its ramp without incident
in every non-diverged run, but that is not a step-response measurement and
should not be recorded as one.

Guidance gains (`GX/GY/GZ_*`) were **not touched** — that is rungs 3 and 4,
which were not reached.

## Errors of unit / sign / convention found (these recur)

1. Simulink `kd = 2*zeta` vs old firmware `kd = zeta` — the factor-2 that made
   the parity map look broken.
2. `int_window` / `time_trajec` / `command_filter` are **sample counts**, not
   time constants: `tau ≈ W / f_s`. Nothing that uses them transfers between
   models at different rates without scaling by `f_b / f_a`.
3. gx/gy virtual command is **N**, not m/s² — comment wrong, code consistent.
4. `NPS_SENSORS_PARAMS` include-order hazard (above).
5. Guidance `command_filter` slider range excluded the flown value.
6. `rc_script 1/2/3` → AUTO2 → NAV on this airframe, not attitude.

## Still open

- The `.m` line that confirms the Simulink `kp`/`kd` convention (§3 of the
  correspondence note) and the model's **sample rate** (needed before any
  window/filter parameter can be compared at all).
- Bench-measure `MODEL_MASS` / `INERTIA_*`. They are JSBSim's own constants, so
  SITL is self-consistent, but every clamp and the hover normalisation scale
  with them. Alpha error is absorbed by the estimator (2026-08-14), so this will
  not destabilise — it will move the envelopes.
- **Where the attitude bandwidth ceiling actually is.** Bracketed only as
  "between wn 2 and wn 6". Worth bisecting (wn 3, 4) — if wn 4 holds, the
  reference filter could be `W = 125` and the axis would be twice as quick for
  free. Cheap: both knobs are runtime `dl_settings` (`roll_kp`, `roll_kd`,
  `roll_traj`), so a bisection needs no rebuild.
- **Whether the ceiling is structural or filter-induced.** The candidates are
  the estimator `int_window`, the `command_filter`, and `ACT_FREQ`. Raising
  `int_window` (faster estimator) is the first thing to try — if the ceiling
  moves with it, the folded-pole separation assumption is the binding
  constraint, which is a result worth having for the MFC method generally, not
  just for this airframe.
- **Yaw step response.** Needs an attitude-yaw stick mode or a direct
  `mfc_yaw.setpoint` injection; `rc_script 5` cannot produce it.
- **Rungs 3–5** (vertical alone, horizontal re-enabled, nav blocks). Rung 4 is
  where the correspondence table predicts trouble: gx/gy are the one axis pair
  that genuinely diverges from the Simulink model (`GX_KD 25` vs
  `kd_pos_x 2`, decoupled vs coupled), and they carry the 1.2 s
  `INTEGRATION_WINDOW` that no one has justified.

## Rung reached

**2.** Rung 1 verified, rung 2 diagnosed and fixed with both noise pairs on
verified-clean captures, configuration restored-or-promoted. Rungs 3, 4 and 5
were not attempted — no guidance gain in this note is a tuned result, and none
was changed.

---

# Part 2 — the position loop (kickoff: `Knowledge/Tasks/kickoff-mfc-quad-sitl-position.md`)

**Rung reached: 5.** Position hold and trajectory tracking both work with noise on.

## Result

| rung | config | result (noise on) |
|---|---|---|
| 2 attitude | `rc_script 5`, ±13.5° steps | t63 0.175 s, overshoot **34 %** (was 65 %) |
| 3 vertical alone | `gx_on=0 gy_on=0`, NAV hold 2 m | **err_z rms 1.7 mm**, `cmd_z` smooth in [−10.1, −7.6] N, no clamping. **F̂_gz rms 57.5** |
| 4 hover position | NAV Standby, full stack | **x/y rms 4.8 / 4.2 cm**, peak ±9 cm, command at **5 %** of the ±1.879 N rail |
| 5 trajectory | `Flat_Traj_Demo` (1 m N, 1 m E, 1 m climb) | err x/y/z rms **6.1 / 5.7 / 0.8 cm**, attitude err rms 0.0002 rad |

Noise pair at rung 4: **4.8 / 4.2 cm with noise, 0.62 / 0.45 cm without.** The
residual is noise tracked through the estimator, not a limit cycle — the command
never approaches its rail and the no-noise case is essentially exact.

For scale, the Simulink reference holds 2.5 cm in hover **with its bank command
on the rail 73–92 % of the time**. Firmware holds 4.5 cm at 5 % rail. The
kickoff's own caution — that the Simulink noise model is optimistic and its rail
fraction is not a target — applies, and this is the better-conditioned loop.

## What actually fixed it — two changes

1. **H1, the estimator command tap** (`mfc_core.c`). Was `est_use_presat_command
   = true` on all six axes; the reference feeds the post-EMA, post-clamp command.
   Feeding the pre-saturation command makes F̂ drift to cover a command the plant
   never received, driving the command further into the rail. Default flipped to
   `false`. See addendum §A1 of the correspondence note.
2. **Roll/pitch `Kd` 6 → 12** (`anton_mfc.xml`), matching the measured Simulink
   applied value. This is a **real 2× under-damping**, not the parameterization
   artefact Part 1 concluded — see §A0, which corrects Part 1's §3 inference.
   Attitude overshoot 65 % → 34 %, rise time *kept* at 0.175 s.

Also reverted from Part 1: `TIME_TRAJECTORY` back to **50** on all three attitude
axes, and `COMMAND_FILTER` 10 → **1**, both matching the reference. 50 is the
flight-validated value; the 250 from Part 1 was not, and with the damping fixed
it is no longer needed. **This is the better answer to Part 1's finding** —
Part 1 cured the overshoot by slowing the reference filter to the loop; the real
cause was that the loop was under-damped by exactly 2×.

## What did NOT work — three reverted attempts

Recorded because each looked obviously right and each broke the vertical loop.

1. **Raising `GZ_INTEGRATION_WINDOW` 4 → 50** (the kickoff's H2). Altitude swung
   10–30 m against a 3 m setpoint, `cmd_z` bang-banging both clamps.
2. **Switching gz to `DECOUPLED = TRUE` with Simulink's `Kd = 5.6`, ref window 50
   and command filter 1.** Same failure.
3. **Filtering the z measurement with the same 3 Hz Butterworth as x/y.** Same
   failure — 4–6.5 m excursions on a 3.4 m setpoint.

The common cause, and the main correction to the kickoff:

> **In the coupled structure, `int_window` is not a noise filter — it is the
> estimator bandwidth, and the estimator IS the feedback path.** Simulink can run
> a 50-sample window on z because *its* z axis is decoupled, where the poles live
> in an explicit iPD and the estimator only cancels disturbance. H2 is a
> decoupled-channel argument; firmware's z is coupled, so applying it detunes the
> controller by 12×.

And for the filter: a 3 Hz Butterworth is ~60 ms of lag against a channel whose
reference filter and estimator window are both 8 ms. **Before filtering an MFC
measurement, compare the filter's lag with that axis's `int_window` and
`ref_window` in milliseconds.** If it is comparable or larger, it will
destabilise the loop; slow the whole channel coherently instead. `guidance_mfc.c`
now carries this note where z is read.

## Left alone deliberately

- **Horizontal gains `Kp 2 / Kd 25`** against Simulink's `75 / 150`. This is the
  one axis pair where a bare comparison is legitimate (same structure, same
  `alpha = 18.75`), so the 37×/6× gap is real. Kept anyway: firmware holds 4.5 cm
  at 5 % rail, and adopting the reference gains would buy ~2 cm at the cost of
  running the bank command into saturation the way Simulink does.
- **`GX/GY_INTEGRATION_WINDOW = 600`** vs Simulink's 100. Horizontal is quiet and
  unsaturated as it stands.
- **`alpha`** on every axis. Constant `alpha` error is absorbed by the estimator.
- **`est_hold_time = 0.1`** vs Simulink's 0/0.01 (H6). Longer is safer and it
  never misbehaved.

## Final configuration

`anton_mfc.xml` vs HEAD — the whole control-law change is:

```xml
<define name="ROLL_DERIVATIVE_GAIN"    value="12."/>   <!-- was 6.  -->
<define name="PITCH_DERIVATIVE_GAIN"   value="12."/>   <!-- was 6.  -->
<define name="ROLL_COMMAND_FILTER"     value="1."/>    <!-- was 10. -->
<define name="PITCH_COMMAND_FILTER"    value="1."/>    <!-- was 10. -->
<define name="YAW_COMMAND_FILTER"      value="1."/>    <!-- was 10. -->
```

plus `mfc_core.c`'s estimator tap. Everything on the z and horizontal channels is
**unchanged from the flight-validated configuration**, and `TIME_TRAJECTORY` is
back at its flight-validated 50. `guidance_mfc.c` carries a comment only.

## Flyability

The user's standing note is that the current attitude behaviour did translate to
flight, so there is no reason to distrust the SITL–real gap. Consistent with
that, this session **narrowed** the diff against the flight-validated config
rather than widening it: Part 1's non-flight-validated `TIME_TRAJECTORY = 250` is
gone, and what remains is one damping coefficient, three command filters and one
estimator tap. The mass/inertia placeholders remain the standing caveat — they
set the clamps and the hover normalisation, not the pole locations.

## Still open

- The horizontal `Kp/Kd` gap (2/25 vs 75/150) is real and unexplained. Worth one
  run at an intermediate value to see whether hover error improves before the
  command starts railing.
- `GZ_KD` firmware 12 vs Simulink applied 5.6 — untested in isolation; the z loop
  is performing well, so it was not touched.
- Bench-measure `MODEL_MASS` / `INERTIA_*`.
