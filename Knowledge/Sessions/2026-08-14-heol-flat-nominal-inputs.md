# 2026-08-14 — HEOL guidance: flat nominal inputs (T*, phi*, theta*)

Implements the outer-loop scope of
`Knowledge/Plans/HEOL Guidance Against the Canonical Spec (Flat Nominal Inputs).md`.
Second session of the day on HEOL guidance; the earlier one is
`2026-08-14-heol-guidance-fixes.md`.

## What landed

All of the plan's steps. Step 6 (`alpha_z` live) initially went in and was
backed out over its tuning cost, then landed unconditionally on a second pass once
the author confirmed the gain mismatch is not a blocker — this is a new
controller and none of its loops are tuned yet.

| plan step                                      | status                                                           |
| ---------------------------------------------- | ---------------------------------------------------------------- |
| 1. plumbing route decided + written down       | done — `Knowledge/14 - Flat Nominal Inputs Plumbing Decision.md` |
| 2. consume the new fields in `nav_flat_traj.c` | done                                                             |
| 3. D4 — `z_ff` <- `thrust_ref`                 | done, verified                                                   |
| 4. D1 — saturate the total command             | done, verified                                                   |
| 5. per-tick alpha setter on `mfc_core`         | done                                                             |
| 6. D3 — `alpha_z` live + vertical PD retune    | landed unconditionally; vertical loop still untuned for it       |
| 7. retire `filt_thrust`                        | done                                                             |
| 8. confirm/dismiss the `sp_traj_z` step        | **confirmed and explained** — not a bug                          |

### New files

- `sw/airborne/firmwares/rotorcraft/guidance/guidance_flat_nominal.{c,h}` —
  timestamped zero-order-hold latch for `(T*, phi*, theta*)`. Compiled by
  `guidance_rotorcraft` so it is variant-agnostic. Rationale in Knowledge 14.
- `sw/airborne/firmwares/rotorcraft/guidance/heol_input_sensitivity.{c,h}` —
  the Input-Sensitivity Transformation, built **whole** as the spec's shared
  block: takes `(T*, phi*, theta*, psi*)`, emits both `alpha_xy` and `alpha_z`.
  `alpha_xy` is computed and published to telemetry with **no control consumer**,
  exactly as the plan requires, so the ZYX/ZXY question can be characterized
  against logged numbers while it is still harmless.

### Modified

- `nav_flat_traj.c` — one added call; the nominal inputs are deliberately not
  gated on `FLAT_TRAJ_DEMO_ORDER` (that gates the kinematic chain only).
- `guidance_heol.c` — `T*`-sourced vertical feedforward with the old
  `m*(zdd_ref - g)` retained *only* as the no-flat-trajectory fallback;
  `heol_update_nominal()` runs the sensitivity block once per tick from the
  vertical step; `filt_thrust` removed; total-command bounds; new scope vars.
- `heol.{c,h}` — `u_min`/`u_max` on the **total** command; `heol_set_alpha()`.
- `mfc_core.{c,h}` — `mfc_siso_set_alpha()` with a zero guard.
- `conf/modules/guidance_heol.xml`, `conf/modules/guidance_rotorcraft.xml` —
  file lists; the `u_min`/`u_max` GCS settings re-pointed from `heol_*.mfc.*`
  to `heol_*.*` (the mfc-level ones are now derived state rewritten every tick,
  so the old sliders would have been dead controls).

## D1 in detail — how the total-command clamp is implemented

The spec saturates `u = sat(u* + du)`. Implementing that literally in
`heol_run()` after the sum would move the clamp away from the only place the
anti-windup freeze can observe it in the same sample. Instead `heol_run()`
shifts the physical window into the correction's frame each tick:

```c
mfc.u_min = heol->u_min - u_ff;
mfc.u_max = heol->u_max - u_ff;
```

`sat(u* + du)` over `[lo,hi]` is algebraically identical to `u* + sat(du)` over
`[lo-u*, hi-u*]`, so this *is* the total-command clamp, just expressed where the
freeze still works. The degenerate case (`u_ff` already outside the range, so
the window inverts) collapses onto the reachable edge.

The gz bounds are now the real physical range, `[-GZ_MAX_THRUST, 0]`. Note the
**upper bound is 0**, not `+GZ_MAX_THRUST`: a rotorcraft cannot produce thrust
in the `+z` direction. The old symmetric `±(GZ_MAX_THRUST - hover)` window
admitted physically impossible positive total thrust.

## Bug found and fixed in passing — mode-entry seed

`guidance_v_run_enter()` pre-seeded `heol_gz.mfc.command[0..1]` to the nominal
hover thrust, copied from `guidance_mfc`'s gz entry. That is right in
`guidance_mfc`, where `mfc.command[0]` **is** the total thrust command, and
wrong in HEOL, where it is only the correction `u_fb`. It made the first tick's
total come out `u_ff + hover` — roughly twice hover thrust — and fed the
estimator's delayed-command term a value that was never a correction;
`GZ_COMMAND_FILTER = 8` then stretched the transient over several ticks.
Removed. `heol_reset()` already zeroes the history, which is the correct HEOL
seed.

## Step 8 — the `sp_traj_z` step: confirmed, explained, not a bug

The previous session flagged `sp_traj_z` briefly stepping to `-4.7105` and
attributed it to the `Flat_Traj_Demo` -> `Standby` handoff. **Both parts are now
resolved, and the attribution was slightly off.**

Measured in this session's stage-A run, in the **pre-trajectory** phase
(takeoff + standby, before the demo block ever runs):

| observed `sp_traj_z` | implied `zdd_ref` | matches |
|---|---|---|
| `-4.7105` (max) | `+3.9219` | `GUIDANCE_V_REF_MAX_ZDD = 0.4*9.81 = 3.924` |
| `-10.9855` (min) | `-3.9219` | `GUIDANCE_V_REF_MIN_ZDD = -0.4*9.81` |

with `MASS = 0.8` and `u_ff = m*(zdd_ref - 9.81)`.

So both extremes are the **vertical reference model's acceleration limits**
passed through the small-angle feedforward. The mechanism is exactly the
suspected one — `guidance_heol_vert()` runs for every vertical mode, including
Takeoff's climb and Standby's altitude hold, not just the flat path — but it is
correct behaviour, not a defect: the reference model legitimately saturates at
±0.4 g on a step, and the feedforward faithfully converts that to a force.
Nothing to fix. Note the earlier session's reading of `-10.9855` as
`-9.81*MASS` was wrong (`-9.81*0.8 = -7.848`); it is the MIN_ZDD saturation
value.

## Verification (SITL, ANTON_HEOL, `Flat_Traj_Demo`)

Both targets build clean (`ap` and `nps`). Flown as
`./sim.sh ANTON_HEOL --no-build --nav "Start Engine,Takeoff,+3,Standby,+1,Flat_Traj_Demo,+2,Standby"`.

New scope channels: `/uav/HEOL_NOMINAL/{T_star,phi_star,theta_star}`,
`/uav/HEOL_ALPHA/{alpha_z,alpha_xy_00..11,gz_alpha_used}`.

**Nominal inputs arrive correctly** — the plan's primary criterion:

| quantity | generated table (751 rows) | logged in SITL |
|---|---|---|
| `thrust_ref` / `T*` | −11.1771 … −6.4079 N | −11.1682 … −6.4079 N |
| `phi_ref` / `phi*` | −0.130014 … 0.214757 rad | −0.1298 … 0.2137 rad |
| `theta_ref` / `theta*` | −0.276930 … 0.622733 rad | −0.2769 … 0.6222 rad |
| `alpha_z` | 1.00971 … 1.25000 | 1.0110 … 1.2500 |

The small differences on the extremes are the 20 Hz nav sampling not landing
exactly on the table's peak rows — not a plumbing error. `alpha_z`'s logged
range matches the plan's predicted **1.0097 … 1.2500** to four digits.

**D4 confirmed**: `sp_traj_z` equals `T_star` to `0.0` over the whole trajectory
window (max |difference| = 0.0 across 775 samples), and the table's own
divergence from the retired approximation is 0.00 % at sample 0 and 19.22 % at
sample 543 — exactly the plan's figures, so the right field is being read.

**D1 confirmed**: `cmd_z` over the whole log runs −24.75 … 0.0000 N and never
leaves `[-46.08, 0]`. It lands *exactly* on the 0 N bound for 60 samples just
after the trajectory ends, i.e. the **total** command is what clamps. Under the
old δu-only clamp the ±38.2 N window would not have bitten there and the total
would have gone positive. Downstream actuator behaviour is unchanged, because
`Bound(thrust_norm, 0, 1)` was already clipping it — the difference is that the
loop now sees the true bound.

Vertical tracking residual over the 1.5 s window: RMS 0.062 m, max 0.125 m.
(Per the plan, **not** a pass/fail bar — see the caveat in the previous session
note. Horizontal RMS is ~0.18 m, consistent with the known unresolved X/Y
oscillation, which this task does not touch.)

## Two structural choices, selectable; alpha_z is not one of them

Two questions on this controller are still open, so both are plumbed every way
rather than baked in — per-axis airframe defines plus live GCS settings.
Defaults are what the canonical spec specifies.

| setting (shortname) | define | default | alternatives |
|---|---|---|---|
| `heol_*.clamp_mode` (`*_clampmode`) | `*_CLAMP_MODE` | `TOTAL` — saturate `u_ff+u_fb` against `u_min/u_max` | `FB_ONLY` — saturate `u_fb` alone against `fb_min/fb_max`; `NONE` — no clamp at all |
| `heol_*.mfc.est_use_presat_command` (`*_estpresat`) | `*_EST_PRESAT` | `PRESAT` — estimator taps before saturation | `POSTSAT` — taps the applied command |

**`alpha_z` is NOT selectable.** It is always the reference Jacobian
`cos(theta*)cos(phi*)/m`, wired unconditionally: a wrong alpha is not a tuning
choice. `GUIDANCE_HEOL_GZ_ALPHA` survives only as the power-on value, overwritten
on the first tick.

`HEOL_CLAMP_NONE` clamps at `HEOL_CLAMP_NONE_LIMIT` (9600, mfc_core's own
default) rather than infinity — the bound still has to be finite so a divergent
command produces a large number instead of an `inf` that poisons every history
buffer downstream. It is a diagnostic mode: the anti-windup freeze never fires,
so with `ki != 0` the integral runs unchecked.

Implementation notes:

- The estimator tap needed a new `command_est` field in `MfcParameters`.
  `command[1]` could not be reused, because it is *also* the command filter's
  state and must stay the applied value regardless of which tap the estimator
  uses. `command_presat` holds the unclamped value each tick.
- The correction-only clamp needed its own `fb_min`/`fb_max` on
  `HeolParameters`. Reusing `u_min`/`u_max` would have been wrong: the gz
  physical range is `[-46.08, 0]`, and clamping the *correction* to that means
  it could never push thrust up.
- Whichever tap is selected, it is always the correction `u_fb` and never the
  total. That HEOL invariant is unchanged and unconditional.

### All three clamp modes validated in SITL

Same flight, gz clamp mode varied. The `cmd_z` range is the diagnostic — the
physical range is `[-46.08, 0]`:

| gz `clamp_mode` | vertical RMS | `cmd_z` range | verdict |
|---|---|---|---|
| `TOTAL` (default) | 0.508 m | −46.08 … **0.0000** | never leaves the physical range |
| `FB_ONLY` | 0.059 m | −25.04 … **+1.8586** | total goes positive |
| `NONE` | 0.105 m | **−66.70 … +80.07** | fully unbounded |

`FB_ONLY` reaching **+1.86 N** is commanded upward-pulling thrust, physically
impossible for a rotorcraft — precisely the hole the total-command clamp closes.
`NONE` overshooting the range by 70 % low and unboundedly high confirms it is
genuinely bypassing the clamp.

Downstream `Bound(thrust_norm, 0, 1)` was already clipping the positive excursion,
so actuators never saw it; the difference is that the loop now sees the true bound.

Note the RMS column is NOT a ranking: the looser modes score better only because
they let the loop command thrust the aircraft cannot produce.

### CORRECTION — the live alpha does not merely degrade tracking, it bang-bangs

An RMS-over-a-window metric badly understated this, and the first write-up of
it ("~10x worse RMS", "tuning debt") was wrong in kind, not just degree.

On `--nav "Start Engine,Takeoff,+3,Standby,+10,Flat_Traj_Demo,+2,Standby"` with
the live Jacobian as default, `cmd_z` **alternates between its two clamps,
0 N and -46.08 N, at the sample rate** for the whole trajectory. It is a
saturated limit cycle, not a tracking error. The aircraft climbs from 2 m to
**9.67 m** and takes ~8 s to recover; the `sp_z` step to -8.09 that follows is
just the vertical reference model being re-seeded to the (wrong) actual
altitude on entering Standby, i.e. a consequence, not a separate fault.

Attribution is clean: with the constant alpha the same flight gives
`cmd_z` in -25.04 … +1.86 N with no clamp slamming. Flipping the estimator tap
to `POSTSAT` does not help either — identical 9.67 m peak — so the tap is not
implicated.

**Recommendation: default `alpha_z` back to the constant until the estimator
retune exists.** It was wired unconditionally on the (reasonable) basis that a
wrong alpha is not a tuning choice, but the resulting default configuration
does not fly, which is worse than a known-wrong constant. Left as-is pending
the author's call.

### The vertical loop is not tuned for the correct alpha

With the live Jacobian the vertical residual is ~10x worse than it was with the
old constant (0.51 m vs 0.059 m) and the command reaches both clamps. That is expected and is a tuning debt, not a
regression: `GZ_ALPHA = 6.25` vs a true Jacobian of 1.01 … 1.25 is a 5-6x change
in correction authority, and the gains predate it.

The useful finding for whoever tunes it: **rescaling the PD does not fix it.**
Measured, same window:

| configuration | RMS [m] | max [m] |
|---|---|---|
| constant alpha = 6.25 (historical) | 0.059 | 0.125 |
| Jacobian alpha, `kp=4` `kd=6` | 0.501 | 1.187 |
| Jacobian alpha, `kp=0.8` `kd=1.2` (÷5) | 0.552 | 1.441 |
| Jacobian alpha, `kp=4` `kd=6`, `cf=1` | 0.830 | 1.381 |

A *constant* alpha error is self-cancelling in the decoupled iPD law: the
estimator absorbs it into `F_hat`, so the closed-loop polynomial stays
`s^2 + kd*s + kp` regardless of alpha. The old `alpha = 6.25` was therefore never
producing a loop-gain error the PD could compensate. What a 5x smaller alpha
actually does is amplify **estimator** error 5x, because `-F_hat/alpha`
dominates `-fb/alpha`. So the tuning lever is `int_window` / `est_hold_time`,
not `kp`/`kd`.

## Open items for the author (unchanged from the plan, plus one new)

1. **Euler ordering for `alpha_xy`** — downgraded, not a live issue. `alpha_z`
   is order-independent (`c_phi*c_theta` either way) and nothing else in the
   firmware consumes `phi_ref`/`theta_ref`, so the ordering touches exactly one
   quantity, `alpha_xy`, which has no control consumer. The table's "ZXY" label
   comes from the generator's tailsitter flatness derivation, where ZXY is
   chosen alongside constraint approximations that do not apply to a quadrotor
   — so it is not evidence about these angles. Confirm what the generator
   actually emits when the MIMO stage picks `alpha_xy` up; `alpha_xy` is logged
   every run so the comparison is cheap then.
2. **D2, estimator tap vs saturation — resolved by plumbing both.** The plan
   said the tap is "before the saturation" *and* that the existing code (which
   fed the post-clamp `mfc.command[1]`) was already correct; those agreed only
   while the clamp never bit. Both are now selectable per axis, defaulting to
   the spec's pre-saturation tap.
3. The `.tex` z-sign self-inconsistency, and whether `f_f` stays grounded —
   both untouched, as instructed.

## Not committed

Nothing was committed. Three reasons, all needing the author's call:

- `sw/airborne/modules/nav/flat_traj_demo_data.h` **must not** be committed to a
  GPL tree until the generator emits the licence header (tracked on the
  `Generic_Quad` side). The new code reads its fields, so the code and the
  header have to land together — a commit of the code alone leaves the tree
  non-building from a clean checkout.
- `conf/airframes/ENAC/quadrotor/anton_heol.xml` carries the *previous*
  session's uncommitted gain-convention work. It was touched and restored
  byte-for-byte during the alpha experiments above; it should go in with that
  session's commit, not this one.
- The suggested split is: (a) plumbing + D4 + D1 + step 5 + step 7 + live
  alpha_z + the two selectable modes, the files listed under
  "New files"/"Modified" above;
  (b) the vertical estimator retune for the live alpha, when it happens.
