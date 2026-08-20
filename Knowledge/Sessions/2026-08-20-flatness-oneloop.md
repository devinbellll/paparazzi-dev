# 2026-08-20 — FINDI flatness oneloop

Built the flatness spine and the FINDI oneloop controller, and generalised the
MIMO core to arbitrary channel width. Spec followed:
`Knowledge/Plans/Flatness Quad Oneloop (FINDI then FMFC).md`.

Unusually, this was done from the vault root rather than by a scoped launch in
this repo — the author lifted that boundary for the session so one agent could
see the canon tree, `Generic_Quad`, `MFC_SISO` and this repo at once. The plan
and the task briefs written for the scoped path still stand.

## What landed

**`mfc_core_mimo` is now width-parametric.** `MFC_MIMO_N` (2) became
`MFC_MIMO_N_MAX` (3) as a *capacity*, with a runtime `n` on each channel and
every loop bounded by it. `mfc_mimo_init()` takes the width. The 2×2 cofactor
inverse is untouched and a 3×3 cofactor was added alongside. `heol_mimo` sets
`n = HEOL_MIMO_N = 2` and is otherwise unchanged.

**`flatness_quad.{c,h}`** — the ZXY force transform and the quaternion attitude
error, transcribed from `quad_flatness_force_transform.m` and
`quad_indi_attitude.m`. Pure functions; allocation deliberately left out
because it is airframe-configured.

**`oneloop_findi.{c,h}` + module XML + `anton_findi.xml`** (ANTON_FINDI,
ac_id 222, in `userconf/ENAC/conf_mfc.xml`).

## Three things worth carrying forward

**1. A latent bug in `mfc_siso_reset()`, found by accident, now fixed.**
The scalar golden check started failing after the MIMO edit with an error that
*changed between builds* — 5.7e7, then 2.5e33. That signature is uninitialised
memory, and the cause was not the edit: `mfc_siso_reset()` cleared `error[]`,
`z[]`, `estimator_num[]` and `estimator_den[]` but **not `setpoint_trajec[]`**,
which the reference smoother reads at k=0. The MIMO struct grew, the stack
layout shifted, and the garbage the test had been silently reading changed.

The firmware consequence is real and separate from the test: every channel is a
file-scope static, so it is zero at boot — but `mfc_siso_reset()` is also the
**re-entry** path, and on re-entry the smoother was carrying the previous
engagement's setpoint into the first `ddot_sp`. `oneloop_mfc_guidance_enter()`
already works around this by hand-seeding `mfc_gz.setpoint_trajec[0..2]`, which
is a per-axis patch for a general defect. Every caller seeds *after* reset, so
clearing inside it is safe. `mfc_mimo_reset()` always did clear its equivalent.

**2. `float_mat_invert()` must not be used for alpha.** It was the obvious
candidate for the N-generalisation. Its Gauss-Jordan has no partial pivoting —
it divides by `a[i][i]` directly — and the HEOL horizontal alpha is
*anti-diagonal at level* (`[[0, −9.81], [9.81, 0]]`), so `a[0][0]` is exactly
zero and the first division is by zero. Cofactor formulas have no such failure.
Noted at the call site so it does not get "simplified" back.

**3. The k*pi branch is ON here and OFF in Simulink.** The paper prescribes it
(`flatness.tex` eq. 17: choose k so the resulting b_y best matches the current
one), and it also resolves the fully degenerate case where any roll satisfies
the constraint — so with it on there is no gimbal-lock error branch at all. The
MATLAB block defaults it OFF so it reproduces the frozen `quad_flatmap` path bit
for bit. **A firmware/sim comparison must set the sim's `findi_use_by_branch` to
match**, or the two disagree past 90° of tilt with nothing reported.

Free fall (`|f_i^c|/m < 1e-6`) has no treatment in the paper — checked, it only
covers the two attitude-*parameterisation* degeneracies — so the C holds the last
valid commanded attitude, passes the computed collective force through, and
raises a status flag with a counter.

## Verification

| | |
|---|---|
| `tests/run_flatness_quad.sh` | 22 analytic property checks, all pass |
| `tests/run_mimo_golden.sh` | all pass, **same error figures as before** the width change (7.277e-04 / 1.181e-03 / \|det\| 51.54) |
| `pprz.sh build ANTON_FINDI nps` / `ap` | both produce an `.elf`, no warnings in the new files |
| `ANTON_HEOL`, `ANTON_MFC`, `ANTON_ONELOOP` (nps) | all still build after the core changes |

**Not verified: anything about whether it flies.** No SITL run was made. The
`heol-firmware-mimo-horizontal` precedent is the reason to say that plainly —
that port was bit-exact against its traces and diverged from the moment the loop
engaged.

**No golden trace.** This environment has no MATLAB, so the closed-loop
`findi_quad` comparison the plan called for could not be made and the checks are
hand-derived properties instead. They are chosen to discriminate (the round-trip
`R_b_i·zb = body z` catches a transposed or wrong-sequence rebuild; the past-90°
case asserts branch-on and branch-off actually differ, so it cannot pass
vacuously) but they are weaker than a trace. Tracked as
`verify-flatness-quad-against-sim` on the vault board.

## Structure notes

**Where the oneloop shape actually lives**: the guidance hooks
(`guidance_h_run_*`, `guidance_v_run_*`) only latch their argument pointers and
return placeholders. The whole chain runs in `stabilization_attitude_run()`,
which the framework calls after both and which owns `cmd[]`. Nothing passes
through a `StabilizationSetpoint`/`ThrustSetpoint` — that seam is exactly what
forces the exact flatness inversion to be replaced by `accel_to_att_sp()` in the
split stacks.

**Bring-up affordance**: `findi_linear_enabled` off drops the linear half and
flies attitude-only from the RC/nav setpoint, exercising the angular half,
allocation and effectiveness on their own. That is rungs 2–3 of the isolation
ladder, and it is a GCS setting rather than a rebuild.

**Tilt limiting is on the commanded force, before the transform**, not on
phi/theta after it: the transform emits `(f, q_cmd)` as a consistent pair and
clamping the attitude afterwards would leave a collective force that no longer
matches the attitude it was computed with.

**Allocation is reuse, not new code.** `stabilization_heol.c` already
established that the identified accel-convention `G1`, scaled row-wise by the
inertias and mass, is the SI (N·m / N per pprz unit) matrix — and for a square
well-conditioned X-quad `G1` the Moore–Penrose inverse *is* the exact inverse the
canon specifies. Same construction, computed once at init since nothing here is
adaptive.

## Next

`oneloop_fmfc`: swap the two increments for MIMO HEOL/MFC brackets with
α_fi = I/m and α_m = I⁻¹. It starts with no open questions — the width work is
done, and the "missing unit delay" question closed (the Simulink `Memory` block
is causality/algebraic-loop enforcement; in C the estimator's delayed term is
just `u[k-1]`).

Before any of that: SITL, one rung at a time, and do not read offline agreement
as evidence the loop flies.

---

# SITL (same session)

**It flies.** Rungs 2–6 pass, and the run turned up two real defects, one of them
outside this controller.

## Ladder

| rung | result |
|---|---|
| 2–3 attitude, `--rc_script 0` | takeoff, 2 m hold to ~1 cm, attitude within ±0.01 rad |
| 4–5 vertical + horizontal | holds Standby, recovers from the trajectory |
| 6 `Flat_Traj_Demo` | tracks; error below |
| 7 noise on/off | indistinguishable |

`Start Engine,Takeoff,+3,Standby,+10,Flat_Traj_Demo,+40,Standby`, position error
RMS in metres (x / y / z):

| | trajectory 25–60 s | late hold t>65 s |
|---|---|---|
| **before the nominal fix** | 2.734 / 2.536 / 0.645 | 0.109 / 0.187 / 0.014 |
| **after** | 0.197 / 0.172 / 0.344 | 0.107 / 0.190 / 0.007 |
| **after, all NPS noise zeroed** | 0.196 / 0.171 / 0.344 | 0.106 / 0.191 / 0.007 |

Noise on and off agree to the third decimal — this loop is not noise-sensitive
at this tuning. Nothing here is tuned; `kx = 1.0`, `ka = 0`, straight from the
Simulink reference.

Worth stating against the HEOL precedent: the comparable HEOL number was
**48 / 79 m and still diverging** in the late hold. This does not diverge.

## Defect 1 — the allocator was returning an adjugate, not an inverse

First run: aircraft on the ground, every command exactly zero. The chain was
fine all the way to `v = [0,0,0,-9.68]`; the allocated `u` came out **0.017
instead of ~2017**, a factor of 1.2e5.

`float_mat_inv_4d()` (`math/pprz_algebra_float.h:919`) **returns 0 on success
and 1 on failure** — the opposite of the usual convention — and on the failure
path it leaves `float_mat_adjoint_4d()`'s output in place, *undivided by the
determinant*. It bails when `|det| < 1e-4`. Our normalised `G G^T` has
det ≈ 8.5e-6, because the SI rows differ by orders of magnitude (moment rows
~2.7e-4, thrust row ~1.2e-3) and largest-element scaling does not fix that.

Fixed by normalising each row of `G` to unit norm, which makes `G G^T` have a
unit diagonal and a determinant of O(1). Exact, not approximate: with
`D = diag(1/||g_r||)`, `pinv(D G) = pinv(G) D^-1`, so dividing column *r* by
`||g_r||` recovers it. The return value is now checked, and a failed inverse
disables allocation rather than flying a garbage matrix.

**This affects `stabilization_heol.c:1341` and `stabilization_indi.c:1179`
too** — same call, same magnitudes on the HEOL side, return value ignored in
both. HEOL defaults to WLS so it does not fly this path, but
`stabilization_heol_use_pseudo_inverse` is a live GCS setting: flipping it in
flight would silently allocate through a matrix that is wrong by five orders of
magnitude. Not fixed here — it is not this task's code and it deserves its own
change.

## Defect 2 — the flat trajectory's terminal nominal is not a rest condition

With the allocator fixed the aircraft flew, but parked **2.7 m from its target,
dead level**, with a persistent `zeta_e_y = -0.334 rad` and essentially zero
commanded moment. Nothing diverged; it just sat there.

`dw_ref_y` was pinned at **+23.92 rad/s²** for the whole 36 s hold, cancelling
`k_rate * k_att * zeta_e = -24.05` almost exactly. The loop was doing precisely
what it was told.

`nav_flat_traj_run()` holds the last table sample once the trajectory ends —
deliberately, so the setpoint stays at the endpoint. Its comment calls that "the
last (zero vel/accel) sample", which is true of the kinematic fields and **false
of the nominal inputs added later**. A min-snap polynomial pins position through
jerk and heading-rate at its boundaries; it does not pin snap or heading-accel,
and `pdot/qdot/rdot` derive from those, with `M = I*wdot` after them. Terminal
values in this table:

```
p/q/r_ref      0, 0, 0          pinned, fine
pdot/qdot/rdot 0, +23.92, -2.09  NOT pinned
Mx/My/Mz_ref   0, +0.163, -0.028 NOT pinned
thrust/phi/theta  -m*g, 0, 0     pinned, fine
```

Fixed in `nav_flat_traj.c`: zero the angular-acceleration and moment nominals
once `flat_traj_done`. Not a workaround — at a rest-to-rest endpoint the
physically correct nominal angular acceleration and moment *are* zero.

**This is the t=0 question, resurfacing at the other end.** When the exporter
was widened, the nonzero snap at sample 0 was examined and dismissed as
harmless — correctly, because nothing consumed those columns then. This task is
the first consumer. The same non-pinned-boundary property is harmless at the
start, where the sample is passed through, and harmful at the end, where it is
held. **[[heol-firmware-attitude-channels]] will hit this too** — it consumes
the same fields.

## What the instrumentation was worth

The first run gave "on the ground, commands zero" and nothing else; 113 scope
variables later, both defects were one table read each. Both were invisible in
the build (clean) and in the offline tests (all passing). Neither could have
been found by reasoning about the code — the first needed the numeric output of
a library function whose contract is inverted, the second needed a value from a
generated data table.

## Not done

- No tuning. `kx`, `ka`, `k_att`, `k_rate` are the Simulink values.
- The ~0.2 m trajectory error and the slow drift during the hold are untuned
  outer-loop behaviour, not investigated.
- Still no golden-trace comparison (`verify-flatness-quad-against-sim`).

---

# HEOL: the measured-velocity derivative (same session)

The gap the author remembered is real and was still open in firmware. The
attitude axes have taken their D term from the gyro since the HEOL stabilizer
landed (`*_USE_MEASURED_RATE`, default true). **Neither guidance channel ever
got the same option** — the vertical differenced its own epsilon history, and
the horizontal MIMO core had no external-derivative path at all. The INS
velocity sat there unread: the same published-but-unread structure the Simulink
work found on both stacks.

Added: `use_external_derivative` / `external_derivative[]` on `mfc_core_mimo`,
`heol_mimo_set_derivative()`, and the wiring in both guidance channels.

## Both default OFF, and that is the finding

This is **a gain change, not a signal swap**. Differencing a position quantised
to `INT32_POS_FRAC` at 500 Hz gives a derivative substantially corrupted by
quantisation, which attenuates the D term. The INS velocity delivers the full
`kd` — and `kd` was tuned against the attenuated one.

ANTON_HEOL, `Flat_Traj_Demo`, position error RMS x/y in metres:

| | trajectory 25–60 s | late hold t>65 s | final z |
|---|---|---|---|
| both off (baseline) | 5.67 / 7.73 | 2.797 / 2.687 | −1.997 |
| **horizontal only** | 7.13 / 10.45 | **1.196 / 0.992** | −1.999 |
| both on | 7.02 / 9.65 | 9.37 / 20.34 | **−1784.7** |

With both on, the vertical loop is stable to 1 cm in hover and goes into a
growing oscillation *the moment the reference starts moving* — `F_hat` 1.5 → 9.4,
thrust command swinging −8.4 to −23.7 N.

**The wiring was verified, not assumed.** Before concluding "needs a retune" I
instrumented `z_ref`, `zd_ref`, a numerical derivative of `z_ref`, and `vz_meas`:
`zd_ref` is the true derivative of `z_ref`, and `vz_meas` is the INS velocity.
The signal is right; the gain is not. That probe was removed afterwards.

The horizontal channel alone **improves the late hold ~2.4×** with the vertical
untouched, so the retune burden is on the vertical. That is worth knowing before
anyone starts a tuning campaign.

## What this does and does not say about the divergence

It does **not** fix `heol-xy-divergence-diagnosis`. The horizontal loop still
diverges during the trajectory in every configuration above; the hold gets
better, the trajectory segment does not. Treat the 2.4× as a lead, not a cure.

Note also that these baseline numbers (5.67/7.73 traj, 2.80/2.69 hold) are much
better than the 48/79 m recorded on 2026-08-15, because this build includes
`cfede3078` — the `est_use_presat_command` fix that was written and never
re-flown. **That fix appears to work**, and this is the first flight of it.
Someone should confirm that deliberately rather than inheriting it from a
side-observation in a different task's session note.
