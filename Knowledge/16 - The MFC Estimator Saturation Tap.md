# The MFC Estimator Saturation Tap

**The single most consequential defect found in the MFC stack.** One boolean, in
one line of `mfc_core.c`, applied to all six MFC axes and to every airframe using
them. Found and fixed 2026-08-17; the same defect had been diagnosed
independently on the sibling HEOL controller.

If you read nothing else: **the MFC disturbance estimator must be fed the command
the plant actually received — after the output filter and after the clamp.**
Feeding it the pre-saturation command turns every saturating axis into a
limit-cycle generator.

## The mechanism

`mfc_core.c` implements, per axis:

```
u = (-F_hat + rddot - fb) / alpha
```

`F_hat` is an algebraic estimate of everything the model does not know. It is
built by inverting the assumed plant `edd = F + alpha*u`, which requires the
estimator to know `u` — the command the plant *received*. In `mfc_core.c` that
enters through the `d2u` term:

```c
float d2u = mfc_stt->time * mfc_stt->time * mfc_stt->command_est;
```

and `command_est` is chosen at the end of each step:

```c
mfc_stt->command_est = mfc_stt->est_use_presat_command ? mfc_stt->command_presat
                                                       : mfc_stt->command[0];
```

- `command_presat` — the raw model-inversion output, **before** the command
  filter and **before** the clamp.
- `command[0]` — the applied command, **after** both.

### Why the pre-saturation tap diverges

While the command is inside its limits the two are identical and nothing happens.
The moment the clamp bites they separate, and the feedback loop through `F_hat`
inverts:

1. The law asks for `u_raw`; the clamp applies `u_sat < u_raw`.
2. The estimator is told the plant received `u_raw`.
3. The plant responds to `u_sat`, so it accelerates **less** than the estimator
   expects.
4. The estimator can only explain the shortfall as an external disturbance, so
   `F_hat` moves to account for it.
5. `u = (-F_hat + …)/alpha` therefore commands **harder in the same direction**.
6. Which saturates further. Go to 2.

`F_hat` winds up exactly like an unprotected integrator, except there is no
integrator to freeze and no anti-windup branch to reach for — the windup lives
inside the estimator. The axis then bangs between rails, because the only thing
that reverses `F_hat` is the vehicle overshooting far enough to flip the error.

**This is windup, and the applied-command tap is its anti-windup.** It is the
same principle as conditioning an integrator on the actuator's real output; MFC
just hides the integrator inside an algebraic estimator, which is why it is easy
to miss.

### Why it is worst on the horizontal axes

Saturation frequency decides exposure. On ANTON_MFC the horizontal command clamp
is `±0.7·m·g·sin(MAX_BANK)` = ±1.879 N, and the upstream Simulink reference
measured its bank command **on the rail 73–92 % of a noisy run, including in
hover**. An axis that spends most of its life clamped spends most of its life in
the divergent loop above. Attitude, which rarely saturates, was almost unaffected
— which is exactly why this survived so long: the layer people looked at worked.

## What was wrong, and the fix

`mfc_siso_init()` set:

```c
mfc_stt->est_use_presat_command = true;   /* WRONG */
```

Nothing in `stabilization_mfc.c` or `guidance_mfc.c` overrode it, so **all six
MFC axes ran the pre-saturation tap.** (`guidance_heol.c` already exposed a
per-axis knob, `GUIDANCE_HEOL_*_EST_PRESAT` — that is how the defect came to be
found on HEOL first.)

The fix is the default:

```c
mfc_stt->est_use_presat_command = false;  /* tap the applied command */
```

The field is kept, so an axis can still be flipped back per-axis — it is a live
structural question worth being able to A/B, not a compatibility shim.

## Corroboration

- **Upstream Simulink** (`MFC_SISO/functions/mfc_siso.m`) has always done it the
  correct way: `mfc_siso.step` ends `state.u_km1 = u`, where `u` is post-EMA and
  post-clamp. The model that flies position uses the applied command.
- **HEOL** hit the identical defect, documented in
  `Generic_Quad/Knowledge/RETRO-2026-08-17-heol-quad.md` as "the command blocks
  are fed the unsaturated command".
- **ANTON_MFC SITL, 2026-08-17.** With this fix (plus a roll/pitch damping
  correction) the stack went from "XY in a large limit cycle, rung 2" to rung 5:
  hover hold **4.8 / 4.2 cm rms** with the command at **5 % of its rail**, and
  `Flat_Traj_Demo` tracking to 6.1 / 5.7 / 0.8 cm. See
  `Knowledge/Sessions/2026-08-17-anton-mfc-sitl-tuning.md` Part 2.

The two changes were made together, so the split of credit between them is not
separately measured. The mechanism above is what makes this one the structural
fix and the damping one a tuning correction.

## How to recognise it in a log

The signature is specific enough to name:

- Command **pinned at a clamp**, alternating between both rails.
- `F_hat` (`fk_*` in the `/uav` schema) **orders of magnitude above its normal
  range** — on ANTON_MFC's attitude axes normal is ~40, a wound-up axis reached
  4.5e4.
- Tracking error **large and oscillatory**, but the loop is not "noisy": the
  oscillation is slow and coherent, not high-frequency.
- Reproduces **identically with all sensor noise zeroed**. Windup is structural;
  noise is not required.

That last point is the cheapest discriminator, and it is the one worth running
first. See the `NPS_NOISE_SCALE` switch in
`conf/simulator/nps/nps_sensors_params_anton_mfc.h`.

## What this does NOT fix

Do not reach for this when the symptom is a *fast, small* oscillation or a noisy
command with the clamp never touched. If the command is not saturating, the two
taps are identical and this changes nothing. Two neighbours that look similar and
are not this:

- **Reference-filter / loop bandwidth mismatch.** A reference filter faster than
  the loop it drives overdrives the axis through the `rddot` feedforward.
  Compare `tau_ref = W/f_s` against `1/wn`. (Part 1 of the same session.)
- **A window changed on a coupled axis.** In the coupled structure `int_window`
  is the *estimator bandwidth*, and the estimator is the feedback path — changing
  it detunes the controller rather than filtering it. Check `decoupled` first.

## Scope

`mfc_core.c` is shared by `stabilization_mfc` (roll/pitch/yaw), `guidance_mfc`
(gx/gy/gz) and, via `heol.c`, the HEOL stack. `mfc_core_mimo.c` carries its own
copy of the same field and **still defaults to `true`** (`mfc_core_mimo.c:56`) —
it should get the same treatment when the MIMO path is next exercised.

Related: [[15 - Simulink MFC Quad ↔ ANTON_MFC Firmware Correspondence]] §A1,
[[Sessions/2026-08-17-anton-mfc-sitl-tuning]], bug-272.
