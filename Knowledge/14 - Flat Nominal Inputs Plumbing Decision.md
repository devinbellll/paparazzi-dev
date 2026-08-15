# 14 — Flat Nominal Inputs: Plumbing Route Decision

Decision record for how the flat map's nominal inputs `T*`, `phi*`, `theta*`
reach `guidance_heol`. Required by step 1 of
`Knowledge/Plans/HEOL Guidance Against the Canonical Spec (Flat Nominal Inputs).md`.

Date: 2026-08-14.

## The question

The generated table `sw/airborne/modules/nav/flat_traj_demo_data.h` now carries
`thrust_ref`, `phi_ref`, `theta_ref` alongside the kinematic chain. The existing
route for flat data into guidance is

```
nav_flat_traj_run()
  -> guidance_h_set_flat() / guidance_v_set_flat()
      -> gh_set_flat_ref() / gv_set_flat_ref()
          -> gh_update_ref_from_flat_ref() / gv_update_ref_from_flat_ref()   [every tick]
```

and that last step **Taylor-extrapolates** pos/vel/accel forward from the held
jerk and snap (`guidance_h_ref.c:117`, `guidance_v_ref.c:108`).

## Decision

**A separate zero-order-hold latch, set from the same call site as the flat
setpoint.** New files:

- `sw/airborne/firmwares/rotorcraft/guidance/guidance_flat_nominal.{c,h}`
- compiled by the `guidance_rotorcraft` module, so it is always present
  regardless of which guidance variant is selected.

`nav_flat_traj_run()` calls `guidance_flat_nominal_set(s->thrust_ref,
s->phi_ref, s->theta_ref)` on the same tick as the two `set_flat` calls.
`guidance_heol` reads it with `guidance_flat_nominal_get()`.

## Why not the reference model

The nominal inputs are **not derivatives of position**. The reference model's
extrapolation is correct precisely because each state it holds is the integral
of the next one: pos from vel, vel from accel, accel from jerk, jerk from snap.
`T*`, `phi*`, `theta*` are not in that chain and have no held higher derivative.
Pushing them through `gh_update_ref_from_flat_ref()` would advance them using
the *position* jerk and snap — an operation with no meaning.

So the preferred option in the plan ("extend the flat-ref struct with a
nominal-input group that is held rather than Taylor-extrapolated") is what was
built; it simply lives in its own small file next to the reference models
instead of inside their structs, because the group is shared across both
channels. Splitting `T*` into `guidance_v_ref` and `phi*`/`theta*` into
`guidance_h_ref` would have scattered one coherent quadruple across two modules
and forced the shared sensitivity block to read from both.

The plan's stated goal — "one path, one place where flat data enters guidance"
— is preserved: the data still enters only through `nav_flat_traj_run()`, on the
same tick, next to the existing `set_flat` calls. Only the *propagation between
samples* differs, because the quantity is of a different kind.

## Why not the alternative (nav_flat_traj → guidance_heol directly)

That would bind the trajectory player to one specific guidance variant. The
latch is variant-agnostic: it lives in `guidance_rotorcraft`, so the planar MIMO
stage, or any future consumer, reads the same values from the same place.

## Zero-order hold, and what it costs

Samples arrive at `NAVIGATION_FREQUENCY` (20 Hz); guidance runs at
`PERIODIC_FREQUENCY` (500 Hz). A held value is therefore up to 50 ms stale.

Over the demo trajectory `T*` spans about 1.4 N, so the stair steps are of order
0.05 N. The error the exact `T*` was introduced to remove is ~19 % of |T| at
peak tilt, about 1.2 N — roughly 25x larger. The hold is comfortably good
enough.

Linear interpolation was rejected because it needs the *next* sample, which a
push API does not have. First-order extrapolation was rejected because it adds
lead and amplifies sample noise for no useful gain on a quantity that is not a
derivative.

## Staleness instead of a mode flag

The latch is timestamped and `guidance_flat_nominal_get()` reports invalid after
`GUIDANCE_FLAT_NOMINAL_TIMEOUT` (0.2 s, four nav ticks). Consumers fall back to
their non-flat behaviour automatically.

This is what makes leaving the trajectory safe with no bookkeeping. When the
flight plan hands off from `Flat_Traj_Demo` to `Standby`, the latch goes stale
on its own and `guidance_heol_vert()` reverts to the old `m*(zdd_ref - g)`
feedforward, which is the correct thing for an ordinary altitude hold. While a
flat block holds at its endpoint it keeps re-sending the final sample (see
`nav_flat_traj_run()`), so the latch stays fresh at the endpoint's hover values
— also correct.

## Related

- `heol_input_sensitivity.h` — the shared alpha block that consumes this
  quadruple plus `psi*` from `guidance_h.sp.heading`.
- `Knowledge/Sessions/2026-08-14-heol-flat-nominal-inputs.md` — the session that
  built it, including the SITL evidence and the unresolved `alpha_z` retune.
