# 2026-08-27 — `plotjuggler_flatness.xml`: one layout for FINDI and FMFC

Branch `plotjuggler-flatness-20260827`, worktree
`.claude/worktrees/plotjuggler-flatness`, based on `mfc-development` at outer
`fe795f0` (submodule pinned `71c995b0e`).

**Scope: one new file at the repo root.** No firmware source, no existing
layout, and `pj_json_relay.py` were touched.

## What was built

`plotjuggler_flatness.xml`, in the house style of `plotjuggler_heol.xml`: a long
leading header that carries the reading conventions, then 9 tabs / 30 plots /
116 curves. Every plot holds **both** the `FINDI/` and the `FMFC/` series for
the same quantity, so one file covers both controllers and the unused prefix
draws empty.

| Tab | Plots |
|---|---|
| Mode and health | guidance, nom_valid, alloc_ok, flat_status (4) |
| Linear: accel [m/s^2] | a_tilde vs a_c, per axis (3) |
| Linear: force [N] | fi_prev / fi_star / fi_c per axis, then flat/f (4) |
| Attitude error [rad] | zeta_e x,y,z both prefixes (1) |
| Angular: rate and accel | w_ref [rad/s]; dw_lpf [rad/s^2]; dw_ref_y + dw_c_y [rad/s^2] (3) |
| Moments [N*m] | m_prev / m_star / m_c per axis (3) |
| Estimator: force bracket (FMFC) | F_fi [m/s^2], du_fi [N], eps_fi [m], den_fi (4) |
| Estimator: moment bracket (FMFC) | F_m [rad/s^2], du_m [N*m], eps_m [rad], den_m (4) |
| Allocation | v_0..2 [N*m], v_3 [N], u_* [pprz], act a_* [pprz] (4) |

Two deviations from the proposed tab list, both forced by the one-unit-per-plot
rule the file exists to obey:

1. **The estimators take two tabs, not one.** `F_fi` is [m/s^2] and `F_m` is
   [rad/s^2]; `du_fi` is [N] and `du_m` is [N*m]; `eps_fi` is a position error
   in [m] and `eps_m` a filtered attitude error in [rad]. Grouping by quantity
   (`F_*` on one plot) would have put two units on four axes. Grouping by
   bracket keeps every plot single-unit, at the cost of one extra tab.
2. **`alloc/v` splits across two plots.** `fmfc_v` / `findi_v` is
   `[Mx, My, Mz, Fz]` — three moments in [N*m] and one force in [N] in the same
   array. Drawing them together reproduces exactly the squashing that made
   `plotjuggler_heol.xml` necessary in the first place.

## Source inventory: no discrepancies

The `NPS_SCOPE_VAR` / `NPS_SCOPE_VARN` blocks in `oneloop_findi.c` (L204..243)
and `oneloop_fmfc.c` (L262..319) match the dispatched list exactly. Counts:
`FINDI_OUTPUTS = FINDI_NUM_ACT = 4`, `FMFC_OUTPUTS = FMFC_NUM_ACT = 4`,
`FMFC_BRACKET_N = 3`. 45 FINDI registrations, 71 FMFC, 116 total. Every one of
them is plotted exactly once and the layout contains no name that is not
registered (checked programmatically against the capture headers).

## Facts dug out of the source that the header now carries

- **`mode/flat_status` is an enum, and 0 is the good value** — the opposite
  sense to the three booleans beside it. `enum FlatnessQuadStatus`
  (`flatness_quad.h:105`): 0 OK, 1 FREE_FALL, 2 GIMBAL_LOCK,
  3 BRANCH_DEGENERATE. On a degraded call the attitude fields are the previous
  call's, held, while `f` is still freshly computed — so a nonzero status means
  the attitude command went stale without going flat.
- **`eps_fi` is not a force residual.** `oneloop_fmfc.c:709` passes `e_p` — the
  raw position error in [m] — as the force bracket's estimator drive.
  `eps_m` (`oneloop_fmfc.c:758`) is `-zeta_e_lowpass[i].o[0]`, the negated
  low-pass filtered attitude error in [rad].
- **`F_*` lives on the far side of alpha**, so `F_fi` is [m/s^2] and `F_m` is
  [rad/s^2], not forces and moments. `alpha_fi = (1/m) I`, `alpha_m = J^-1`.
- **`alloc/act` is the modelled actuator state**, not a measurement: `act_obs`
  plus the first-order lag driven by `u` (`fmfc_get_actuator_state`,
  `oneloop_fmfc.c:535`). It is what the applied wrench is computed from, which
  is why it gets its own plot next to `u`.
- **`dw_c` is not the same signal on the two prefixes.** FINDI:
  `dw_c = k_rate (w_c - w) + dw_ref`. FMFC: no `dw_ref` term, the feedforward
  entering only through `m* = I dw_ref`. They share a plot and must not be read
  as comparable.

## Verification

**SITL, both aircraft, names read out of the capture** — the check that matters
here, since a typo'd series name is the whole failure mode:

```
timeout 60 ./sim.sh ANTON_FMFC  --no-build --nav "Start Engine,Takeoff,+15,Standby"
    -> sim_logs/mfc_sim_20260827_190032.csv   28448 rows, 71 /uav/FMFC/* columns
timeout 60 ./sim.sh ANTON_FINDI --no-build --nav "Start Engine,Takeoff,+15,Standby"
    -> sim_logs/mfc_sim_20260827_190146.csv               45 /uav/FINDI/* columns
```

Both exited 124 (expected). Set-compared against the layout: zero curves in the
layout absent from the captures, zero registered series left unplotted, zero
duplicates. Each capture carries **only** its own prefix, which is the
structural confirmation that the other controller's curves have no series at
all rather than a flat-zero one.

XML parses clean; the double-hyphen sweep over comment bodies reports zero.

**PlotJuggler itself could not be run.** It is not installed in this sandbox and
is not installable from apt or pip here, so the layout has never been opened.
The "does PlotJuggler ignore curves whose series is absent" question is
therefore **not empirically confirmed** by this session. The in-repo precedent
is that `plotjuggler_heol.xml` already ships SITL-only `HEOL_NOMINAL/*` and
`HEOL_ALPHA/*` curves in a layout its own header says is read off flight logs
too. The header records the caveat and names the fallback (two files sharing
the one header) in case PlotJuggler complains rather than ignores.

## Tooling notes

- `./pprz.sh build <AC> nps` prints `==> Done: .../nps/obj/nps.elf`, a path that
  **never exists**. The nps artifact is `var/aircrafts/<AC>/nps/simsitl` and
  there is no `obj/` subdirectory under the nps target. Logged as `bug-300`;
  `bug-299` (`pprz.sh db` wipes the nps binary) refers to the same artifact.
- `ANTON_FMFC`'s nps binary had to be rebuilt after `bug-299`; `ANTON_FINDI`'s
  survived. `WORKSPACE_DIR="$PWD"` was set on every `pprz.sh` / `sim.sh` call.

## Not done, deliberately

- No signal was added to either module. Nothing genuinely useful looked missing
  for a structural read; the closest candidates are `dw_c_x` / `dw_c_z` and
  `w_ref_z`, whose absence is documented in the header as intentional rather
  than reported as a gap. If a future session wants the full angular picture
  those three registrations are the ask, and they are a firmware change.
