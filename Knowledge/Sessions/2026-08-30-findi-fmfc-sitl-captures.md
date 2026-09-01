---
date: 2026-08-30
topic: FINDI / FMFC SITL capture campaign
tags: [session, sitl, contract, flat-traj]
---

# FINDI / FMFC SITL captures on the flat trajectories

## Why

The Simulink leg of `Data/2026-08-30_quad_controller_comparison_v3` compares
`Flat_INDI_Quad` vs `Flat_MFC_Quad`. The only SITL capture that existed was on
`ANTON_MFC`, so the cross-source comparison was between different controllers.
This session produced the matching SITL leg.

## Build

Both `nps` targets build clean:

```
CONF="conf/userconf/ENAC/conf_mfc.xml" ./pprz.sh build ANTON_FINDI nps
CONF="conf/userconf/ENAC/conf_mfc.xml" ./pprz.sh build ANTON_FMFC  nps
```

`pprz.sh` defaults `CONF` to `conf_enac.xml`, which does not list either
aircraft — the `CONF` override is required on that path. `sim.sh` already
defaults to `conf_mfc.xml`.

## Captures

Eight runs, `timeout 55 ./sim.sh <AC> --no-build --nav "Start Engine,Takeoff,+15,<block>"`.

| airframe | trajectory | ref span n/e/z [m] | truth span x/y/z [m] | peak &#124;e&#124; [m] | rms [m] | verdict |
|---|---|---|---|---|---|---|
| ANTON_FINDI | minsnap      | 1.00 / 1.00 / 1.00 | 1.11 / 1.12 / 1.00 | 0.173 | 0.100 | tracks |
| ANTON_FINDI | circle4      | 3.00 / 3.00 / 0.00 | 1.73 / 2.01 / 0.58 | 1.406 | 0.829 | does not track |
| ANTON_FINDI | loop_upright | 0.00 / 2.00 / 2.00 | 0.07 / 1.83 / 2.01 | 1.029 | 0.747 | degraded |
| ANTON_FINDI | loop_roll    | 0.00 / 2.00 / 2.00 | 0.33 / 5.45 / 7.14 | 6.380 | 4.149 | **diverges** |
| ANTON_FMFC  | minsnap      | 1.00 / 1.00 / 1.00 | 1.10 / 1.07 / 1.02 | 0.157 | 0.096 | tracks |
| ANTON_FMFC  | circle4      | 3.00 / 3.00 / 0.00 | 1.74 / 2.53 / 0.19 | 1.508 | 0.839 | does not track |
| ANTON_FMFC  | loop_upright | 0.00 / 2.00 / 2.00 | 0.06 / 1.47 / 1.86 | 0.582 | 0.285 | degraded |
| ANTON_FMFC  | loop_roll    | 0.00 / 2.00 / 2.00 | 0.05 / 2.42 / 1.67 | 1.238 | 0.664 | degraded |

Window is the manoeuvre plus 2 s of settle, isolated from the last contiguous
run of horizontal motion in `SP/guidance/h_ref_{n,e}`.

Two findings worth carrying:

1. **circle4 fails on both airframes in the same way** — a ~45 % span undershoot
   against a 3 m reference. That makes it a trajectory/guidance property, not a
   controller difference, and it reproduces the earlier `ANTON_MFC` observation.
2. **loop_roll separates the two controllers cleanly.** `ANTON_FINDI` departs to
   a 5.45 m lateral / 7.14 m vertical excursion (`TRUTH/phi` to -2.11 rad) and
   takes ~7 s to return to the hold point; `ANTON_FMFC` bounds the identical
   manoeuvre at 1.238 m. This is the sharpest controller contrast in the set.

## Contract

`contract/signals.json` v4 -> v5, three copies synced,
md5 `d5304a0a873cc29b23e42b0be49772fc`. Two new branches,
`SITL_6DOF_ANTON_FINDI` and `SITL_6DOF_ANTON_FMFC`, structural twins of
`SITL_6DOF_ANTON_MFC` (no `message` key, no `err`, fully-qualified bindings).
The only binding that differs is `u`: `/uav/FLAT/alloc/u/u_0..3`, registered by
`oneloop_findi.c:271` and `oneloop_fmfc.c:345`, verified against real capture
headers rather than taken on trust.

New hazard recorded in `u_is_airframe_dependent_and_in_pprz_units`: those
columns sit at **-9600 before the motors arm** (first ~2 s, 3.8 % of samples)
and at ~1570-2400 pprz units in flight. A `u_rms` or `max_abs_u` taken over a
whole capture is poisoned by the sentinel.

## Ingest

Eight stems under `Data/2026-08-30_quad_controller_comparison_v3/data/`:
`sitl_{findi,fmfc}_{minsnap,circle4,loop_upright,loop_roll}`, giving run ids
`tracking_6dof_sitl_findi_minsnap` etc. that pair by suffix with the Simulink
`tracking_6dof_{findi,fmfc}_quad_<traj>`. Each sidecar carries
`tracking_quality` (`tracks` / `degraded` / `diverged`) plus the measured spans
and errors, so the runs that failed stay legible as evidence.

`mfcdata check`: 0 errors, 0 warnings. All 19 contract columns resolve against
all 8 captures. Every payload is git-ignored by the `*/data/` rule in
`Data/.gitignore`.

## Pasteable MATLAB overlay (NOT executed here — MATLAB is on the author's host)

`qsim.compare` and `qsim.plot_run` take **stems** and resolve them through
`qsim.load_run`, which reads `<stem>.mat`. A CSV-ingested SITL run has no
`.mat` and no `scenario`, so it cannot go through those two. The overlay has to
call the recipe directly. Two further things the block has to do itself:

- **Shift the time base.** `qsim.import_csv` folds `t0_offset_s` into `.meta`
  but does not apply it to `log.t`. Left alone, the SITL traces sit at t = 23 s
  and the Simulink ones at t = 0.
- **Read the `u` panel with care.** Firmware `u` is pprz actuator units
  (~1570-2400 in flight, -9600 before arming); Simulink `alloc_u` is normalised
  [0,1]. The control panel overlays two different physical quantities.

```matlab
%% SITL vs Simulink -- minsnap, FINDI and FMFC
% Run from MFC-Workspace/Generic_Quad (qsim.config().root, and where
% recipes/tracking_6dof.m lives).

E = qsim.open_effort("../../Data/2026-08-30_quad_controller_comparison_v3");

% --- Simulink leg: saved .mat runs, addressed by stem --------------------
runs    = qsim.load_run(E, "tracking_6dof_findi_quad_minsnap");
runs(2) = qsim.load_run(E, "tracking_6dof_fmfc_quad_minsnap");
T_sim   = runs(1).log.t(end) - runs(1).log.t(1);   % 9 s on this effort

% --- SITL leg: wide CSVs, bound to roles through the contract ------------
% label defaults to the file stem, so import_csv builds
%   run_id = "tracking_6dof" + "_" + "sitl_findi_minsnap"
%          = tracking_6dof_sitl_findi_minsnap
sitl_stems = ["sitl_findi_minsnap",       "sitl_fmfc_minsnap"];
branches   = ["SITL_6DOF_ANTON_FINDI",    "SITL_6DOF_ANTON_FMFC"];

for k = 1:numel(sitl_stems)
    r = qsim.import_csv(fullfile(E.data, sitl_stems(k) + ".csv"), ...
                        'source',    "sitl", ...
                        'branch',    branches(k), ...
                        'data_type', "tracking_6dof");

    % Put the manoeuvre at t = 0 and clip to the Simulink horizon.
    t0   = r.meta.t0_offset_s;                       % from <stem>.meta.json
    keep = r.log.t >= t0 & r.log.t <= t0 + T_sim;
    f    = fieldnames(r.log);
    for j = 1:numel(f)
        v = r.log.(f{j});
        if size(v,1) == numel(keep), r.log.(f{j}) = v(keep,:); end
    end
    r.log.t = r.log.t - t0;

    runs(end+1) = orderfields(r, runs(1)); %#ok<SAGROW>
end

% --- One overlay, drawn by the runs' own recipe --------------------------
[~, recipe] = qsim.recipe_for("tracking_6dof");
figs = qsim.render(recipe, struct('runs', {runs}));

files = strings(0,1);
for k = 1:numel(figs)
    files(end+1,1) = qsim.export_fig(figs(k).fig, E, ...
        "compare_sitl_vs_simulink_minsnap", figs(k).name); %#ok<SAGROW>
end
disp(files)
```

Swap `minsnap` for `circle4` / `loop_upright` / `loop_roll` to get the other
three pairs; `loop_roll` is the one worth looking at first.
