# 2026-08-22 — HEOL rung-5 re-run: the horizontal loop is a saturated limit cycle

Scope: `ANTON_HEOL`, SITL only. Purpose was to close the HEOL horizontal
divergence diagnosis with a deliberate rung-5 re-run on a re-baselined tree,
after `cef3454d3` (trajectory swap + gain/filter changes) and `a6051d547`
(attitude retune) invalidated every recorded number.

**Verdict: the task does NOT close.** The horizontal channel spends 70–84 % of
the trajectory block on the ±20° bank rail and never recovers, with every NPS
noise source zeroed. Details below.

## What was run

Seven SITL runs, all on the flight plan
`Start Engine,Takeoff,+3,Standby,+10,Flat_Traj_Demo,+40,Standby`, submodule at
`f0d48a013` plus this session's `anton_heol.xml` edits.

| run | log | HXY / GZ deriv filter | NOISE_SCALE | note |
|---|---|---|---|---|
| A  | `mfc_sim_20260822_002715.csv` | 20 / 20 | stock | tree as found |
| A' | `mfc_sim_20260822_003043.csv` | 20 / 20 | 1. | with the new noise plumbing |
| B  | `mfc_sim_20260822_003244.csv` | 0 / 0   | 1. | |
| C  | `mfc_sim_20260822_003625.csv` | 0 / 20  | 1. | |
| C' | `mfc_sim_20260822_003832.csv` | 0 / 20  | 1. | **control**: engagement +1 s |
| D  | `mfc_sim_20260822_004348.csv` | 0 / 20  | 0. | |
| E  | `mfc_sim_20260822_004831.csv` | 0 / 0   | 0. | |

Scored with `tools/heol_rung5_metrics.py` (added this session) on **fixed
absolute windows** so every run is graded on the same clock — Standby hold
12–21.5 s, flat-traj block 22–62 s, return-to-Standby 63–76 s, late hold
76–end. Using a window derived from each run's own reference motion, as I did
first, silently included the 3 m→2 m altitude step in the vertical figure and
inflated `err_z` by 3× in one run. Fixed windows or nothing.

**A and A' are identical to three decimals.** The sim is deterministic run to
run, which is what makes any of the comparisons below meaningful at all.

## 1. The horizontal loop is small-signal stable and large-signal unstable

This is the finding worth keeping, and it reframes the whole diagnosis.

The new `Flat_Traj_Demo` trajectory is **not a trajectory** in any meaningful
sense at this scale: `flat_traj_demo.xml` says its full extent is ~1.5 m, and
the log confirms it — the position reference ramps from (0,0) to (1.00, 1.00) m
over about 1.5 s and then **sits there, exactly constant, for the remaining
38 s**. `sp_traj_x`/`phi_star` (the flatness feedforward) returns to zero by
t ≈ 24 s. So from t ≈ 24 s onward the horizontal channel is holding a
stationary 1 m-offset point, with no feedforward.

It cannot do it. Run B, with noise (run E, noise off, in brackets):

| phase | err RMS x / y [m] | err RMS z [m] | tilt cmd railed φ / θ |
|---|---|---|---|
| Standby hold 12–21.5 s | 0.158 / 0.263 [0.021 / 0.016] | 0.0057 [0.0059] | **0.0 / 0.0 %** |
| flat-traj block 22–62 s | 11.71 / 10.24 [10.38 / 11.44] | 0.0083 [0.0090] | **78.4 / 79.1 %** [72.9 / 73.3] |
| Standby 63–76 s | 2.09 / 8.76 [5.53 / 2.34] | 0.0378 [0.0357] | 74.5 / 59.3 % [63.7 / 76.3] |
| late hold 76–91 s | 1.27 / 0.40 [1.58 / 1.15] | 0.0023 [0.0020] | 1.5 / 47.5 % [44.0 / 66.6] |

Peak position error 24.2 m [24.4 m]. Growth timing, run C: `|err_x|` passes
1 m at t = 30.3 s, 5 m at 32.9 s, 10 m at 38.2 s, first bank rail at 29.1 s.

Two things follow:

1. **Before engagement the loop is quiet.** 0.0 % rail, error RMS 0.16/0.26 m
   with noise and 0.02 m without. Nothing is structurally broken at small
   amplitude.
2. **After the ~1 m step it never recovers, even back in Standby.** The
   return-to-Standby and late-hold phases still rail 44–76 % of their samples
   and hold a ±2 m limit cycle out to the end of the log. Deroute back to a
   plain waypoint hold does not reset it.

So this is not "trajectory tracking is poor" and it is not the guided-mode
setpoint path — the same block that holds fine at (0,0) fails at (1,1) once the
command has been driven into saturation. It is a **saturation-induced bang-bang
limit cycle**: a 1 m reference step drives the tilt command onto the ±0.3491 rad
clamp, and the loop cannot come off it.

`F_hat` **does not run away** any more — the original signature was −143 by
t = 15 s; here `fk_x`/`fk_y` stay inside roughly ±24 m/s² and, in the late hold,
inside ±2.2 m/s². It is bounded and it oscillates with the limit cycle rather
than driving it. That part of the earlier diagnosis is genuinely fixed. The
loop still does not fly.

## 2. The vertical channel is healthy and is the control

`err_z` RMS is 0.005–0.009 m through the flat-traj block and 0.002–0.004 m in
the late hold, in **every** run — filter on or off, noise on or off. The 0.036 m
in the 63–76 s window is the real 3 m → 2 m altitude change when Standby
resumes, not an error. `cef3454d3` retuned this channel (GZ_KP 4→16, GZ_KD
6→12) and it survived the retune intact.

## 3. Noise pair — the elimination still holds

Final configuration (both deriv filters 0), noise on vs all NPS noise sources
zeroed: flat-traj block 11.71/10.24 m vs 10.38/11.44 m, railed 78.4/79.1 % vs
72.9/73.3 %. Qualitatively identical; the numeric differences are inside the
run-to-run spread established in §4.

**The divergence reproduces with perfect sensors.** It is structural. No
noise-side, filter-cutoff or estimator-window work will fix it.

The airframe previously had no noise plumbing at all. Added
`NPS_SENSORS_PARAMS = nps_sensors_params_anton_mfc.h` and `NPS_NOISE_SCALE = 1.`
to `anton_heol.xml`, the same contract `anton_mfc.xml` already used.
`NOISE_SCALE = 1.` is bit-identical to the stock implicit configuration —
verified, not assumed: A (no plumbing) and A' (plumbing, scale 1) agree to three
decimals on every metric.

## 4. The derivative-filter A/B could not discriminate — and why that matters

`cef3454d3` set `HXY_DERIV_FILTER` and `GZ_DERIV_FILTER` 0 → 20 while both
`*_USE_MEASURED_VEL` stayed false, which the airframe's own comment block
forbids ("Enable the pair, or neither"). Confirmed at source that the filter is
live regardless: `mfc_deriv_filter_step()` is applied unconditionally in
`mfc_core.c:200-204` and `mfc_core_mimo.c:249-251`; `use_external_derivative`
selects *which* derivative feeds the PD, not whether it is filtered, and the
filter bypasses only for `N <= 0`. There is no interlock — only the comment.

Flat-traj block 22–62 s:

| config | traj RMS x / y [m] | railed φ / θ |
|---|---|---|
| HXY 20, GZ 20 | 11.27 / 11.18 | 84.2 / 79.4 % |
| HXY 0, GZ 0 | 11.71 / 10.24 | 78.4 / 79.1 % |
| HXY 0, GZ 20 | 11.55 / 10.32 | 76.0 / 76.2 % |
| **HXY 0, GZ 20, engagement +1 s** | **10.89 / 9.74** | **70.6 / 70.3 %** |

The last row is the control and it is the point. Nothing changed but the
engagement time, by one second — and the rail figure moved 5.4 points, as much
as any filter setting moved it. **The loop is in a chaotic saturated limit cycle
in every variant, so RMS and rail percentage are not measuring the filter.**

Reverted both to 0 — on the pair rule, explicitly **not** on a measurement. The
airframe file and its comment block now agree, and the comment records that the
A/B was inconclusive rather than pretending it settled anything.

Also: the "late hold 0.17/0.32 m → 33.3/17.1 m" figure the comment block carried
for the filter-on case **was not reproduced**. The current tree does not reach a
0.17 m late hold in any configuration tested here. That number is retired.

`GZ_KP`/`GZ_KD` = 16/12 (wn=4, ζ=1.5) vs the sim's 4/6 (wn=2): left at 16/12,
because the vertical measures excellently there, and flagged in the comment as
an open decision for the author. Firmware and Simulink are not at parity on z
and now the file says so.

## Traps hit this session

- **`WORKSPACE_DIR` is the launch root, not the repo root.** `pprz_docker.sh`
  bind-mounts `${WORKSPACE_DIR:-<repo root>}` to `/workspace`. Launched from the
  vault root, `WORKSPACE_DIR` is the *vault*, so `/workspace/pprz.sh` does not
  exist and every dispatch fails with `stat ./pprz.sh: no such file or
  directory`. Fix: `WORKSPACE_DIR=<repo root> ./pprz.sh …`. Nothing in the repo
  is wrong; the assumption that the sandbox is rooted here is.
- **A leftover sim container silently corrupts the next run's CSV.**
  `sim.sh` runs `--network host`, so two live sim containers share one IVY bus
  and one scope port, and the CSV writer merges both streams. One run's log
  (`…_004035.csv`, quarantined as `.CONTAMINATED`) had 3520 backwards time
  steps from exactly this. `timeout` kills the `docker run` client, not always
  the container. **Run `docker ps -q | xargs -r docker kill` before and after
  every sim, and check time monotonicity before trusting any log** —
  `heol_rung5_metrics.py` now prints a backstep count and flags it.
  A second worker was operating in this submodule concurrently (`darko_findi`,
  untracked), which makes the shared-bus hazard a live one, not theoretical.
- **A run-relative analysis window is a fabricated metric.** Deriving the
  scoring window from each run's own reference motion made a 3× vertical
  difference appear out of nothing — it was the Standby altitude step drifting
  in and out of the window. Fixed absolute windows.

## Where this leaves the ladder

Rung 5 (horizontal guidance with attitude closed) **fails**. Rung 4 (vertical)
passes cleanly and is not in question. The next question is not a gain and not a
filter — it is why a 1 m step saturates the tilt command at all, and why the
loop cannot come off the clamp once it is there. Candidates, in the order I
would take them:

1. **Clamp/anti-windup structure.** `heol_hxy.clamp_mode` and the estimator's
   pre/post-saturation tap decide what the estimator sees while the command is
   railed. `est_use_presat_command` was the last root cause found here; the
   clamp interaction is the same family of defect and has not been examined
   with the loop actually saturated.
2. **The step itself.** `heol_mimo_init()` hardcodes `use_trajec_sp = false`, so
   the reference enters as `epsilon` with no smoother — the exact difference
   from `guidance_mfc.c` written up on 2026-08-19. A 1 m step into an unfiltered
   PD is the trigger; that path is still unfixed.
3. Only then gains.

Do not re-litigate `HXY_INTEGRATION_WINDOW = 500`; it is deliberate.

## Task-board consequence

`heol-xy-divergence-diagnosis` stays **open**. `heol-firmware-mimo-horizontal`
stays **blocked**; `heol-firmware-attitude-channels` and `tune-heol-quad-sitl`
stay **gated**. The closing criterion was explicit that improved tracking with
the loop still railed is not a close, and the loop is railed 70–84 % of the
trajectory block.
