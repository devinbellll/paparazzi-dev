# 2026-08-14 — HEOL guidance: feedforward enable, unit fix, gain reconciliation

Three independent fixes to the ANTON_HEOL guidance variant, none touching control
architecture. All three are confirmed correct/wired-up (build + short SITL
check below); X/Y long-horizon stability with the reconciled gains is **not**
yet verified and is flagged as open — see "Update" section near the bottom.

## 1. Flat-trajectory feedforward turned on

`sw/airborne/modules/nav/flat_traj_demo_data.h`: `FLAT_TRAJ_DEMO_ORDER` was `0`,
which made `guidance_h_set_flat()` / `guidance_v_set_flat()` mask every
derivative above position to zero — despite the 751-sample table already being
fully populated with real min-snap data through snap and heading-snap. Bumped
to `4`; rewrote the stale "TODO(user): all-zero placeholder" comment block to
describe the actual populated table.

Checked before the change (per the task brief, reconfirmed here): at t=0,
position/velocity/acceleration/jerk/heading/heading-rate are all zero — only
snap and heading-accel are nonzero, which is normal for a min-snap polynomial
at rest, not a discontinuity. No t=0 ramping was added.

## 2. Horizontal clamp unit mismatch

`guidance_heol.c` set `heol_gx.mfc.u_max/u_min` (and `heol_gy`) to
`9.81f * GUIDANCE_HEOL_MASS * sinf(GUIDANCE_H_MAX_BANK) * 0.7f` — a force in
newtons. But the gx/gy command is an acceleration in m/s² (see
`guidance_heol_horiz()`: "command unit is already m/s^2"). Dropped the `MASS`
factor from both `u_max`/`u_min` on gx and gy — the clamp was off by ~0.8x
(ANTON's hover mass) before this fix. gz's clamps are untouched — that channel
really is in newtons (`GZ_MAX_THRUST` etc.).

## 3. Gain convention reconciled (canonical: raw-coefficient form)

`mfc_core.c` implements `fb = kd*edot + kp*e`, i.e. the raw coefficients of
`s^2 + kd*s + kp` — not the Simulink reference's second-order form
(`P = wn^2`, `D = 2*zeta*wn`). Decision: raw-coefficient form is canonical
(no conversion in firmware; conversion lives in Simulink/MATLAB). Re-derived
`anton_heol.xml`'s `GUIDANCE_HEOL` section from the Simulink loops:

| axis | Simulink (wn, zeta) | kp = wn² | kd = 2·zeta·wn | old kp/kd (wrong convention/stale) |
|------|---------------------|----------|----------------|-------------------------------------|
| x/y  | wn=2, zeta=0.7      | 4        | 2.8            | 2 / 25 |
| z    | wn=2, zeta=1.5      | 4        | 6              | 16 / 12 |

`*_KI = 0.` left untouched (matches Simulink's PD-with-no-integrator).
Integration windows also differed (600 vs Simulink's 500 on x/y, 4 vs 5 on z)
— ported straight from Simulink (500 / 500 / 5), since `int_window` is an
estimator parameter, not a gain, so the convention question doesn't apply.
Added a permanent comment in the airframe XML stating the convention and the
derivation so it can't silently drift back to the second-order form.

## Verification (SITL, ANTON_HEOL, Flat_Traj_Demo block)

Ran `./pprz.sh build ANTON_HEOL ap` and `nps` — both link clean with all three
changes in place (pre-existing `PERIOD_STAB_ATTITUDE_FlightRecorder_0`
redefinition warnings are unrelated/pre-existing).

Flew `./sim.sh ANTON_HEOL --nav "Start Engine,Takeoff,+3,Standby,+1,Flat_Traj_Demo,+2,Standby"`
twice — once with `FLAT_TRAJ_DEMO_ORDER=4` (post-change), once temporarily
reverted to `0` (pre-change) for a same-conditions comparison — and inspected
the GUIDANCE_MFC telemetry (`/uav/MFC_GUIDANCE/sp_traj_*` = `u_ff`, confirmed
via `send_guidance_heol()` in `guidance_heol.c`):

- **Pre-change (ORDER=0):** `sp_traj_x`/`sp_traj_y` identically 0 throughout;
  `sp_traj_z` pinned at `-10.9855` (= `-9.81*MASS`, the nominal hover force)
  for the entire trajectory window — exactly the old constant/zero behaviour.
- **Post-change (ORDER=4):** `sp_traj_x`/`sp_traj_y` become nonzero and track
  the trajectory's acceleration profile (small early swing, decaying back
  toward 0 as the horizontal motion settles); `sp_traj_z` moves smoothly away
  from the hover value and back, following the trajectory's vertical
  acceleration lobe — i.e. `u_ff` now tracks `ref.accel`/`zdd_ref` instead of
  sitting at the old zero/constant value. Confirms the fix.

  Aside, not a regression: both runs show `sp_traj_z` briefly stepping to
  `-4.7105` around the point the flight plan hands off from `Flat_Traj_Demo`
  to the following `Standby` block. Since it reproduces identically with
  `ORDER=0` (no feedforward at all), it's pre-existing behaviour of
  `guidance_heol_vert()` being called unconditionally for `guidance_v_run_pos`
  (Standby's altitude hold) as well as the flat-trajectory path, not something
  introduced by this change — out of scope here, worth a follow-up look if
  MIMO work touches the vertical reference model.

### Tracking-error baseline

Position tracking error (`/uav/MFC_GUIDANCE/err_x/y/z`, the residual fed into
each axis's decoupled MFC feedback loop) over the trajectory-active window
(1.5 s, ORDER=4 run):

| axis | RMS [m] | max [m] |
|------|---------|---------|
| x    | 0.0072  | 0.0112  |
| y    | 0.0086  | 0.0129  |
| z    | 0.0603  | 0.0949  |

For reference, the same window with `ORDER=0` (no feedforward) gave nearly
identical numbers (x: 0.0072/0.0112, y: 0.0086/0.0128, z: 0.0603/0.0949) —
expected in SITL, since HEOL's decoupled `u_fb` loop is designed to null the
residual regardless of `u_ff`; the feedforward's benefit is reduced `u_fb`
effort and faster/less-lagged tracking, not necessarily a smaller final
position error against an ideal-actuator sim model.

**Caveat, added after user follow-up (see below): this table is NOT a clean
stability baseline.** It only covers the scripted run's 1.5 s trajectory
window immediately after trigger, which is too short to show what happens
next — a longer/interactive run reveals large X/Y oscillations that this
window doesn't capture at all. Treat these RMS numbers as "the feedforward
path is wired up and doing something sane in the first 1.5 s," not as
evidence the retuned loop is stable.

## Update — large X/Y oscillations observed in interactive SITL

Running the same aircraft/block interactively (GCS/PlotJuggler attached,
longer duration than the scripted 1.5 s window above) shows the horizontal
position (`TRUTH/x`, `TRUTH/y` vs. `SP/guidance/h_*`) tracking the trajectory
initially, then diverging into sustained ~30 m oscillations rather than
settling. The short automated check above ran for exactly one 1.5 s
trajectory pass and immediately handed off to `Standby`, so it never ran long
enough to expose this — the low tracking-error numbers recorded above are
real for that narrow window, but don't say anything about longer-horizon
stability, and should not be read as "the retuned gains are stable."

Likely contributors, not yet investigated: the re-derived x/y gains
(`kp=4, kd=2.8`, i.e. wn=2, zeta=0.7) are considerably less damped in the
raw-coefficient sense than the previous ad hoc `kp=2, kd=25`, so the u_fb
loop's own dynamics plus the outer INDI attitude loop plus the WLS allocator
stacked together may not be stable at this operating point even though each
piece was reconciled individually. Per team direction, this is **not** being
chased right now — the priority is finishing the surrounding
infrastructure (this session's three fixes, telemetry, data tooling) before
doing a real stability/performance pass on HEOL's X/Y guidance. Flagging it
here as the known-open item for whoever does that pass next; robust XY
performance has been an ongoing, nontrivial problem for this guidance
variant independent of these fixes.

## Files changed

- `paparazzi/sw/airborne/modules/nav/flat_traj_demo_data.h` — ORDER 0→4, comment rewrite
- `paparazzi/sw/airborne/firmwares/rotorcraft/guidance/guidance_heol.c` — gx/gy clamp unit fix
- `paparazzi/conf/airframes/ENAC/quadrotor/anton_heol.xml` — GUIDANCE_HEOL gains re-derived, convention comment added
