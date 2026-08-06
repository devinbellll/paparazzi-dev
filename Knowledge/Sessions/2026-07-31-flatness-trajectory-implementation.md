# Session — 2026-07-31: Flatness Trajectory Setpoints implementation

Implemented [[Flatness Trajectory Setpoints (Pos-Vel-Accel-Jerk-Snap + Psi)]] on a new
branch `feature-diff-flatness` in `paparazzi/` (created from `feature-mfc-SI-units`, the
branch that was checked out — NOT a worktree, per instruction).

## What changed

- `sw/ext/pprzlink/message_definitions/v1.0/messages.xml` (pprzlink submodule, branch
  `mfc-scope-parity`, uncommitted): new `GUIDED_TRAJECTORY_NED` message, id **42** (datalink
  class; free ids 42–49 confirmed by scanning the class). Carries pos/vel/accel/jerk/snap +
  heading through heading-snap + `order` (uint8, 0=pos..4=snap) + `ac_id`.
- `guidance_h.h`/`.c`: added `jerk`/`snap` (`FloatVect2`) and `heading_accel/jerk/snap` to
  the setpoint/reference structs, new `h_mask` value `GUIDANCE_H_SP_FLAT`, new
  `guidance_h_set_flat()`. Heading integration in `guidance_h_update_reference()` extended
  from single-term rate to full quartic Taylor (defaults to old behavior when
  accel/jerk/snap are zero).
- `guidance_h_ref.h`/`.c`: `jerk`/`snap` added to `struct GuidanceHRef`; new
  `gh_set_flat_ref()` (resets/syncs state, zeroing fields beyond `order`) and
  `gh_update_ref_from_flat_ref()` (quartic Taylor extrapolation every tick — this is what
  replaces the zero-order-hold staircase bug for the flat setpoint path).
- `guidance_v.h`/`.c`, `guidance_v_ref.h`/`.c`: vertical mirror. New guided-mode
  `GUIDANCE_V_GUIDED_MODE_FLAT`, `guidance_v_set_flat()`, `gv_set_flat_ref()`,
  `gv_update_ref_from_flat_ref()`. Vertical jerk/snap (`j_sp/s_sp`, `gv_j_ref/gv_s_ref`) are
  plain floats — no new fixed-point frac scheme was introduced (decided at implementation
  time per the plan's open question; existing `z/zd/zdd` fixed-point ints untouched, jerk/snap
  ride alongside as float and get folded back into the fixed-point ints each tick).
- `autopilot_guided.h`/`.c`: new `autopilot_guided_parse_GUIDED_TRAJECTORY()`, additive next
  to `parse_GUIDED_FULL` (untouched). Stub under the `#else` (`!AP_MODE_GUIDED`) branch too.
- `conf/modules/autopilot_guided.xml`: registered the new datalink handler
  (`<datalink message="GUIDED_TRAJECTORY_NED" fun="autopilot_guided_parse_GUIDED_TRAJECTORY(buf)" cond="AP_MODE_GUIDED"/>`).
- No changes to `guidance_indi.c` or `guidance_mfc.c` — confirmed unnecessary, both already
  read `gh->ref.pos/speed/accel` generically each tick.

## Verification done

- Regenerated pprzlink headers directly (`make -C sw/ext/pprzlink pymessages ...`), **not**
  `make pprzlink_protocol` / `*.update` (would `git submodule update` and revert the
  messages.xml edit — known gotcha from [[cerebrum]]/prior sessions).
- Confirmed generated `DL_GUIDED_TRAJECTORY_NED_*` accessor macros match the parser.
- Clean compile, zero warnings/errors touching any changed file:
  - `ANTON_MFC ap` (no `AP_MODE_GUIDED` — exercises the `#else` stub path).
  - `Hoops_111 ap` (has `AP_MODE_GUIDED` — exercises the real parser/setter path), full
    rebuild.

## Follow-up: canned no-external-stream demo trajectory

Added a self-contained way to trigger the flat-trajectory feature without a companion
computer/GCS streaming `GUIDED_TRAJECTORY_NED`, mirroring the existing
`nav_fish.c`/`fish_outdoor.xml` "Guided_run" pattern (flight-plan block → `autopilot_set_mode
(AP_MODE_GUIDED)` + a per-tick `<call>`):

- `sw/airborne/modules/nav/flat_traj_demo_data.h` — data stub: `struct FlatTrajSample`
  (pos/vel/accel/jerk/snap + heading terms, one row per `FLAT_TRAJ_DEMO_DT`) and a `= {0}`
  placeholder array sized for a 1.5s maneuver at 50Hz. **User fills in the real numbers.**
  Gotcha caught by the build: the array bound must be built from integer-millisecond
  `#define`s, not `(int)(1.5f/0.02f)` — a float-derived bound is not a real
  integer-constant-expression in C, so gcc silently treated the file-scope `static const`
  array as a VLA (warn-only, not an error) instead of a fixed-size array.
- `sw/airborne/modules/nav/nav_flat_traj.c`/`.h` — playback module:
  `nav_flat_traj_start()` captures the current NED position/heading as the trajectory
  origin; `nav_flat_traj_run()` indexes the table **by elapsed wall-clock time**, not by
  call count, since flight-plan `<call>` runs at the ~4Hz nav loop rate while the table
  is sampled much faster — time-indexing also means this exercises the same
  sparse-update→Taylor-extrapolation path a real external streamer would.
- `conf/modules/nav_flat_traj.xml`, `conf/flight_plans/ENAC/flat_traj_demo.xml` (new
  standalone flight plan, based on `rotorcraft_basic.xml`, not wired into any operational
  aircraft).
- Compile-verified by temporarily pointing `Hoops_111` at the new flight plan + module —
  **note for next time: `pprz.sh`'s default fleet file is `conf/userconf/ENAC/conf_mfc.xml`,
  NOT `conf/airframes/ENAC/conf_enac.xml`** (CLAUDE.md's "fleet XML is fixed to conf_enac.xml"
  is stale/wrong for the default case). First attempt edited conf_enac.xml and silently
  built the old flight plan. Also hit and fixed a second bug: copying `fish_outdoor.xml`'s
  `InsideSafety(...)` exception without its backing `<sector name="Safety">` polygon caused
  `undefined reference to InsideSafety` at link time — dropped the geofence exception with a
  comment explaining how to add a real one, since fabricating fake site coordinates would be
  worse. All temporary edits (airframe module addition, fleet-conf flight_plan pointer) were
  reverted after the compile check; `git status` on the submodule confirms zero diff on
  Hoops_111's real config, only the 4 new files remain.

### Actually ran it in NPS — found and fixed 2 real bugs

User asked directly "does it execute?" — compiling was not enough, so re-wired Hoops_111
again (module + `conf_mfc.xml` flight_plan pointer, temporarily) and ran it for real:
`./sim.sh Hoops_111 --nav "Wait GPS,Start Engine,Takeoff,+15,Flat_Traj_Demo,+5,Standby"`.
Discovered `sim.sh`'s `--nav SEQ` sends real `JUMP_TO_BLOCK` Ivy messages — the headless
block-trigger mechanism I'd assumed didn't exist actually does; no temporary auto-deroute
hack was needed.

- **Bug 1 (build-system trap, not code):** an incremental `pprz.sh build ... nps` did NOT
  pick up the `conf_mfc.xml` flight_plan pointer change — `sim_anton --nav` listed the OLD
  flight plan's blocks. Needed `pprz.sh rebuild` (clean) to force codegen to re-read the
  fleet conf. Matches the known "incremental codegen keys off airframe/conf hash, not every
  input file" gotcha, just a new trigger for it.
- **Bug 2 (real, in my flight-plan block — now fixed in `flat_traj_demo.xml` +
  `nav_flat_traj.h`'s docstring):** `<exception cond="nav_flat_traj_is_done()"
  deroute="Standby"/>` fired on the block's very first tick, before `<call_once>`s ever ran
  — `nav_flat_traj_is_done()` defaults `true` (never-started) until `nav_flat_traj_start()`
  sets it false, and Paparazzi checks a block's own `<exception>`s before Stage 0. First
  sim run confirmed this: `Block(5)` was entered and NAV_STATUS showed it snapping straight
  back to `Standby` (block 4) every time, never calling `autopilot_set_mode(AP_MODE_GUIDED)`.
  Fix: removed the exception; `<call fun="nav_flat_traj_run()"/>`'s own stage machinery
  already advances once the function returns false, so a trailing unconditional
  `<deroute block="Standby"/>` is enough.
- **Verified after the fix**: `NAV_STATUS` showed `block=5 stage=2` (the run stage) actually
  reached, then a clean return to `block=4`. Filled in throwaway minimum-jerk-polynomial test
  data (7 samples, zero vel/accel at both ends, step = +1m north/east/up) to check real
  motion, not just wiring. Reading `/uav/ROTORCRAFT_FP/{north,east,up}` from the sim CSV
  (`sim_logs/mfc_sim_*.csv`) across exactly the `cur_block==5` window: the vehicle moved
  **+1.52m north, +1.46m east, +0.53m up** — right direction, right order of magnitude
  (some overshoot from the aggressive 2.47 m/s² test acceleration + pre-existing velocity
  from Standby's in-transit `NavGotoWaypoint`, and up lagged since 1.5s wasn't quite enough
  for the slower vertical loop to fully converge). This is real end-to-end confirmation:
  block trigger → `guidance_h_set_flat`/`guidance_v_set_flat` → Taylor-extrapolated
  reference → INDI stabilization → actual vehicle motion, not just "it links."
- All temporary state (module wiring, fleet-conf pointer, test data, `DT_MS`) reverted again
  after the run; `flat_traj_demo_data.h` is back to the all-zero `= {0}` stub. Only the two
  genuine fixes (dropped exception in `flat_traj_demo.xml`, corrected docstring in
  `nav_flat_traj.h`) persist.

## Follow-up: end-of-trajectory hold + origin-continuity fix (user-reported)

User (now iterating on `Hoops_111_MFC`/MFC guidance+stabilization, their own conf/airframe
edits) reported two things after hands-on testing: (1) glitchy behavior + setpoint reverting
to zero when the trajectory finishes, (2) suspected the origin uses the "previous setpoint"
rather than current position, which they reasoned would cause a kick if measured position
isn't exactly at the previous setpoint.

- **Root cause of (1)**: once `nav_flat_traj_run()` returned false, the flight-plan block
  exited (`NextBlock()`/`<deroute>`), handing control to whatever came next (e.g. `Standby`'s
  `guidance_h_nav_enter()` → `guidance_h_set_pos(nav.carrot)`, which snaps toward the
  flight-plan's own waypoint — "zero" once the user zeroed out the demo plan's waypoints).
  **Fix**: `nav_flat_traj_run()` now always returns `true` — once the trajectory ends it
  keeps re-sending the final (zero vel/accel) sample every tick forever, holding the
  setpoint exactly at the endpoint. `nav_flat_traj_is_done()` still reports completion
  (telemetry/GCS) but no longer ends the block. Flight plan's trailing
  `<deroute block="Standby"/>` removed (now unreachable).
- **Root cause of (2), confirmed correct by re-reading it through**: with the block now
  holding forever, re-triggering the SAME block (e.g. GCS strip button again) doesn't change
  `AP_MODE_GUIDED` (already in it), so `guidance_h_hover_enter()`/`guidance_v_guided_enter()`
  (which would otherwise reset the reference to the raw current measurement) never runs. If
  `nav_flat_traj_start()` seeds the origin from **raw measured position**
  (`stateGetPositionNed_f()`) instead of the guidance system's **own current reference**
  (`guidance_h.ref.pos`, `guidance_v.z_ref`, `guidance_h.sp.heading`), any ordinary tracking
  error between the two becomes a discontinuous "kick" the instant `gh_set_flat_ref()` hard-
  resets the reference model. **Fix**: origin now comes from `guidance_h.ref.pos`/
  `guidance_v.z_ref`/`guidance_h.sp.heading` (via `POS_FLOAT_OF_BFP`), not raw state. On a
  fresh NAV→GUIDED transition these are identical (hover_enter already synced them that same
  tick), so this is strictly safer with no downside on first trigger.
- **Verified in NPS** (`Hoops_111_MFC`, using the user's own real 751-sample/500Hz data file,
  not touched): ran `--nav "...,Takeoff,+15,Flat_Traj_Demo,+5,Flat_Traj_Demo,+5"` (re-trigger
  case). Scanning `/uav/SP/guidance/h_ref_n`/`h_ref_e` for any >5cm single-sample jump across
  ~118k scope samples found exactly one, at the very first trajectory start (not the
  retrigger) — the retrigger shows the reference stepping cleanly from the first endpoint
  (~1.08, 1.00) to a second one ~1m further (~2.08, 2.00), confirming no kick. Final state
  (t=241s, well after both triggers) still holds at (2.078, 2.004) — never reverted.
- **Unrelated finding, flagged not fixed**: over the same long unattended hold, MFC's own
  measured-position scope vars (`MFC_GUIDANCE/me_x`/`me_y`) diverge wildly after ~90-120s
  (bounded ~2-4m error at t=30-60s, swinging to hundreds of meters by t=120-241s). This
  reproduces independently of the two fixes above (values are already elevated at t=30s,
  before any repeated-hold-specific behavior could matter) and matches the user's own prior
  commit message flagging MFC guidance/stab tuning as unresolved
  ("something is very wrong with mfc guidance ... maybe a sign flip or a scale issue").
  Very likely a pre-existing MFC long-duration stability/tuning issue, not something these
  two fixes introduced — flagged to the user, not investigated further (out of scope).

## Committed, then two more real bugs found via user reports

Committed on `feature-diff-flatness` (`5185b6749` in `paparazzi/`, `52e35ff` in the `pprzlink`
submodule on `mfc-scope-parity`). User then reported two more things while iterating on
`Hoops_111_MFC`:

1. **Pre-flat-traj altitude climbing to 6+m instead of ~2m.** Root cause #1 (fixed): the
   flight plan's `alt="152" ground_alt="147"` (copied verbatim from `rotorcraft_basic.xml`)
   gave every waypoint without an explicit `height=` a default of 5m AGL, not 2m — the `2.0`
   in `Takeoff`'s exception is only the threshold to *leave* that block, not a target
   altitude. Changed `alt` to `149` (149-147=2m) so all waypoints default to 2m. **Root
   cause #2 (diagnosed, NOT fixed):** even after that, altitude still climbs to ~3.85m and
   settles there — confirmed via generated `WAYPOINTS_ENU` that the compile-time target
   really is 2.0m, so the divergence is a runtime issue. Hypothesized it was vertical
   reference-model velocity carryover from `Takeoff`'s open-loop `NavVerticalClimbMode` into
   `Standby`'s altitude-hold (`guidance_v_mode_changed(GUIDANCE_V_MODE_NAV)`'s
   `GuidanceVSetRef()` reset only fires on an actual `AP_MODE` transition, and
   `Takeoff`→`Standby` both stay in `AP_MODE_NAV` the whole time, so it never fires) —
   confirmed `guidance_v_run_pos()` in `guidance_mfc.c:507` does read the standard
   `gv->z_ref` reference-model output, so the theory was plausible. **Tested and disproven**:
   added an explicit `guidance_v_set_ref(stateGetPositionNed_i()->z, 0, 0)` at `Standby`
   entry, rebuilt, reran — identical ~3.85m settle, no change. Reverted the ineffective fix
   (traced all the way to `guidance_v.z_ref`/`z_sp` being correct only in *sim scope
   telemetry naming*, not confirmed at the true source of `nav.nav_altitude`). **Left open,
   flagged to user** — looks like a pre-existing MFC vertical-guidance quirk unrelated to the
   flat-trajectory feature (entirely reproducible with stock `Takeoff`/`Standby` blocks,
   before `AP_MODE_GUIDED` is ever entered), matching the user's own prior commit
   acknowledging general MFC guidance/tuning problems. Needs focused investigation into
   `nav.nav_altitude`/`WaypointAlt()` at the exact runtime moment, not guessed at further.

2. **"The estimate itself jumping when the traj starts."** Root cause found and fixed: user's
   real 751-row/500Hz trajectory data has `order=0` (pos-only, by design) but EVERY row
   carries a sustained, large `heading_accel` (±2.094 rad/s²) and a constant
   `heading_jerk=-2.792527` (exact same value all 751 rows — clearly an artifact of however
   their generator computed those fields, not intentional). Bug: `guidance_h_set_flat()`
   never gated `heading_rate`/`heading_accel`/`heading_jerk`/`heading_snap` (or linear
   velocity `vx`/`vy`/`vz`) by `order` at all — only position's accel/jerk/snap were masked,
   even though the docstrings already documented the correct "trailing fields beyond order
   are zero" contract. So with order=0, those huge heading derivatives were being integrated
   into the yaw setpoint from t=0 via the quartic Taylor update, spinning the vehicle and
   (through normal guidance/attitude coupling) dragging translational position along with
   it. **Fixed**: added `order >= 1` gating for velocity (both linear and heading_rate) in
   `guidance_h_set_flat()`/`guidance_v_set_flat()`, and `order >= 2/3/4` gating for
   heading_accel/jerk/snap, matching the existing position-side gating exactly. **Verified**:
   re-ran the same NPS test — zero single-tick jumps >0.2rad detected in truth position or
   heading across the whole run; heading now smoothly settles at exactly `atan2(1,1)=0.785
   rad` (their trajectory's *intentional* raw `.heading` field, which does legitimately ramp
   to 45° — that part is real, not a bug, since the base `heading` value itself is correctly
   never order-gated, only its derivatives are).

## Z-overshoot root cause: WaypointAlt() frame mismatch (SOLVED — supersedes the "left open" note above)

User pushed back correctly: "it has nothing to do with MFC internals because the sp itself
follows that weird path." They were right — the *setpoint* walked up, so it was upstream of
any controller. Traced by instrumenting the whole nav→guidance altitude chain
(commit `609b0cd9e`).

**A prerequisite bug found first:** `nps_scope_z_sp`/`nps_scope_z_ref` in `guidance_v.c` were
`NPS_SCOPE_VAR`-registered but **never assigned anywhere** — permanently reporting 0. That's
why every earlier CSV showed `guidance_v/z_ref = 0` and why this took so long to see. Fixed
(assigned in `guidance_v_run()`), plus added `nav_altitude`/`fp_altitude`/`hmsl_origin`/
`v_mode` to make the nav-layer chain observable. All SITL-only.

**Root cause:** `conf/modules/ins_ekf2.xml:83` defines `USE_ALT_LLA_WAYPOINTS=TRUE`, which
redefines (`modules/nav/waypoints.h:56-60`):
```c
#define WaypointAlt(_wp)  waypoint_get_lla_alt(_wp)
// = waypoints[wp].lla.alt/1000.f - stateGetLlaOrigin_i().alt/1000.f
```
i.e. an altitude relative to the **live INS origin**. But `guidance_v.c:322` consumes it as a
plain **local NED** altitude (`z_sp = -POS_BFP_OF_REAL(nav.nav_altitude)`). Those frames only
agree while the origin's ellipsoid altitude equals the local frame's zero. A local waypoint's
`.lla` is computed lazily once by `waypoint_globalize()` and then cached (`WP_FLAG_LLA_I`), so
when EKF2 refined its origin ~1s after takeoff the cached-minus-live difference jumped.

Instrumented proof (transition table, every discrete change):
```
 time | nav_block | wp_stdby_alt | fp_altitude | nav_altitude | z_sp | hmsl_origin
10.60 |     4     |      2       |    1.98     |      2       | -2.04|   147
11.60 |     4     |      2       |    1.98     |      2       | -2   |   147.059   <- origin moves
11.65 |     4     |      2       |    3.882    |    3.882     | -2   |   147.059   <- setpoint jumps
```
`wp_stdby_alt` (via `waypoint_get_alt`, the ENU value) stays **2.0 the entire time** — the
waypoint is never moved. Only the LLA-vs-origin *difference* changes. Note `hmsl_origin` moved
only +0.06m while the setpoint moved +1.9m: the origin's **ellipsoid** altitude shifted ~1.9m
even though its MSL altitude barely did, i.e. the runtime geoid separation disagrees with the
codegen `NAV_MSL0` (51.85m) by about that much.

**Fix:** give the Standby block an explicit `alt="2.0"` (`<stay wp="STDBY" alt="2.0"/>`), which
goes straight into `nav.fp_altitude` in the frame `guidance_v` actually wants, bypassing
`WaypointAlt()` entirely. Verified: holds z_sp = -2.0 / truth -1.99 through the origin shift,
then the flat trajectory steps to -3.0 (exactly the intended +1m). Scoped to this bench/sim
plan deliberately — outdoors on GPS the LLA-referenced behaviour is intentional (it keeps
waypoint altitudes absolute rather than drifting with the local frame), so it was NOT changed
globally.

**Also corrected an earlier wrong call in this same session:** the `alt="152"→"149"` change was
necessary (waypoints default to `alt - ground_alt`, so they were targeting 5m) but NOT
sufficient, and my follow-up hypothesis — vertical reference-model velocity carryover from
`Takeoff`'s open-loop climb — was **wrong**; I tested it with an explicit
`guidance_v_set_ref()` at Standby entry, saw zero change, and reverted it. Worth remembering:
the earlier "settles at 4.875 with alt=152, 3.848 with alt=149" pair should have been the tell,
since a 3m change in waypoint height moved the settle by only ~1m — inconsistent with any pure
waypoint-altitude explanation, and consistent with a constant ~1.9m origin offset on top.

Portability check on the retained instrumentation: an earlier probe read `WaypointAlt(WP_STDBY)`
directly, which would have broken every other airframe (`guidance_v.c` is shared core;
`WP_STDBY` is flight-plan-specific). Removed before committing; verified `Hoops_111_MFC nps`,
`ANTON_MFC nps`, and `Hoops_111 ap` all build clean.

## Not done (flagged, out of scope for this session)

- MFC follow-up (plan section 5): `guidance_mfc.c` still only reads `gh->ref.pos`, not
  `ref.speed/accel` as feedforward. Separate task.
- Verification-plan items 1–2 (synthetic min-snap Python/Ivy publisher + PlotJuggler
  inspection) and item 4 (enabling `AP_MODE_GUIDED` on a bench airframe) — not built; only
  a compile-time correctness check was done here, matching this repo's stated bar
  ("a successful compile is the only automated correctness check; functional correctness
  requires flight test / sim").
- The pprzlink submodule change is uncommitted, still on its existing branch
  `mfc-scope-parity` — did not create a matching `feature-diff-flatness` branch there since
  the user's branch instruction was scoped to `paparazzi/` itself.

## Next steps

1. Decide whether to commit (paparazzi/ + pprzlink submodule) — nothing was committed this
   session, only built and compile-checked.
2. Write the synthetic min-snap trajectory publisher for NPS verification (verification plan
   step 1).
3. MFC feedforward follow-up if MFC guidance is the target consumer.
