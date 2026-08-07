# Plan — HEOL Stages 2+3: the `heol` library and ANTON_HEOL guidance

> Continuation of [[MFC Decoupled Core Port (HEOL Stage 1)]], which is executed
> and flight-ready (coupled path proven bit-identical to pre-change; decoupled
> path added and bounded). This plan executes stages 2 and 3 from that plan's
> sketch. Stage 4 (trajectory plumbing) turned out to be **already built** by
> the "flatness trajectory setpoints" work — `guidance_h_set_flat`/
> `guidance_v_set_flat`, the Taylor-extrapolating reference model in
> `guidance_h_ref.c`/`guidance_v_ref.c`, and `nav_flat_traj.c` all exist and
> compile today. Only `flat_traj_demo_data.h`'s canned table is still a
> pos-only placeholder (`FLAT_TRAJ_DEMO_ORDER 0`) — filling it with a real
> min-snap maneuver is explicitly deferred to a later session, per user
> decision.
>
> Created 2026-08-06/07 in plan mode. Not yet executed.

## Context

Stage 1 made the SISO MFC estimator structure (coupled/decoupled) a runtime
choice, but a single `MfcParameters` instance is still one closed loop with
setpoint tracking baked in. HEOL is architecturally different: the plant sees
`u = u_ff + u_fb`, where `u_ff` is a flatness-inverted feedforward computed
directly from the trajectory reference, and `u_fb` is a **decoupled** MFC loop
that only ever sees the *residual* `ε = ŷ − r` (measurement minus reference) —
never the raw setpoint, never the total command. This lets `u_fb`'s estimator
converge on the *leftover* disturbance (real drag, mass error, wind) instead of
having to reconstruct the whole trajectory-tracking problem from scratch.

Stage 1's sketch already worked out that this needs **no changes to
`mfc_core`**: driving a decoupled `MfcParameters` with `setpoint = 0` and
`measure = ε` makes `mfc_core` *be* the `u_fb` block exactly, because its
internal `error[0] = measure − setpoint_trajec[0]` collapses to `ε − 0 = ε`,
its own feedforward (`dot_dot_setpoint_trajec`, from setpoint history) collapses
to `0` (constant zero setpoint ⇒ zero second difference), and its estimator's
`d2u` term already uses `command[1]` — this loop's own previous command — so
the estimator only ever sees `u_fb`, not `u_ff + u_fb`. This was verified
against the current `mfc_core.c` (re-read in full for this plan): confirmed
correct, nothing to change there.

**Decisions taken (from the earlier clarifying round):**
1. This plan covers stages 2 (library) + 3 (guidance wiring) only. Trajectory
   data-table work is a separate follow-up.
2. New **ANTON_HEOL** aircraft (new airframe XML, new fleet entry), not a
   flag on ANTON_MFC — keeps ANTON_MFC as the untested-in-decoupled-mode but
   flight-tested fallback, same pattern stage 1 used for coupled-vs-decoupled.
3. Scope is **guidance only** (position loop: gx/gy/gz), matching stage 1's
   scope decision and mirroring `guidance_mfc.c`. Stabilization stays on
   `stabilization` type `mfc` (attitude loop untouched) — ANTON_HEOL pairs
   HEOL guidance with the existing MFC attitude stabilizer, same as ANTON_MFC
   pairs MFC guidance with MFC stabilization today.
4. The vertical feedforward's mass/gravity inversion reuses `MODEL_MASS`
   (`<section name="MODEL" prefix="MODEL_"><define name="MASS" .../>`,
   `anton_mfc.xml:160-161`) — already the source of
   `GUIDANCE_MFC_GZ_NOMINAL_HOVER_THROTTLE = -9.81*MODEL_MASS`
   (`anton_mfc.xml:313`), confirming the sign convention: gz command is
   Newtons, NED, **negative = up**.

## Unit contract for `u_ff` (worked out from `guidance_mfc.c`, confirmed by reading it)

| axis | command unit | `u_ff` formula | source data |
|---|---|---|---|
| gx, gy | m/s² (acceleration — `accel_to_att_sp` converts to tilt) | `u_ff = ẍ_ref` directly, no inversion | `gh->ref.accel.{x,y}` (already Taylor-extrapolated from jerk/snap by `gh_update_ref_from_flat_ref`) |
| gz | N, NED, negative = up | `u_ff = MODEL_MASS * (zdd_ref − 9.81)` | `gv->zdd_ref` (int32 BFP → float via `ACCEL_FLOAT_OF_BFP`), reusing the sign convention `GZ_NOMINAL_HOVER_THROTTLE = -9.81*MODEL_MASS` already encodes for the hover case (`zdd_ref=0` ⇒ `u_ff = -9.81*MODEL_MASS`, matches) |

## 1. `heol.c`/`heol.h` — the library (Stage 2)

New files at
`paparazzi/sw/airborne/firmwares/rotorcraft/stabilization/heol.c`/`.h`,
mirroring `mfc_core.c`/`.h`'s shape (a thin, `dir="stabilization"`,
dependency-only library — same location as `mfc_core` since it *wraps*
`mfc_core`, not because it's attitude-specific; `guidance_heol.c` below is the
guidance-side consumer).

```c
// heol.h
#include "firmwares/rotorcraft/stabilization/mfc_core.h"

/** One HEOL-controlled axis: u = u_ff + u_fb, where u_fb is a decoupled
 * mfc_core loop driven by the tracking residual (never the raw setpoint or
 * the total command — see heol.c file header for why that matters). */
struct HeolParameters {
  struct MfcParameters mfc;   /* the u_fb loop; caller must set kp/kd/ki/alpha/u_min/u_max */
  float trajectory_ref;       /* r: this tick's flat reference value, set by caller */
  float u_ff;                 /* this tick's feedforward, set by caller */
  float epsilon;              /* r - measure residual actually fed to mfc_core, for telemetry */
  float command;              /* u_ff + u_fb: what the caller applies to the plant */
};

extern void heol_init(struct HeolParameters *heol, float sample_time);
extern void heol_reset(struct HeolParameters *heol);
extern void heol_run(bool in_flight, struct HeolParameters *heol,
                      float measure, float trajectory_ref, float u_ff);
```

```c
// heol.c
void heol_init(struct HeolParameters *heol, float sample_time) {
  mfc_siso_init(&heol->mfc, sample_time);
  heol->mfc.decoupled = true;      // HEOL's u_fb is always the decoupled structure
  heol->mfc.use_trajec_sp = false; // setpoint is held at 0; no filtering needed
  heol->mfc.setpoint = 0.f;
}

void heol_reset(struct HeolParameters *heol) {
  mfc_siso_reset(&heol->mfc);
}

void heol_run(bool in_flight, struct HeolParameters *heol,
              float measure, float trajectory_ref, float u_ff) {
  heol->trajectory_ref = trajectory_ref;
  heol->u_ff = u_ff;
  float epsilon = measure - trajectory_ref;
  mfc_siso_run(in_flight, &heol->mfc, epsilon);
  heol->epsilon = epsilon;
  heol->command = u_ff + heol->mfc.command[0];
}
```

Doc comment at the top of `heol.c` should restate the invariant plainly: *do
not* call `mfc_siso_run` on `heol->mfc` from anywhere else, and *do not* feed
it anything but `epsilon` — either breaks the "estimator sees only `u_fb`"
property this whole design rests on.

`conf/modules/heol.xml`, mirroring `mfc_core.xml` exactly (dependency-only,
no settings, depends on `mfc_core`):

```xml
<module name="heol" dir="stabilization" task="control">
  <doc><description>
    HEOL (u = u_ff + u_fb) wrapper around mfc_core: u_fb is a decoupled SISO
    MFC loop driven by the tracking residual only. See heol.h.
  </description></doc>
  <header><file name="heol.h"/></header>
  <dep><depends>mfc_core</depends></dep>
  <makefile target="ap|nps" firmware="rotorcraft">
    <file name="heol.c" dir="$(SRC_FIRMWARE)/stabilization"/>
  </makefile>
</module>
```

**Standalone proof, before wiring anything else.** Reuse the exact host-side
harness technique from stage 1 (`heol.c` has the same two dependencies as
`mfc_core.c`, both stubbable) to check numerically: with `u_ff = 0` and a
step-change trajectory_ref, `heol.command` should equal a plain decoupled
`mfc_core` run (`setpoint = trajectory_ref` directly instead of the
residual trick) to float rounding — this is the regression check that proves
the `setpoint=0, measure=ε` reformulation is truly equivalent to a decoupled
loop tracking `r` directly, just with `u_ff` split out.

## 2. `guidance_heol.c`/`.h` — wiring into ANTON's guidance (Stage 3)

New files at
`paparazzi/sw/airborne/firmwares/rotorcraft/guidance/guidance_heol.c`/`.h`,
a mutually-exclusive guidance variant mirroring `guidance_mfc.c`/`.h`
structurally (same `guidance_h_run_pos/speed/accel`,
`guidance_v_run_pos/speed/accel`, `guidance_h_run_enter`/`guidance_v_run_enter`
plug functions; same Butterworth position filters seeded on mode entry; same
`accel_to_att_sp`/`th_sp_from_thrust_f` output packaging), but:

- Three `struct HeolParameters heol_gx, heol_gy, heol_gz;` instead of
  `MfcParameters`.
- Horizontal (`guidance_heol_horiz`, mirrors `guidance_mfc_horiz` at
  `guidance_mfc.c:578-612`):
  ```c
  float x_ff = ACCEL_FLOAT_OF_BFP(gh->ref.accel.x);
  float y_ff = ACCEL_FLOAT_OF_BFP(gh->ref.accel.y);
  heol_run(in_flight, &heol_gx, x_meas, x_sp, x_ff);
  heol_run(in_flight, &heol_gy, y_meas, y_sp, y_ff);
  acc_x = heol_gx.command; acc_y = heol_gy.command;
  ```
  (`x_sp`/`x_meas` computed exactly as in `guidance_mfc_horiz` — filtered NED
  position for `measure`, `POS_FLOAT_OF_BFP(gh->ref.pos.x)` for
  `trajectory_ref`.)
- Vertical (`guidance_heol_vert`, mirrors `guidance_mfc_vert` at
  `guidance_mfc.c:525-559`):
  ```c
  float z_ff = GUIDANCE_HEOL_MASS * (ACCEL_FLOAT_OF_BFP(gv->zdd_ref) - 9.81f);
  heol_run(in_flight, &heol_gz, z_meas, z_sp, z_ff);
  thrust_cmd = heol_gz.command;
  ```
  Keep the same nominal-hover fallback, thrust-filter seeding, and
  `THRUST_SP_FLOAT` normalization (`GZ_MAX_THRUST`) as `guidance_mfc_vert` —
  none of that packaging is HEOL-specific.

`guidance_heol.h` declares `extern struct HeolParameters heol_gx, heol_gy, heol_gz;`
and `guidance_heol_init(void)`, mirroring `guidance_mfc.h`.

`conf/modules/guidance_heol.xml`, mirroring `guidance_mfc.xml`
(`paparazzi/conf/modules/guidance_mfc.xml:1-101`) closely:

- `<section name="GUIDANCE_HEOL" prefix="GUIDANCE_HEOL_">` with the same
  per-axis `ALPHA/KP/KD/KI/EST_HOLD_TIME` defines as `guidance_mfc.xml`
  (decoupled-only here, so no `_DECOUPLED`/`_TIME_TRAJECTORY`/
  `_INTEGRATION_WINDOW`/`_COMMAND_FILTER` defines — those are coupled-mode or
  reference-filter concepts HEOL's `u_fb` doesn't use since
  `use_trajec_sp=false` is hardcoded in `heol_init`). Add `MASS`,
  `GZ_NOMINAL_HOVER_THROTTLE`, `GZ_MAX_THRUST` matching `guidance_mfc.xml:31,41-42`.
- `<dl_settings>` panel exposing `heol_gx.mfc.{kp,kd,ki,alpha,u_min,u_max,est_hold_time,enabled}`
  (note the extra `.mfc.` — the settings var path walks through the embedded
  struct) for gx/gy/gz, same `persistent="true"` convention.
- `<dep><depends>@navigation,guidance_rotorcraft,heol</depends><provides>guidance,attitude_command</provides></dep>`
- `<init fun="guidance_heol_init()"/>`
- `<makefile target="ap|nps" firmware="rotorcraft">` with
  `<define name="GUIDANCE_PID_USE_AS_DEFAULT" value="FALSE"/>`, same as
  `guidance_mfc.xml:99`.

## 3. `anton_heol.xml` + fleet registration

Clone `paparazzi/conf/airframes/ENAC/quadrotor/anton_mfc.xml` →
`anton_heol.xml`. Changes only:
- `<module name="guidance" type="mfc"/>` (`anton_mfc.xml:58`) →
  `<module name="guidance" type="heol"/>`.
- Replace `<section name="GUIDANCE_MFC" ...>` (`anton_mfc.xml:279-316`) with
  `<section name="GUIDANCE_HEOL" ...>` using the defines above; convert the
  existing `GX/GY_ALPHA/KP/KD` values across (drop the coupled-only defines
  per §2) as a reasonable starting tuning, not a claimed-equivalent one — HEOL
  is a structurally different loop, there is no exact conversion formula the
  way stage 1 had one for kp/kd.
- Leave `stabilization type="mfc"` (`anton_mfc.xml:54-57`) and everything else
  (servos, commands, IMU, MAG, MODEL, STABILIZATION_MFC section, NAV, BAT,
  AUTOPILOT, SONAR, AGL, TAG_TRACKING, MISC, GCS, SIMULATOR) untouched.

Register in `paparazzi/conf/userconf/ENAC/conf_mfc.xml` (the fleet file
ANTON_MFC/ANTON_ONELOOP/Hoops_111_MFC actually live in — **not**
`conf/airframes/ENAC/conf_enac.xml`, confirmed by grep), cloning the
`ANTON_MFC` entry (`conf_mfc.xml:13-23`):
- `name="ANTON_HEOL"`, next free `ac_id` (218/221/177/111 taken → e.g. `219`).
- `airframe="airframes/ENAC/quadrotor/anton_heol.xml"`.
- Same `radio`, `telemetry="telemetry/mfc_flight_test.xml"`,
  `flight_plan="flight_plans/ENAC/flat_traj_demo.xml"` (already drives
  `nav_flat_traj`, no flight-plan change needed), `settings`.
- `settings_modules`: same list as ANTON_MFC's but
  `modules/guidance_mfc.xml` → `modules/guidance_heol.xml`.

## 4. Out of scope, deliberately

- Stabilization-axis HEOL (attitude loop) — not requested, ANTON_HEOL keeps
  `stabilization type="mfc"`.
- Filling in `flat_traj_demo_data.h` with a real jerk/snap-populated
  trajectory (still `FLAT_TRAJ_DEMO_ORDER 0`) — explicitly deferred.
- `GUIDED_TRAJECTORY_NED` companion-computer streaming path — already exists
  and is untouched by this plan; `nav_flat_traj` (the canned-table path) is
  what ANTON_HEOL's flight plan uses.
- Any change to `mfc_core.c`/`.h` — confirmed unnecessary; stage 1's structure
  already supports this exactly.

## Verification

**1. Standalone `heol.c` equivalence check** (§1 above) — host-compiled,
before touching guidance code at all.

**2. Compile:**
```bash
./pprz.sh build ANTON_HEOL ap
./pprz.sh build ANTON_HEOL nps
./pprz.sh build ANTON_MFC ap     # regression: untouched, must still link
```

**3. SITL.** `./sim.sh` against `ANTON_HEOL` running the existing
`nav_flat_traj` flight-plan block (pos-only trajectory today, since
`FLAT_TRAJ_DEMO_ORDER=0` — still exercises the full HEOL command path with
`u_ff` from `gh->ref.accel`/`gv->zdd_ref`, which the reference model populates
from its internal 2nd-order dynamics even on a pos-only setpoint). Watch
`heol_gx.epsilon`/`heol_gz.epsilon` via PlotJuggler (extend
`plotjuggler_mfc.xml`) — in steady hover this should sit near zero and the
estimator (`heol_gx.mfc.estimator`) should read as a small residual
disturbance, much smaller in magnitude than the equivalent decoupled-mode
`mfc_gx.estimator` from stage 1's A/B, since `u_ff` now carries the bulk of
the known dynamics.

**4. OpenWolf upkeep**: update `anatomy.md` for the 6 new files + 2 new/1
modified XML + fleet entry, append to `memory.md`, record the "HEOL = mfc_core
+ residual-only wrapper" pattern in `cerebrum.md`, write
`Knowledge/Sessions/<date>-heol-stage-2-3.md`.

## See also

- [[MFC Decoupled Core Port (HEOL Stage 1)]] — the stage this continues, and
  the source of the `mfc_core` unit contract and estimator invariants relied
  on here
- [[Flatness Trajectory Setpoints (Pos-Vel-Accel-Jerk-Snap + Psi)]] — the
  `gh->ref`/`gv` reference-model plumbing this reads `u_ff` from (built since
  that plan was written)
