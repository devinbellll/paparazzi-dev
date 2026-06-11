# Session: 2026-06-10 — In-flight control-stack switching design

Follow-on to [[2026-06-10-autopilot-stack-binding]]. How to switch
guidance/stabilization stacks **live in flight** (GCS setting) rather than at
build time.

## Prerequisite is unchanged

Both stacks must still be compiled in and coexisting (see the binding note):
unique `stabilization_mfc` symbol, additive `stabilization_mfc.xml`, airframe
declaring both stabs + both guidances. Runtime switching is impossible if only one
stabilizer is linked.

> **BLOCKER discovered while implementing:** the INDI and MFC stabilizer cores
> (`stabilization_indi.c`, `stabilization_mfc.c`) define identically-named
> non-static globals — `actuators_pprz`, `actuator_state_filt_vect`,
> `act_is_servo`, `act_pref`, `num_thrusters`, `g1g2`, `Bwls`, etc. Linking both
> into one binary causes duplicate-symbol errors. Coexistence therefore requires
> **namespacing one core's globals** (prefix the ENAC-local MFC core's globals
> with `mfc_`, leaving upstream INDI untouched so the rest of the fleet is
> unaffected). This is a real porting sub-task, not just XML wiring.

## What changes vs. the compile-time 2×2 plan

Selection moves out of the autopilot XML call-site (fixed at build) into a
**runtime dispatcher** driven by GCS settings.

You cannot branch cleanly in the autopilot XML, because each `<call>` with
`store="struct ... x"` *declares* that variable in the generated scope — two
conditional calls both storing into the same name = duplicate declaration. So put
the branch in C.

### New dispatcher module `ctrl_stack.c/.h`

Exposes exactly the three signatures the autopilot already expects:

```c
enum ctrl_src { CTRL_INDI, CTRL_MFC };
struct {
  enum ctrl_src guid_v, guid_h, stab;              // GCS-settable
  enum ctrl_src last_guid_v, last_guid_h, last_stab; // edge detection
} ctrl_stack;

struct ThrustSetpoint ctrl_stack_run_vert(bool in_flight) {
  if (ctrl_stack.guid_v != ctrl_stack.last_guid_v) {   // bumpless transfer
    if (ctrl_stack.guid_v == CTRL_MFC) guidance_mfc_enter();
    else                               guidance_v_run_enter();
    ctrl_stack.last_guid_v = ctrl_stack.guid_v;
  }
  return (ctrl_stack.guid_v == CTRL_MFC)
       ? guidance_mfc_run_vert(in_flight, &guidance_v)
       : guidance_v_run(in_flight);
}
// ctrl_stack_run_horiz(in_flight, &thrust_sp) -> StabilizationSetpoint
// ctrl_stack_run_stab (in_flight, &stab_sp, &thrust_sp, cmd)
```

### Autopilot NAV `<control>` calls dispatchers unconditionally

```xml
<call fun="ctrl_stack_run_vert(autopilot_in_flight())"             store="struct ThrustSetpoint thrust_sp"/>
<call fun="ctrl_stack_run_horiz(autopilot_in_flight(), &thrust_sp)" store="struct StabilizationSetpoint stab_sp"/>
<call fun="ctrl_stack_run_stab(autopilot_in_flight(), &stab_sp, &thrust_sp, stabilization.cmd)"/>
```

Three independent enums keep the full 2×2 (and H/V split) live, selectable via
`dl_setting`s (`values="INDI|MFC"`) in the module XML.

## The part that matters for in-flight: bumpless transfer

Both controllers carry internal state (MFC algebraic estimator + reference
filter; INDI rate/accel filters + integrators). Flipping the selector with stale
state kicks the aircraft. Cheap fix: on the **rising edge** of a switch, call the
incoming controller's `_enter()` (`guidance_mfc_enter`, `stabilization_mfc_enter`,
`guidance_v_run_enter`, `guidance_h_run_enter`, `stabilization_indi_enter`, …) so
it re-inits references/integrators to current state and tracks from there.

Caveats:
- **Filter warm-up:** `_enter()` resets references but signal filters need a few
  ticks to settle (tens of ms at 500 Hz). First switch under aggressive motion is
  the risky one — test switches in hover first.
- **Cost vs. warmth:** alternative is running *both* controllers every tick and
  selecting the output (truly bumpless, but ~2× control-loop CPU and the idle
  controller's integrators keep winding unless frozen). Start with edge-reinit;
  only go dual-run if transients show up.

## Net diff from compile-time plan

| | Compile-time 2×2 | In-flight 2×2 |
|---|---|---|
| Both stacks linked | yes | yes (same prereq + global namespacing) |
| Selection point | which `<call_block>` NAV calls | runtime enums in `ctrl_stack` |
| New file | none | `ctrl_stack.c/.h` + module XML |
| Switch transients | n/a (ground) | needs `_enter()` on edge |
| GCS control | rebuild | live `dl_setting` |

Autopilot XML actually gets *simpler* (no duplicated control blocks); cost is one
new dispatcher module + the global-namespacing porting sub-task.

## Definitive collision inventory (2026-06-10)

`stabilization_indi.c` ∩ `stabilization_mfc.c` non-static globals that collide at
link — **~40 symbols**, not the ~7 first sampled:

```
act_dyn_discrete, act_first_order_cutoff, act_is_servo, act_obs, act_pref,
act_rate_limit, actuators_pprz, actuator_state, actuator_state_filt_vect(/d/dd),
angular_acceleration, angular_accel_ref, angular_rate_ref, Bwls,
calc_g1_element(), calc_g1g2_pseudo_inv(), calc_g2_element(), ddu_estimation,
du_estimation, estimation_rate_d(/dd), g1, g1_est, g1g2, g1g2inv,
g1g2_pseudo_inv, g1g2_trans_mult, g1_init, g2, g2_est, g2_init, mu1, mu2,
num_thrusters, q_filt, r_filt, sum_g1_g2, wls_stab_p
```

Of these, only ~7 are exposed via `stabilization_mfc.h` (`g1g2`, `Bwls`,
`actuators_pprz`, `actuator_state_filt_vect`, `act_is_servo`, `act_pref`,
`wls_stab_p`); grep confirms nothing outside `stabilization_mfc.c/.h` references
them. The other ~33 are file-local.

## RESOLUTION — config-time switch implemented (2026-06-10)

User opted for **config-time** switching instead of runtime. This sidesteps the
40-symbol collision entirely: each build links only ONE stabilizer, so no
namespacing port is needed. The MFC controller C code was NOT touched.

**Single switch** in `anton_mfc.xml` — the `type` on the stabilization module:
- `type="mfc"`  → INDI guidance → MFC stabilizer  (INDI->MFC)  [default]
- `type="indi"` → MFC guidance  → INDI stabilizer (MFC->INDI)

**How it works:**
- Guidance is auto-paired to the compiled stabilizer. The stab module makefiles
  emit `MFC_OUTPUTS` (mfc) vs `INDI_OUTPUTS` (indi); the autopilot includes derive
  `STACK_USE_MFC_GUIDANCE` (0 when MFC stab, 1 when INDI stab) via
  `<define ... cond="ifdef/ifndef MFC_OUTPUTS"/>`.
- `gen_autopilot.ml` hoists+dedups `store=` declarations and wraps a `<call cond=>`
  as `if (cond) { var = fun(); }`. So NAV inlines both guidance variants under
  `cond="!STACK_USE_MFC_GUIDANCE"` / `cond="STACK_USE_MFC_GUIDANCE"`, sharing one
  `thrust_sp`/`stab_sp` decl; the compiler drops the dead branch.
- `stabilization_run()` is the same call for both stacks — the compiled stab
  module supplies the implementation. Both guidance modules + both attitude param
  sections are always compiled in.
- INDI gains borrowed from `anton_indi_aruco.xml` (G1/G2/ACT_FREQ/COMMANDS already
  match anton_mfc's FR/BR/BL/FL setup).

**Files changed:** `anton_mfc.xml` (STACK SELECT comment + STABILIZATION_ATTITUDE_INDI
section), `anton_mfc_autopilot.xml` (auto-derive define, NAV cond-based guidance,
removed the two fixed control_blocks, on_enter MFC guard).

**Verified:** both variants build+link clean for the `nps` target (`simsitl`
produced). The `ap` target fails to link on a *pre-existing, unrelated* issue —
`guidance_indi.c` references `nps_scope_z_ref`/`nps_scope_z_sp`, which only exist
in the NPS scope file (not my change).

**GOTCHA (bug-023):** switching `type=` and doing an *incremental* build fails with
`'MFC_OUTPUTS' undeclared` from `stabilization_mfc.h` — the stale
`generated/modules.h` still includes the dropped MFC module's header. **Must
rebuild with `build_fw.sh -c`** after changing the stabilizer type. Documented in
the STACK SELECT comment.

**Final state:** default restored to `type="mfc"` (INDI->MFC); clean `nps` build
on disk matches the config. Changes confined to `anton_mfc.xml` +
`anton_mfc_autopilot.xml`; controller C untouched. `bug-023` and `.wolf/memory.md`
logged. Task complete.

## FOLLOW-UP — MFC vertical guidance reads a never-updated reference (2026-06-10)

While running the config-time MFC->INDI stack (airframe `type="indi"` →
`STACK_USE_MFC_GUIDANCE=1`, MFC guidance active) in NPS via
`python3 sim_anton.py --fg --mfc`, the `SITL_PRINT("z_sp: %.3f")` in
`guidance_mfc.c:233` printed `0.000` every tick (and downstream
`th_cmd.z`/`v_thrust.z` were 0 too).

**Not a mode problem — NAV mode IS entering.** The print only fires from
`guidance_mfc_run_vert()`, which is only called from the NAV `<control>` block
(`anton_mfc_autopilot.xml:98`, `cond="STACK_USE_MFC_GUIDANCE"`). The print firing
at all proves NAV is active.

**Root cause:** `guidance_mfc_run_vert` reads its setpoint from
`gv->z_ref` (`guidance_mfc.c:226`). But `guidance_v.z_ref` (and `z_sp`) are only
ever populated as a side-effect of `guidance_v_run()` / `guidance_v_from_nav()` —
that's the function that reads `nav.nav_altitude`, writes
`guidance_v.z_sp = -POS_BFP_OF_REAL(nav.nav_altitude)`, then runs
`gv_update_ref_from_z_sp()` to fill `z_ref` (`guidance_v.c:319-324`). In the MFC
stack the autopilot *replaces* `guidance_v_run` with `guidance_mfc_run_vert`
(mutually exclusive on `STACK_USE_MFC_GUIDANCE`, autopilot lines 97 vs 98), so
nothing copies `nav.nav_altitude → z_sp → z_ref`. Both stay at their init value 0.
The MFC vertical axis tracks a constant-zero reference forever.

The flaw is structural: MFC vertical guidance depends on a buffer
(`guidance_v.z_ref`) that is *only* filled by the very function it displaced.

**Proposed fix (not yet applied):** source the setpoint from nav directly inside
`guidance_mfc_run_vert`, the way `guidance_v_from_nav` does —
`guidance_v.z_sp = -POS_BFP_OF_REAL(nav.nav_altitude); gv_update_ref_from_z_sp(...);`
then read `z_ref`; or bypass the ref model with `float z_sp = -nav.nav_altitude;`.
Needs `#include "firmwares/rotorcraft/navigation.h"` for `nav`. Horizontal axes
(`gh->ref.pos`) are likely fine — `guidance_h`'s reference is still updated — but
confirm once vertical works. Logged as a bug; user asked for diagnosis only this
turn, awaiting go-ahead to apply.

## DESIGN PIVOT — make MFC a "normal" guidance, drop the custom autopilot (2026-06-10)

User question: now that we never co-compile both stacks, why keep MFC guidance as a
bespoke `guidance_mfc_run_vert/horiz` pair + custom autopilot? Why not make it a
standard guidance variant like INDI so the stock control block works:

```xml
<call fun="guidance_v_run(autopilot_in_flight())" store="struct ThrustSetpoint thrust_sp"/>
<call fun="guidance_h_run(autopilot_in_flight())" store="struct StabilizationSetpoint stab_sp"/>
<call fun="stabilization_run(autopilot_in_flight(), &stab_sp, &thrust_sp, stabilization.cmd)"/>
```

**Answer: yes, idiomatic and better.** Key architecture finding (verified in source):

- `guidance_h.c`/`guidance_v.c` are **generic drivers** — they own mode logic, the
  reference models, AND the nav integration that fills `gv->z_sp/z_ref` and
  `gh->ref.pos`. They dispatch to six plug functions the active variant supplies:
  `guidance_v_run_pos`, `guidance_h_run_pos/_run_speed/_run_accel`,
  `guidance_h_run_enter`, `guidance_v_run_enter`.
- Every real variant defines exactly those: `guidance_pid.c`, `guidance_indi.c`
  (via `guidance_indi_run_mode`), `guidance_hybrid.c`, `guidance_oneloop.c`,
  `guidance_indi_hybrid.c`. e.g. `guidance_indi.c:586` `guidance_h_run_pos` →
  `guidance_indi_run_mode(...)`.
- The variants are **mutually exclusive by the module resolver**:
  `guidance_indi_base.xml:21` `<provides>guidance,attitude_command</provides>`;
  `guidance_hybrid.xml`, the hybrid tailsitter/quadplane all `provides guidance`.
  Stock `rotorcraft_autopilot.xml` / `rotorcraft_control_loop.xml` use the
  run_guidance_control block above.

**Why the bespoke design existed:** `guidance_mfc.xml` declares
`<provides>guidance_mfc</provides>` (NON-conflicting) on purpose, so it co-compiles
alongside `guidance_indi` — both linked, autopilot cond-picks at the call site. That
single choice is the entire reason for the custom autopilot,
`STACK_USE_MFC_GUIDANCE`, and the `_run_vert/_run_horiz` signatures.

**This refactor also deletes the z_sp=0 bug for free.** That bug exists *only*
because `guidance_mfc_run_vert` bypassed `guidance_v_run()` — the driver that copies
`nav.nav_altitude → z_sp → z_ref`. Plug in at `guidance_v_run_pos(in_flight, gv)`
and `gv->z_ref` is already populated. The manual nav-read fix becomes unnecessary.

**The one constraint:** a refactored `guidance_mfc.c` and `guidance_indi.c` both
define `guidance_h_run_pos` etc. → duplicate-symbol link error if co-compiled. So
MFC must become a real `provides>guidance` module, which forces the guidance choice
to be a config-time switch too. Since the stacks are always crossed, swap BOTH lines
together (still trivial):

| Stack | airframe |
|---|---|
| INDI→MFC | `guidance type="indi"` + `stabilization type="mfc"` |
| MFC→INDI | `guidance type="mfc"` + `stabilization type="indi"` |

**Custom autopilot then unnecessary** for stack reasons — stock
`rotorcraft_autopilot.xml` drives both. Only keep a local (much simpler) copy if the
trimmed mode set (KILL/FAILSAFE/ATTITUDE_DIRECT/NAV) + `MODE_STARTUP=NAV` is wanted;
none of the cond/`STACK_USE_MFC_GUIDANCE`/`guidance_mfc_enter` machinery survives.

**Proposed implementation order (not yet started, awaiting go-ahead):**
1. Convert `guidance_mfc.c/.h` to the six standard plug functions; move the
   nav-setpoint read out (driver does it now).
2. Rework `guidance_mfc.xml`: `provides guidance,attitude_command`, give it a `type`.
3. Update `anton_mfc.xml` to the paired 2-line switch.
4. Delete custom autopilot / point airframe at stock (or trim a simple copy).
5. Build both `nps` variants → confirm clean links.

Supersedes the compile-time 2×2 cleverage in the RESOLUTION section: we now accept
single-guidance-per-build and gain the stock control block + bug fix.

## IMPLEMENTED — refactor done + two bugs fixed (2026-06-10)

Executed the plan above in place (no worktree). Working tree was clean.

**Code changes:**
- `guidance_mfc.c`: removed `guidance_mfc_enter` / `guidance_mfc_run_vert` /
  `guidance_mfc_run_horiz`; added the six standard plug functions
  (`guidance_v_run_pos/speed/accel`, `guidance_h_run_pos/speed/accel`,
  `guidance_v_run_enter`, `guidance_h_run_enter`). Vertical step latches a
  file-static `mfc_thrust_sp` that the horizontal step consumes for its
  throttle→tilt scaling (loop order is v-then-h). All vertical/horizontal
  variants route to one `guidance_mfc_vert`/`guidance_mfc_horiz` that track
  `gv->z_ref` / `gh->ref.pos` (filled by the generic driver).
- `guidance_mfc.h`: dropped the bespoke prototypes; note that the runtime entry
  points are the generic plug functions from guidance_h.h/guidance_v.h.
- `guidance_mfc.xml`: `<provides>guidance,attitude_command</provides>` (was
  `guidance_mfc`); makefile adds `GUIDANCE_PID_USE_AS_DEFAULT=FALSE` so PID's
  default plug functions don't collide.
- `anton_mfc.xml`: STACK SELECT is now a paired 2-line swap
  (`guidance type` + `stabilization type` together); removed the standalone
  `<module name="guidance_mfc"/>` and the second `guidance type="indi"` line.
  Default = MFC->INDI (`guidance type="mfc"` + `stabilization type="indi"`).
- `anton_mfc_autopilot.xml`: kept (trimmed mode set still wanted) but stripped of
  all MFC machinery — removed the guidance_mfc.h include, `STACK_USE_MFC_GUIDANCE`
  defines, `guidance_mfc_enter` on_enter, and the cond= dual-guidance calls. NAV
  now uses a stock `run_guidance_control` block (guidance_v_run / guidance_h_run /
  stabilization_run).

**bug-028 (z_sp=0) — FIXED.** Root cause confirmed: bespoke `_run_vert` read
`gv->z_ref`, only ever filled by the `guidance_v_run()` it displaced. NAV *was*
entering. The refactor plugs in at `guidance_v_run_pos`, so the driver fills
`z_ref` first. NPS: z_sp now ramps 0 → -2.0 m.

**bug-029 (NaN throttle) — FIXED, surfaced by bug-028's fix.** Once z_sp was real,
throttle came out `-nan`. Cause: `mfc_core`'s shared `mfc.sample_time` was 0 →
`mfc_siso_run` divided by sample_time → inf/NaN. `mfc_core_init()` is only called
by `stabilization_mfc_init()` (and `guidance_indi_init` when THRUST_MFC); in the
MFC->INDI stack the MFC side is the *guidance*, so nobody initialised the core.
Fix: `guidance_mfc_init()` now calls `mfc_core_init(1.f / PERIODIC_FREQUENCY)`
(mirrors `guidance_indi.c:250`). The shared single `mfc_core` instance is safe
because the stacks are crossed — MFC is *either* stab *or* guidance, never both.

**Verified:** `build_fw.sh -c ANTON_MFC conf/airframes/ENAC/conf_enac.xml nps`
links clean for BOTH pairings (MFC->INDI and INDI->MFC). NPS run of the default
MFC->INDI shows finite throttle and z_sp tracking. CONF_XML must be relative to
the paparazzi dir (`conf/...`), NOT `paparazzi/conf/...` — the latter fails with
`File_not_found` (the generator runs with cwd=paparazzi).

**Left for the user (out of scope, not a code bug):** the MFC vertical throttle is
noisy/oscillating during the startup transient — untuned GZ estimator gains
(GZ_ALPHA=32.7, GZ_KP=0.5 are starting points). Tune after a stable hover.
The temporary `SITL_PRINT` debug line was removed after verification.

**Wrap-up (logs finalized):** bug-028 + bug-029 logged in `.wolf/buglog.json`;
`.wolf/memory.md` got 4 action lines; `.wolf/cerebrum.md` gained two Key Learnings
(the 6-function guidance plug seam; the single shared `mfc_core` instance + its
init requirement) and a Decision Log entry marking this refactor as superseding the
compile-time 2×2 / `<control_block>` swap. Final diffstat: 5 files in `paparazzi/`
(anton_mfc.xml, anton_mfc_autopilot.xml, guidance_mfc.xml, guidance_mfc.c/.h),
+123/-92. Both `nps` pairings link clean; tree otherwise matches the default
MFC->INDI config.

## FOLLOW-UP 2 — F_k=0, estimator windup, and an INDI thrust-units mismatch (2026-06-10)

After the refactor, user moved to the MFC->INDI vertical-throttle path and added a
debug edit in `stabilization_indi.c` (rate_run, else/absolute-thrust branch):
`v_thrust.z = (float)thrust->sp.thrust_f[THRUST_AXIS_Z];` (replacing the stock
`v_thrust.z += th_cmd.z * Bwls[3][i] * act_thrust_mat[2][i]`). Symptom:
`v_thrust.z` and `th_cmd.z` print 0; "estimator F_k of mfc_z is 0."

**bug-030 (F_k=0) — FIXED.** Instrumented `guidance_mfc_vert` (print mfc.time,
err, F_k, num, den, cmd). Smoking gun: `mfc.time` pinned at exactly 0.002 (one
sample period) every cycle. `mfc.time = get_sys_time_float() - mfc.start_time`,
and `mfc_core_start()` (sets start_time=now) was being called EVERY ground cycle
via the enter-hook chain: `guidance_h_from_nav()` calls `guidance_h_nav_enter()`
→ `guidance_h_run_enter()` **every cycle while `!in_flight`** (guidance_h.c:393).
My refactor had wired `mfc_core_start()` into BOTH `guidance_v_run_enter` and
`guidance_h_run_enter`. With mfc.time stuck at 0.002, the estimator gate
`mfc.time > 0.1` (mfc_core.c:83) never opened → F_k≡0 → zero MFC thrust → never
lifts. **Fix:** `guidance_h_run_enter()` is now a documented no-op; only
`guidance_v_run_enter()` starts the shared clock.

**bug-031 (windup to ±∞) — FIXED, surfaced by bug-030's fix.** With mfc.time free
to grow, F_k and cmd diverged (F_k: 0.1→3593, num→56322, cmd→-718 and climbing)
while still on the ground. Root cause: the algebraic estimator is effectively an
integrator — dominant terms give `F_k ≈ kp²·err − alpha·cmd[k-1]`, so
`cmd[k] ≈ cmd[k-1] + (ydd_ref − kp²·err)/alpha`. On the ground `z_meas≡0` while the
altitude reference ramps, so `err` never decays → integrator windup. `mfc_siso_run`
accepts `in_flight` but IGNORES it (mfc_core.c:95). NOTE the stabilizer survives
unbounded mfc.time because attitude error ≈ 0 (plant responds); the vertical
guidance on the ground does not. **Fix:** gate the estimator on `in_flight` in
`guidance_mfc_vert` — while `!in_flight`, re-latch the clock (mfc.time < 0.1 gate),
zero cmd history, and output `GUIDANCE_MFC_GZ_NOMINAL_HOVER_THROTTLE`; run MFC only
in flight, starting fresh at the takeoff transition. Result: `v_thrust.z` steady at
0.30 on the ground (not 0, not ∞).

**bug-032 (won't take off) — DIAGNOSED, user's call, NOT fixed.** With clean 0.30
throttle the vehicle still won't lift: the user's `v_thrust.z = thrust_f` edit feeds
a NORMALISED throttle (0.30) into `indi_v[3]`, but that WLS thrust objective lives in
PPRZ·effectiveness units. Stock mapping: `th_cmd.z=0.30·9600=2880`,
`Bwls[3][i]=g1_thrust/INDI_G_SCALING=−1.5/1000=−0.0015`, hover objective
`= Σ 2880·(−0.0015) ≈ −17.3`. So INDI needs `indi_v[3]≈−17.3`; the edit feeds +0.30
— ~58× too small AND wrong sign → WLS commands motors ~off → no takeoff → in_flight
never trips (the earlier windup masked this by brute-forcing the motors). It is NOT
newtons; nothing in the path converts to force. Options given to user: (1) revert to
stock `v_thrust.z += th_cmd.z*Bwls[3][i]*act_thrust_mat[2][i]` (recommended); or
(2) make guidance output the indi_v[3]-space value (~−17 at hover) and retune
GZ_ALPHA + sign. Awaiting user choice. Did NOT touch their stabilization_indi edit,
the `-mfc_gz.command[0]` negation, commented `Bound`, or gz gains (alpha=5,kp=4,
tt=250,iw=5). The verbose `SITL_PRINT` in guidance_mfc_vert is still in place (user
is actively debugging; only fires in-flight so currently silent).

INDI_G_SCALING = 1000.0 (stabilization_indi.h:31). bug-030/031/032 logged.

## Deferred (runtime/in-flight switching — not done)

The in-flight design below remains the plan IF live switching is ever wanted; it
still requires the global-namespacing port. The agreed approach was:

1. **Prereq step 0 (blocker):** mark the ~33 file-local collisions `static` in
   `stabilization_mfc.c`; rename only the ~7 header-exposed ones with an `mfc_`
   prefix (edit just `stabilization_mfc.c/.h`). Keeps upstream INDI / the rest of
   the fleet untouched. Verify the *current* single-stack MFC build still compiles
   before going further.
2. Make `stabilization_mfc.xml` additive (drop its `stabilization_attitude_quat_mfc.c`
   shim, keep the core; change `<provides>`); `anton_mfc.xml` declares both stabs.
   **INDI param section source: borrow `STABILIZATION_ATTITUDE_INDI` from
   `anton_indi_aruco.xml`** (proven gains for this airframe). Then build to confirm
   both cores link.
3. Then the `ctrl_stack` dispatcher + GCS settings + autopilot rewire (above).
