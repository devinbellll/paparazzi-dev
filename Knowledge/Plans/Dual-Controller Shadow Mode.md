# Plan — Dual-Controller Shadow Mode (MFC alongside INDI)

**Goal:** compile and run **both** the MFC and INDI stacks simultaneously at the
guidance *and* stabilization levels. INDI (the trusted law) drives the motors; MFC
runs as a **shadow** — fully computed every tick but its `cmd[]` is discarded and
logged. This lets us validate that MFC's commands track what we'd expect, against a
flying aircraft, **before** ever giving it motor authority. Handover is the sibling
plan: `Dual-Controller Handover Mode.md`.

Date drafted: 2026-06-18. Target airframe: `conf/airframes/ENAC/quadrotor/anton_mfc.xml`.
Background: read `Knowledge/11 - In-Flight Controller Switching (Oneloop Pattern).md`
first — it explains the codegen and why this is harder than the oneloop ANDI/INDI switch.

---

## Why this is not free (the core problem)

Oneloop ANDI/INDI is one module, one symbol set, branch on an int. **MFC and INDI
are two separate modules whose symbols collide.** Confirmed collisions:

| Symbol | MFC definition | INDI definition | Kind |
|--------|----------------|-----------------|------|
| `stabilization_attitude_run()` | `stabilization_attitude_quat_mfc.c:8` | `stabilization_attitude_quat_indi.c:40` | strong dispatch |
| `set_rotorcraft_commands()` | `stabilization_mfc.c:923` | `stabilization_indi.c:978` | strong WEAK-override |
| `g1g2`,`g1g2inv`,`g1g2_pseudo_inv` | `stabilization_mfc.c:297,298,329,347` | `stabilization_indi.c:254,255,289,307` | data global |
| `actuators_pprz`,`act_is_servo`,`stab_thrust_filt`,`Bwls`,`thrust_bx_*`,`act_pref` | stabilization_mfc.c | stabilization_indi.c | data globals |
| `guidance_h_run_pos/speed/accel`,`guidance_v_run_*`,`guidance_*_run_enter` | `guidance_mfc.c:436,489…` | guidance_indi (base) | strong WEAK-override |

Plus the module system forbids two modules that both `<provides>commands</provides>`
(stabilization) or `<provides>guidance,attitude_command</provides>` (guidance).

**Therefore Phase 0 — de-confliction — is mandatory and is shared with the handover
plan.** Everything else builds on it.

---

## Phase 0 — De-conflict so both stacks link (shared prerequisite)

Recommended approach: **a thin "dual" wrapper module owns the contested singleton
symbols and dispatches to renamed entry points of each underlying controller.** This
mirrors the proven oneloop pattern (one outer symbol, internal branch) without
forcing a full rewrite of either law.

### 0a. Rename the contested *dispatch* symbols in each stack
- In `stabilization_attitude_quat_mfc.c`: rename `stabilization_attitude_run` →
  `stabilization_attitude_run_mfc`. Same for INDI → `…_run_indi`.
- Remove the strong `set_rotorcraft_commands` override from **both** stabilization
  `.c` files; the dual wrapper will own the single override (it needs to anyway, to
  pick which buffer reaches `commands[]`).
- In `guidance_mfc.c` and `guidance_indi.c`: rename the `guidance_h_run_*` /
  `guidance_v_run_*` strong hooks to `…_mfc` / `…_indi`. The dual guidance wrapper
  owns the real `guidance_h_run_*` hooks and dispatches.

### 0b. Namespace the contested *data* globals
For each colliding global in the MFC stack, give it a unique linkage name. Cheapest
mechanical route: `#define g1g2 g1g2_mfc` (etc.) at the top of the MFC `.c`/`.h`, or
make them `static` where they are not referenced cross-file. Audit with:
`nm` on the two `.o` files, or grep each global across `sw/airborne` for external use.
INDI keeps the original names (it is the incumbent). Verify nothing outside the MFC
`.c` reads the renamed globals (telemetry/log modules may — see `mfc_core.xml`,
`nps_scope_state.xml`).

### 0c. New module wiring
Create `conf/modules/control_dual_mfc_indi.xml` that:
- `<depends>` on the **files** of both stacks (compile both `.c` sets directly,
  bypassing the per-stack `<provides>commands</provides>` conflict by not selecting
  either stabilization module the normal way),
- `<provides>commands,guidance,attitude_command</provides>` (it is now the single
  provider),
- declares the matrix-size defines once (`WLS_N_U_MAX/N_V_MAX = 4`, valid for both).

In `anton_mfc.xml`, replace the "STACK SELECT" block (lines ~53–71) with a single
`<module name="control" type="dual_mfc_indi"/>`. Keep **both** parameter sections
(`STABILIZATION_MFC`, `STABILIZATION_ATTITUDE_INDI`, plus guidance equivalents).

### 0d. Init both
Wrapper `init()` calls `stabilization_indi_init()`, `stabilization_mfc_init()`,
`guidance_indi_init()`, `guidance_mfc_init()`. Watch for init order / shared-ABI
subscriptions (IMU, actuator feedback) registered twice — dedupe if both subscribe.

**Exit criterion for Phase 0:** `./pprz.sh build ANTON_MFC nps` links a single `.elf`
with both stacks present (verify both symbol sets via `nm`). No behavior change yet —
wrapper dispatches 100% to INDI.

---

## Phase 1 — Shadow execution

In the dual stabilization wrapper's `stabilization_attitude_run()`:
```c
// active law drives motors
stabilization_attitude_run_indi(in_flight, sp, thrust, stabilization.cmd);
// shadow law computes into a separate buffer, never committed
stabilization_attitude_run_mfc (in_flight, sp, thrust, mfc_shadow_cmd);
```
- `mfc_shadow_cmd[]` is a wrapper-owned buffer (size `COMMANDS_NB`).
- The dual `set_rotorcraft_commands()` override commits **only** INDI's
  `stabilization.cmd` to `commands[]` (shadow is inert).
- Do the same at guidance: run both guidance laws each tick; INDI's
  `StabilizationSetpoint` feeds the active stabilizer; MFC's setpoint is stored for
  logging only.

### Filter/state integrity (critical)
A shadow controller that never acts will see its own integrators/filters diverge
unless they are driven from **measured** state, not from its own (discarded) output.
- INDI/MFC attitude laws are incremental around measured rates → mostly fine, but
  any actuator-feedback term must be fed the **true committed** actuator state
  (`actuator_state_filt_vect`), not the shadow's intended commands. Wire the shadow's
  actuator-feedback input to the real actuators so its increments are computed about
  the actual operating point (this is exactly what `enter()` does on a real switch).
- Re-`enter()` the shadow law whenever the active law's operating point jumps, to keep
  the comparison meaningful.

---

## Phase 2 — Telemetry & validation

Add a telemetry message (extend `mfc_core.xml` / a new `DUAL_CTRL` msg) logging per tick:
- `cmd_indi[ROLL,PITCH,YAW,THRUST]` (committed) vs `cmd_mfc[…]` (shadow)
- the two guidance attitude setpoints (phi/theta/psi, thrust)
- residual `cmd_mfc - cmd_indi` per axis

Validate in NPS first, then flight:
- **Agreement band:** define an acceptable `|cmd_mfc - cmd_indi|` envelope in hover and
  in gentle maneuvers. MFC is expected to differ (different law) but must be bounded,
  same sign on disturbance rejection, no runaway.
- **No NaNs / saturation storms** in the shadow path.
- Plot in PlotJuggler via the existing NPS telemetry stream (see
  `Knowledge/08 - NPS Simulation Telemetry.md`).

This is the deliverable: confidence, with a flying aircraft, that MFC *would* have
done something sane — without risking the airframe.

---

## Touch points (checklist)

- [ ] `stabilization_attitude_quat_mfc.c`, `..._indi.c` — rename dispatch symbol
- [ ] `stabilization_mfc.c:923`, `stabilization_indi.c:978` — remove strong `set_rotorcraft_commands`
- [ ] MFC stack `.c/.h` — namespace colliding data globals (Phase 0b)
- [ ] `guidance_mfc.c`, `guidance_indi.c` — rename `guidance_*_run_*` hooks
- [ ] NEW `sw/airborne/firmwares/rotorcraft/control/control_dual_mfc_indi.c/.h` — wrapper
- [ ] NEW `conf/modules/control_dual_mfc_indi.xml`
- [ ] `conf/airframes/ENAC/quadrotor/anton_mfc.xml` — swap STACK SELECT → dual module
- [ ] `mfc_core.xml` or new telemetry msg — `DUAL_CTRL` shadow comparison
- [ ] Rebuild clean: `./pprz.sh rebuild ANTON_MFC nps` (module set changed)

## Risks
- **Code budget / ANTON Tawaki flash + CPU:** two full stacks at 1000 Hz. Profile the
  added tick cost in NPS; if the `ap` target overruns, gate the shadow behind a lower
  prescaler (e.g. shadow at 500 Hz) — acceptable for validation.
- **Shared-symbol audit misses** → silent wrong behavior. `nm` diff is mandatory.
- **Double ABI subscription** (IMU/actuator feedback) → dedupe in wrapper init.
- A successful compile is NOT correctness — NPS then flight test required.
