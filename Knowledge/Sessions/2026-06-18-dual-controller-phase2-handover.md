# Session — 2026-06-18: Dual-Controller Phase 2 (runtime handover MFC ↔ INDI)

## Goal
Make the dual stack able to give motor authority to **either** MFC or INDI, and
switch it **in flight via a GCS command** (Handover plan, Option A). Builds on
Phase 0 (co-compile) + Phase 1 (shadow telemetry/fidelity).

## What changed

### Command routing — the key realisation
The motors are driven by **two** paths, and a handover must serve both:
- **Hardware/ap:** `cmd[]` → `set_rotorcraft_commands()` → `commands[]`. INDI &
  MFC both write `cmd[act_to_commands[i]]` (STABILIZATION_*_COMMANDS defined).
- **Sim/nps:** `ANTON_MFC` has `NPS_NO_MOTOR_MIXING=TRUE` and **no** `NPS_USE_COMMANDS`,
  so `nps_autopilot_rotorcraft.c` drives the JSBSim motors from the global
  `actuators_pprz[]` directly (NOT `commands[]`). INDI writes that global; MFC
  writes a renamed static `mfc_actuators_pprz[]`.

So the wrapper, when MFC is active, copies MFC's `cmd` into the real `cmd[]` AND
copies `mfc_actuators_pprz[]` into the global `actuators_pprz[]`.

### Symmetric operating-point feedback
Both laws are incremental (Δu about a modelled `actuator_state`). The ACTIVE law
must integrate its OWN committed command (real operating point); the SHADOW law
must COPY that point. Made both directional:
- `stabilization_mfc.c`: runtime `stabilization_mfc_shadow_mode` (default TRUE);
  `get_actuator_state()` copies the fed point when shadow, else integrates `mfc_u`.
  Added getters `stabilization_mfc_get_actuator_state/_actuators_pprz`.
- `stabilization_indi.c`: new `STABILIZATION_INDI_SHADOW`-gated
  `stabilization_indi_shadow_mode` + `stabilization_indi_set_shadow_actuator_state()`
  + a shadow-copy branch in `get_actuator_state()`. Gated → stock INDI airframes
  unaffected.

### Wrapper selector (control_dual_mfc_indi.c)
- Separate `indi_cmd[]` / `mfc_shadow_cmd[]` buffers; both laws run every tick.
- `control_dual_mfc_indi_set_active(want)`: bumpless — flips the two shadow-mode
  flags and re-`enter()`s the newly active law (oneloop-style re-linearisation).
- Run order = active law first, so its fresh operating point is fed to the shadow.
- **Failsafe:** edge-triggered RC-really-lost → force INDI (edge, not level, so the
  GCS selector still works in NPS where there is no RC link).

### GCS command
`<settings>` block in `stabilization_dual_mfc_indi.xml` →
`dl_setting var="dual_ctrl_active" handler="set_active" module=".../control_dual_mfc_indi"`.
The GCS sends datalink **SETTING (msg id 4)**: index, ac_id, value.
For ANTON_MFC the index is **47** (settings.h `case 47`). value 0=INDI, 1=MFC.
In the GCS this is the **DualCtrl → active_law** dropdown.

## Gotchas hit (logged: bug-072/073/074)
- **settings_modules whitelist (bug-072):** a module's `<settings>` are only
  collected if the module file is listed in the aircraft's `settings_modules=`
  attribute in `conf_enac.xml`. Had to add `modules/stabilization_dual_mfc_indi.xml`.
  Symptom: handler never generated, no GCS setting, despite codegen running.
- **DTD order (bug-073):** module.dtd is `(doc,settings_file*,settings*,dep?,…)`
  — `<settings>` must follow `<doc>`, before `<dep>`.
- **Incremental codegen misses module-XML changes:** keys off the airframe/conf
  hash. After editing a module XML, `touch` the airframe XML to force regen.
- **CONF="" from container (bug-074):** `pprz_run` injects `-e CONF=""`; use
  `os.environ.get("CONF") or default` in sim scripts.

## Verification
- `./pprz.sh build ANTON_MFC nps` and `ap` both link. nm shows all new symbols
  (`control_dual_mfc_indi_set_active`, both `*_shadow_mode`, getters/setters).
- **NPS live-switch test** (tmp `handover_test.py`): takeoff, then GCS SETTING
  idx47 → 1 → 0. `DUAL_CTRL.active` went **0 → 1 → 0**. No NaN, no assert/segfault.
  When MFC active, committed `cmd_act == cmd_shd` (committed = MFC); when INDI
  active they differ by the small agreement band. Commands stayed continuous
  (~1880/9600 hover band) across both transitions → bumpless.

## Bumplessness — quantitative result (NPS, hover)
Continuous 10 Hz DUAL_CTRL capture around both transitions (committed `cmd_act`):
- **→ MFC** @18.61s: last INDI `[1883,1882,1888,1879,1882]` → first MFC
  `[1882,1882,1882,1882,1882]` (per-motor Δ ≤6). Worst step in ±1.5 s window = 21
  counts, occurring at t=17.41 — i.e. ordinary hover jitter *before* the switch.
- **→ INDI** @24.61s: last MFC `[1883,1882,1882,1882,1883]` → first INDI
  `[1884,1883,1880,1883,1881]` (Δ ≤2). Worst window step = 5 counts, also away
  from the switch.
- The step *at* the switch is within hover jitter (~0.2 % of MAX_PPRZ=9600). The
  uniform `[1882×5]` right after the MFC switch is the `enter()` re-linearisation
  landing on hover trim before attitude differential resumes → confirmed bumpless.

## Next
- Exercise the switch during translation / a small attitude command, not just hover.
- Optionally add the RC AUX gate + one-way panic from the Handover plan.
- Flight test only after shadow-band validation.
