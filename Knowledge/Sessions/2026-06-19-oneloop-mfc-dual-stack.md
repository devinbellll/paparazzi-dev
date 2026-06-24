# 2026-06-19 — oneloop_mfc: complete parallel MFC stack + independent layer switching

## Goal
Run a complete MFC control stack (guidance + stabilization) in parallel with the
INDI stack at all times, switchable **independently per layer** at compile and
run time. Four reachable combos: guidance{INDI|MFC} × stabilization{INDI|MFC}.
All four controllers run every tick and stream data; only the selected pair
drives the aircraft. Replaces the previous stab-only `feat/shadow-handoff`
dual-wrapper (which relied on static-ifying MFC + `#ifdef` shadow branches in
INDI — fragile).

Branch: `feat/shadow-handoff` (paparazzi submodule). Built/verified target:
`ANTON_MFC nps`.

## What changed (architecture)
- **New `oneloop_mfc.{c,h}`** (`sw/airborne/modules/control_dual/`) — the entire
  MFC stack (attitude stabilizer + position guidance) in one self-contained TU
  with **private (file-local) globals**: own WLS instance, g1g2/Bwls, actuator
  buffers, filters. Merged from the tuned `stabilization_mfc.c` + `guidance_mfc.c`
  (math preserved verbatim). Public API: `oneloop_mfc_init`,
  `oneloop_mfc_attitude_enter/run`, `oneloop_mfc_guidance_enter/run`, shadow
  actuator-state accessors, and `bool oneloop_mfc_stab_active`. Because nothing is
  named like INDI's externs, it links beside stock INDI with **zero collisions**
  and **no INDI edits** and **no SHADOW #ifdefs**.
- **Stab wrapper** `control_dual_mfc_indi.c` — owns `stabilization_attitude_run`,
  routes between untouched `stabilization_indi_attitude_run` and
  `oneloop_mfc_attitude_run`. Selector `dual_ctrl_active` (GCS `StabCtrl`).
  Because INDI is now fully stock (no `STABILIZATION_INDI_SHADOW`), the INDI
  shadow branch is gone — INDI always integrates its own command; only MFC copies
  the active operating point when it is the shadow (documented asymmetry).
- **Guidance wrapper** `guidance_dual_mfc_indi.{c,h}` — owns the framework plug
  symbols `guidance_h/v_run_*`. Calls stock INDI via `guidance_indi_run_mode()`
  (compiled with `GUIDANCE_INDI_USE_AS_DEFAULT=FALSE` so it does NOT own the plug
  symbols → INDI untouched) and `oneloop_mfc_guidance_run()`. Selector
  `guidance_ctrl_active` (GCS `GuidanceCtrl`). MFC thrust is emitted in the active
  stabilizer's format (physical for MFC stab, pprz-scaled for INDI stab), keyed on
  `dual_ctrl_active`.
- **Telemetry**: kept `DUAL_CTRL` (id 194, stab compare); added `GUIDANCE_DUAL`
  (id 195, guidance compare: active + both att setpoints + both thrusts). NPS
  scope: `dual/*`, `guid_dual/*`, `mfc/*` (re-homed), `mfc_g/*` (re-homed).
- **Wiring**: `anton_mfc.xml` guidance → `type="dual_mfc_indi"`; `conf_enac.xml`
  ANTON_MFC `settings_modules` now lists `oneloop_mfc.xml` +
  `guidance_dual_mfc_indi.xml` (dropped the superseded standalone panels).
- **Reverts**: `stabilization_mfc.{c,h}` restored to the clean pre-shadow
  (`feature-mfc-thrust`) version — no longer compiled in the dual build, kept
  valid for standalone `type="mfc"`. `stabilization_indi.c/.h` left as-is: the
  dormant `#ifdef STABILIZATION_INDI_SHADOW` code is never enabled, so INDI is
  functionally stock; the harmless `actuator_state` export (used by the wrapper)
  stays.

## Verified
- `./pprz.sh rebuild ANTON_MFC nps` → `simsitl` linked OK.
- `nm simsitl`: exactly one `T stabilization_attitude_run`, `guidance_h_run_pos`,
  `guidance_v_run_pos`; all four controllers present
  (`stabilization_indi_attitude_run`, `oneloop_mfc_attitude_run`,
  `guidance_indi_run_mode`, `oneloop_mfc_guidance_run`); selectors
  `dual_ctrl_active`, `guidance_ctrl_active`, `oneloop_mfc_stab_active` present.
- Sim boots and runs 6 s with no crash/NaN/assert (JSBSim ANTON model, autopilot,
  IVY all up).

## Build gotchas hit (logged bug-114..117)
- `MFC_G_SCALING` moved out of `stabilization_mfc.h` → redefined locally.
- Module carrying a settings panel must include the header declaring those vars
  (guidance_indi.h) or settings.h won't compile.
- `dl_setting handler="X" module=".../foo"` generates `foo_X` — function name must
  match exactly.
- Editing pprzlink `messages.xml` needs an explicit regen (bug-096). pprzlink is a
  submodule, so the normal `pprzlink_protocol` path (`*.update` →
  `git submodule update`) would REVERT the edit. Use instead:
  `pprz_run -- make -C sw/ext/pprzlink pymessages MESSAGES_INSTALL=/workspace/paparazzi/var PPRZLINK_LIB_VERSION=2.0 VALIDATE_XML=FALSE`

## Remaining / next
- **Autopilot-mode switching not yet wired** (the user asked for "both" modes +
  GCS settings). The GCS `StabCtrl` / `GuidanceCtrl` dropdowns work now. To add
  mode-based selection: enable `<autopilot name="anton_mfc_autopilot.xml"/>` in
  the airframe and add modes whose `<on_enter>` call
  `control_dual_mfc_indi_set_active(...)` + `guidance_dual_mfc_indi_set_active(...)`.
- **Functional flight test** (the real correctness gate): in NPS, fly each of the
  four combos, confirm bumpless switching (no command step) and that INDI→INDI
  matches the pure-INDI baseline; compare laws in PlotJuggler via `dual/*` and
  `guid_dual/*`.
- **MFC→INDI thrust scale**: `GUIDANCE_MFC_THRUST_PPRZ_SCALE` (default 1) likely
  needs tuning for the guidance_mfc→stab_indi combo.
- `mfc_thrust_physical = -12` hardcode preserved from the tuned guidance_mfc.c —
  revisit when enabling MFC vertical guidance properly.
