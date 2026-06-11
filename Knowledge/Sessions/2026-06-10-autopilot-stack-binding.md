# Session: 2026-06-10 — Autopilot ↔ stabilization/guidance binding (Q&A)

No code changed. Clarified how `anton_mfc_autopilot.xml` selects control stacks,
and how to make the stabilizer swappable the way guidance already is.

## Key findings

- **Selection is link-time symbol binding, not config dispatch.** Autopilot
  `<call fun="...">` lines call fixed names; the airframe `<module>` decl decides
  which `.c` *provides* each symbol.

- **Stabilization** has one generic entry: `stabilization_run()` (in
  `stabilization.c`) → dispatches by mode → `stabilization_attitude_run()`, which
  exactly ONE stab module may define.
  - `type="indi"` → `stabilization_attitude_quat_indi.c` defines it.
  - `type="mfc"` → `stabilization_attitude_quat_mfc.c` defines it (thin shim into
    `stabilization_mfc_attitude_run()`).
  - Two providers = duplicate-symbol link error → only one stab per build. That's
    why both `run_indi_stack`/`run_mfc_stack` feed the same `stabilization_run` and
    therefore always use MFC stab. The two blocks differ *only* in guidance.

- **Guidance MFC deliberately coexists.** `guidance_mfc.xml` doc says: *"Designed
  to coexist with guidance_indi in the same build; the autopilot XML picks the
  active stack."* It uses uniquely-named `guidance_mfc_run_horiz()` /
  `guidance_mfc_run_vert()` (not the standard `guidance_h_run()` /
  `guidance_v_run()`), so both INDI and MFC guidance link in and the
  `<control_block>` chooses at the call site.

- **Asymmetry:** guidance is swappable from autopilot XML; stab is not — because
  MFC stab *hijacks* the standard `stabilization_attitude_run` symbol instead of
  taking a unique name.

## Recommended refactor for full 2×2 mix-and-match

Mirror the guidance coexistence pattern for the stabilizer:

1. `stabilization_attitude_quat_mfc.c` — stop defining `stabilization_attitude_run()`;
   expose uniquely-named `stabilization_mfc_run(in_flight, sp, thrust, cmd)`
   (can just call the existing `stabilization_mfc_attitude_run()`).
2. `stabilization_mfc.xml` — make additive: drop `STABILIZATION_ATTITUDE_TYPE_H` /
   `STABILIZATION_ATTITUDE_TYPE_INT` defines and `<provides>commands</provides>`.
3. `anton_mfc.xml` — `<module name="stabilization" type="indi">` (provides
   `stabilization_run`) + additive `<module name="stabilization_mfc"/>`.

Then autopilot control blocks pick:
- one stab call: `stabilization_run(...)` (INDI) vs `stabilization_mfc_run(...)` (MFC)
- one guidance call per axis: `guidance_v_run()`/`guidance_h_run()` vs
  `guidance_mfc_run_vert()`/`guidance_mfc_run_horiz()`

H and V are independently mixable since guidance_mfc already splits them. Switching
stays a one-line edit in the NAV `<control>`.

**Caveat:** both stabs pull in WLS and want `WLS_N_U_MAX/N_V_MAX`; compiling both
means carrying both `STABILIZATION_MFC_*` and `STABILIZATION_INDI_*` sections and
reconciling WLS sizing. That's the real cost of the 2×2.

**Lighter alternative:** keep one stab per build, select via two airframe files
(`anton_mfc.xml`, `anton_indi.xml`) that differ only in the stab module line and
share everything else through XML `<include>`. No live A/B in one flight, but zero
symbol/WLS entanglement.

## What is next

- Not implemented — user was asking for clarification + design direction. If we
  proceed, do it as two commits (module refactor, then airframe+autopilot wiring)
  given the `paparazzi/CLAUDE.md` 3-files-per-task cap (this touches 4 files).
