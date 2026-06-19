# Session — 2026-06-18: Dual-Controller Phase 0 (MFC + INDI co-compile)

## Goal
Assess the two plans in `Knowledge/Plans/` (Shadow Mode, Handover Mode) and start
implementing.

## Strategy decided
The two plans are **sequential phases of one effort**, not alternatives — both
reuse Phase 0 (de-confliction) and Shadow is the safe validation gate for
Handover. Chosen order: **shared Phase 0 → Shadow → Handover (Shadow-first)**,
Handover via Option A (wrapper selector + GCS/RC). Implemented Phase 0 at the
**stabilization level** first as the smallest verifiable milestone (guidance left
as stock INDI), giving a pure-INDI active path with the MFC stabilizer shadowing.

## What changed
- **stabilization_mfc.c** — the 39 strong globals that collide with
  stabilization_indi.c made file-local (`static`); `actuators_pprz` renamed
  `mfc_actuators_pprz` (it must stay exported for the NPS glue, but as INDI's);
  `set_rotorcraft_commands` renamed `static mfc_set_rotorcraft_commands`
  (INDI owns the single commit point); MFC NPS scope keys `wls/*` → `mfc_wls/*`.
- **stabilization_mfc.h** — removed the 8 now-dangling colliding externs.
- **NEW modules/control_dual/control_dual_mfc_indi.{c,h}** — the dual wrapper:
  owns `stabilization_attitude_run`/`_enter`; runs INDI active (drives motors) +
  MFC shadow (into inert `mfc_shadow_cmd[]`) every tick; `stabilization_dual_init`
  inits both cores. Header declares only `extern int16_t actuators_pprz[]` for the
  NPS glue (NOT all of indi.h — that leaks externs into the MFC TU via modules.h).
- **NEW conf/modules/stabilization_dual_mfc_indi.xml** — compiles both stabilizer
  cores + wrapper (NOT the quat_*.c dispatchers); provides `commands`; defines
  INDI + MFC matrix sizes. Selected with `<module name="stabilization" type="dual_mfc_indi"/>`.
- **anton_mfc.xml** — stabilization module → `dual_mfc_indi` (guidance stays `indi`).

## What I learned (see cerebrum)
- bug-039 is moot: stabilization_mfc.c is self-contained today; the problem is
  pure link-time duplicate symbols.
- `generated/modules.h` is included by the stabilizer .c files, so a wrapper
  module's `<header>` leaks whatever it includes into every control TU — must
  declare only the needed symbol (bug-068).
- `set_rotorcraft_commands` IS compiled in both stabilizers for ANTON
  (`STABILIZATION_*_COMMANDS` is generated from the command map) (bug-067).

## Verification
`./pprz.sh rebuild ANTON_MFC nps` links one `simsitl`. `nm` confirms both
stabilizer cores (`stabilization_{indi,mfc}_attitude_run`, `_init`, `_enter`),
the single wrapper dispatch (`stabilization_attitude_run`, `stabilization_dual_init`),
one `set_rotorcraft_commands` (INDI's), and the namespaced MFC globals
(`mfc_actuators_pprz`, `mfc_shadow_cmd`). **Phase 0 exit criterion met.**
Compile is the only automated check; behaviour = pure-INDI (no change) by design.

## Next
- **Phase 1 (finish):** add a `DUAL_CTRL` telemetry message logging
  `cmd_indi[] vs mfc_shadow_cmd[]` + per-axis residual; validate the agreement
  band in NPS, then flight. Wire the shadow MFC's actuator-feedback input to the
  *real* committed actuator state so its increments are about the true operating
  point (re-`enter()` on operating-point jumps).
- Smoke-test simsitl init (watch for double ABI/telemetry registration from
  running both `*_init`).
- (Optional) guidance-level shadow (MFC guidance alongside INDI guidance).
- **Phase 2 — Handover:** Option A selector (`dual_ctrl_active`,
  `dual_ctrl_set_active()` re-`enter()`s the newly active law), GCS dropdown +
  RC AUXn gate, failsafe forces INDI.
