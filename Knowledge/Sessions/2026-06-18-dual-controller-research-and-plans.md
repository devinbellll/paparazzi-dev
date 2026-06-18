# 2026-06-18 — Dual-controller research & plans (MFC alongside INDI)

## Goal
Investigate the "two controllers running at once, switch in flight to validate a new
controller" claim, trace how it actually works in Paparazzi, and draft plans to run
MFC alongside INDI at guidance + stabilization levels.

## What changed (docs only — no code)
- **NEW** `Knowledge/11 - In-Flight Controller Switching (Oneloop Pattern).md` — end-to-end
  trace of the autopilot XML → `gen_autopilot.ml` codegen → runtime mode machine, the
  oneloop ANDI/INDI switch, and the `WEAK set_rotorcraft_commands()` chokepoint.
- **NEW** `Knowledge/Plans/Dual-Controller Shadow Mode.md`
- **NEW** `Knowledge/Plans/Dual-Controller Handover Mode.md`
- Updated `00 - Index.md`, `.wolf/anatomy.md`, `.wolf/memory.md`, `.wolf/cerebrum.md`.

## Key findings
1. **The autopilot XML is a codegen input, not runtime config.** `gen_autopilot.ml`
   turns `<call fun>`/`store=`/`cond=`/`<control_block>`/`<call_block>`/`<control freq>`/
   `<select>`/`<on_enter>` into `autopilot_core_ap.h`. Documented the full mapping table.
2. **"Switch controllers in flight" = ONE law at a time, re-linearized on entry — not a
   parallel shadow.** Oneloop (`rotorcraft_oneloop_switch.xml`) is a single module with
   ANDI+INDI sharing state; flip `oneloop_andi.ctrl_type` and re-`enter()` (rebuilds G via
   `G1G2_oneloop()`) for bumpless handover. Triggered by `RCMode2() && RCAP2()/RCAP0() &&
   DLMode…()`; `AP_MODE_SWITCH = RADIO_AUX7` on the rotwing airframe.
3. **MFC vs INDI is materially harder than oneloop:** two separate modules with COLLIDING
   globals — `stabilization_attitude_run()` (strong: `..._quat_mfc.c:8` vs `..._indi.c:40`),
   `set_rotorcraft_commands()` (strong override: `stabilization_mfc.c:923` vs
   `stabilization_indi.c:978`), plus `g1g2`/`actuators_pprz`/`act_is_servo`/`stab_thrust_filt`/
   etc., and both `<provides>commands</provides>` / `guidance,attitude_command`. They cannot
   naively co-compile/link.
4. Both plans therefore share **Phase 0 — de-confliction** via a thin dual wrapper module
   owning the contested singletons + namespaced data globals + renamed dispatch entry points.

## Decisions
- Kept everything as **plans only** — no code written this session (user's call).
- Recommended sequencing: Shadow Phase 0+1 → validate MFC in shadow → promote to active
  via Handover. Recommended Handover Option A (selector inside wrapper) over Option B
  (full two-mode autopilot XML) for lower codegen risk.

## Next
- When ready to implement: start Phase 0 — create `control_dual_mfc_indi` wrapper module,
  rename colliding dispatch symbols, namespace MFC data globals, verify single `.elf`
  links with both symbol sets (`nm` diff), wrapper dispatches 100% to INDI (no behavior
  change). Then Shadow Phase 1.
- Watch: ANTON Tawaki flash/CPU budget for two warm stacks at 1000 Hz; double ABI
  subscriptions in wrapper init.
