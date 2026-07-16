# Session — 2026-07-16: MFC conf split + flight-plan rework

## What changed

- **`paparazzi/conf/airframes/ENAC/conf_enac.xml`** reverted to pre-MFC state: removed the 4
  MFC-era aircraft entries (`ANTON_MFC`, `ANTON_DUAL`, `ANTON_ONELOOP`, `Hoops_111_MFC`). MFC
  aircraft now live solely in `paparazzi/conf/userconf/ENAC/conf_mfc.xml`.
- Deleted the two redundant/broken anton flight plans:
  - `anton_mfc_attitude.xml` and `anton_dual_ctrl_test.xml` shared the same header/comment
    block and internal name (`"Anton Dual Ctrl Test"`), clearly a copy/paste fork.
  - **Bug found**: `anton_dual_ctrl_test.xml` had an extra "Geo init" block that shifted
    "Start Engine"/"Takeoff" to indices 3/4, but `sim_anton.py`'s `takeoff_sequence()` always
    sends `JUMP_TO_BLOCK` for hardcoded indices **2** and **3** (`sim_anton.py:389-391`). That
    file would have jumped to the wrong block on every sim takeoff.
- Added two new flight plans, both preserving the block-2/3 contract:
  - **`anton_mfc_attitude_direct.xml`** (→ `ANTON_MFC`): attitude/altitude-only blocks, no GPS
    wait, no `<stay wp=.../<go wp=...>` — usable under ATTITUDE_DIRECT / ATTITUDE_Z_HOLD without
    a GPS fix. This is the plan used to test all 5 NPS rc_scripts.
  - **`anton_mfc_nav.xml`** (→ `ANTON_DUAL`, `ANTON_ONELOOP`, `Hoops_111_MFC`): generic
    NAV-mode waypoint triangle, stripped of the `stabilization_dual_mfc_indi_set_active()` calls
    (ANTON_ONELOOP/Hoops_111_MFC don't build that module — would have failed to link). Live
    INDI/MFC switching for ANTON_DUAL is still available at runtime via `sim_anton.py`'s
    `switch indi|mfc` / `pprz_ctrl.py`, no flight-plan block needed.

## Verification

- Built `ANTON_MFC` (ap + nps), `ANTON_DUAL` (nps), `ANTON_ONELOOP` (nps), `Hoops_111_MFC` (nps)
  against `CONF=conf/userconf/ENAC/conf_mfc.xml` — all produced a `.elf` cleanly.
- Built `ANTON` (ap) against the default (reverted) `conf_enac.xml` — confirms the revert didn't
  break the main fleet conf.
- Ran `./sim.sh ANTON_MFC --rc_script N --render` for N = 0, 1, 2, 3, 4, 5 (30-40s windows). Each
  reached `CMD: BLOCK 3 (Takeoff)` with no crashes/errors; rc_script 5's window covered its own
  ~15s AUTO2→AUTO1 timer with no issues.
- Telemetry (`NPS_RATE_ATTITUDE`/`NPS_POS_LLH` via Ivy) stayed "stale" throughout every run —
  this looks like a pre-existing sim-harness telemetry-subscription gap unrelated to this
  change (JUMP_TO_BLOCK commands clearly reach the airborne side; the render dashboard's
  attitude/position readout just never updates). Worth a follow-up session if PlotJuggler/GCS
  telemetry needs to be trusted for real analysis.

## Learnings

- `sim_anton.py`'s takeoff sequence hardcodes block indices 2 and 3 for "Start Engine"/"Takeoff"
  — any flight plan wired to an aircraft launched via `sim_anton.py`/`sim.sh` must keep exactly
  two blocks before those (regardless of block names).
- A single `<aircraft>` conf entry only has one `flight_plan=` attribute — the `NPS_FLIGHT_PLAN`
  define mentioned in the old `anton_dual_ctrl_test.xml` comment was never actually wired up
  anywhere in this repo (grepped `sw/` and `conf/` — no matches), so ap and nps builds always
  share one flight plan per aircraft today.

## Update — removed ANTON_DUAL entirely

Standalone MFC (`ANTON_MFC` / `ANTON_ONELOOP`, both backed by `oneloop_mfc`) is stable, so the
parallel INDI+MFC dual-controller stack is no longer needed — if in-flight switching is ever
wanted again, the plan is to reuse `oneloop_mfc` directly, not resurrect a wrapper module. Removed:

- `paparazzi/sw/airborne/modules/control_dual/{guidance,stabilization}_dual_mfc_indi.{c,h}` and
  the now-empty `control_dual/` dir.
- `paparazzi/conf/modules/{guidance,stabilization}_dual_mfc_indi.xml`.
- `paparazzi/conf/airframes/ENAC/quadrotor/anton_dual.xml`.
- The `ANTON_DUAL` `<aircraft>` entry in `paparazzi/conf/userconf/ENAC/conf_mfc.xml`.
- `ANTON_DUAL (ap)/(nps)` entries in `.vscode/c_cpp_properties.json`.
- `DUAL_CTRL` (id 194) / `GUIDANCE_DUAL` (id 195) messages from the pprzlink submodule's
  `messages.xml` (needs `make pprzlink_protocol`-equivalent regen — see
  `[[mfc-flight-test-enablement]]`-style note in cerebrum — before the next build touches
  telemetry headers).
- Dual-controller support in `sim_anton.py` (`--switch-after`, `switch` stdin command,
  `on_dual_ctrl`/`DUAL_CTRL` subscription, dead `RED` color) and `pprz_ctrl.py` (`switch`
  subcommand).
- Stale doc comments in `anton_mfc.xml`, `anton_oneloop.xml`, `anton_mfc_nav.xml`, and
  `conf/modules/oneloop_mfc.xml` that pointed at the now-gone dual wrappers; dropped the
  vestigial `dir="control_dual"` attribute on the `oneloop_mfc` module (unused — its `<file>`
  elements already set explicit `dir=`).

Left untouched on purpose: `oneloop_mfc.c/.h`'s shadow-mode API (`oneloop_mfc_stab_active`,
`oneloop_mfc_set_shadow_actuator_state()`) — it's generic infrastructure, not dual-specific, and
touching the tuned 1600-line stable control file for comment-only cleanup wasn't worth the risk.
Also left the historical `Knowledge/Sessions/`, `Knowledge/Plans/`, `.wolf/buglog.json`, and
`.wolf/cerebrum.md` entries that mention `ANTON_DUAL` — they're debugging history, not live
config, and remain useful record of why the dual stack behaved the way it did.

**Not yet done:** regenerate pprzlink headers (`pprz_run -- make -C sw/ext/pprzlink pymessages
MESSAGES_INSTALL=/workspace/paparazzi/var PPRZLINK_LIB_VERSION=2.0 VALIDATE_XML=FALSE`) and rebuild
`ANTON_MFC`/`ANTON_ONELOOP` to confirm nothing else referenced the removed messages/modules.
