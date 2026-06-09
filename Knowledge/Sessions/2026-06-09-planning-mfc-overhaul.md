# Session — 2026-06-09: Planning MFC Overhaul

## What changed
- **Created 6 retroactive session documents** covering full project history from 2026-05-27 through 2026-06-08
  - 2026-05-27: Initial workspace setup (NPS sim, simsitl integration)
  - 2026-05-28: VSCode build tasks and IDE integration
  - 2026-06-01: Devcontainer freeze tool and VSCode debugging
  - 2026-06-04: FlightGear integration and OpenWolf initialization (4 bugs fixed)
  - 2026-06-05: PlotJuggler scope emitter and firmware-registered vars refactor
  - 2026-06-08: INDI guidance and ANTON_MFC_THRUST airframe config
- **Re-assessed `lets-move-on-from-floofy-scone.md`** and wrote a cleaner plan to
  `~/.claude/plans/re-assess-the-plan-...composed-squid.md` for a complete MFC stack on
  ANTON_MFC (MFC position guidance → MFC attitude stabilization), plus a custom minimal
  autopilot. No code written — planning only (plan mode).
- **Explored** the MFC core (`mfc_core.c/h`, `MfcParameters`, `mfc_siso_run`),
  `stabilization_mfc.c`, the autopilot state-machine system (`rotorcraft_autopilot.xml` vs
  `rotorcraft_oneloop.xml`, `autopilot.dtd`), the guidance layer (`guidance_indi.c`, setpoint
  constructors), and `sim_anton.py`'s takeoff path.

## What I learned
- **`stabilization_mfc_attitude_run()` currently ignores its attitude setpoint** —
  `stabilization_mfc.c:792-794` hardcode `mfc_*.setpoint = 0.0` with the real
  `stab_sp_to_eulers_f()` conversion commented out. MFC stabilization regulates to *level*,
  not to commanded attitude. Any guidance feeding it does nothing until this is uncommented.
  **This is the load-bearing fix for the whole MFC-guidance effort.**
- **Autopilot transparency lives in the XML, not C.** `<control_block>` with
  `<call fun=.. store=..>` writes the layer sequence and the setpoints passed between layers
  (`thrust_sp`, `stab_sp`) directly in the autopilot XML. The `oneloop` family collapses this
  into one `oneloop_from_nav()` C call — flat XML but opaque per-layer. We chose explicit
  blocks for debuggability.
- **Static INDI↔MFC switch via two control_blocks.** Compile both stacks into `anton_mfc.xml`
  (both guidance modules + both tuning sections); NAV calls one `<call_block>`. Swap = one
  line + rebuild. No GCS/runtime mode toggling needed (user explicitly de-scoped live switching).
- **`guidance_mfc` must `<provides>guidance_mfc</provides>`, NOT the generic `guidance`** — so
  it coexists with `guidance_indi` instead of replacing it (guidance_indi uses
  `GUIDANCE_INDI_USE_AS_DEFAULT` to override `guidance_h_run`/`guidance_v_run`).
- **Barebones test = NAV mode only.** `sim_anton.py` sends BLOCK 3 (Start Engine /
  NavResurrect) → BLOCK 4 (Takeoff, derouts to Standby at z>2.0) → BLOCK 5 (Standby hover),
  all flight-plan blocks run in NAV. Standby hover is the steady-state to tune/observe.
  Minimal autopilot needs only ATTITUDE_DIRECT / NAV / FAILSAFE / KILL.
- **Naming:** `accel_to_angle` → `accel_to_att_sp` (user choice).

## What's next
- **Implement the plan, ideally in a fresh session** (plan file is self-contained; build/debug
  loops are the token sink). Stage it: (1) `mfc_core` enable flag + `stabilization_mfc.c`
  setpoint fix → (2) `guidance_mfc` module → (3) autopilot + airframe wiring → (4) Knowledge
  doc `10 - Simplified Autopilots & Flight Modes.md`. Build between stages.
- **Heed the 3-files-per-task rule** in `paparazzi/CLAUDE.md` — the plan touches 8 files, so
  stage commits.
- **Validate `accel_to_att_sp` sign convention** against ANTON's `BODY_TO_IMU_PSI=-45°` in NPS
  before any real flight.