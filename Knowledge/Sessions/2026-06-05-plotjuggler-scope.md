# Session — 2026-06-05: PlotJuggler Scope Emitter and Architecture Redesign

## What changed
- **Created in-process NPS scope emitter** (`nps_scope.c/.h`): reads truth (fdm) + MFC/WLS controller globals at end of sim step, sends UDP/JSON to PlotJuggler
- **Initial MFC-hardcoded implementation:** guarded by `STABILIZATION_MFC_ROLL_ALPHA`, emitted MFC roll/pitch error, WLS priorities
- **Redesigned scope architecture:** firmware-registered variables via `NPS_SCOPE_VAR` macro
  - Added `nps_scope_var.h` shim: constructor-based registry on `USE_NPS`, no-op on real targets
  - Removed MFC guard: scope is now controller-agnostic
  - Firmware files register globals with one line: `NPS_SCOPE_VAR("name", &addr, type)`
  - Emits all registered vars as top-level JSON keys (slash names → PlotJuggler tree)
- **Telemetry path clarification:** Two independent streams:
  - (A) NPS truth: `nps_ivy_display` → Ivy @ 10 Hz (3*DISPLAY_DT)
  - (B) Firmware telemetry: `pprz_msg_send_*` @ UDP:4242 → server JSON/9870 (event-driven)
  - (C) **NEW:** In-process scope via UDP/JSON to PlotJuggler @ ~512 Hz (decim=2)
- **Cleanup:** Removed enac_paparazzi from git, added to .gitignore
- **Created `bebop-set.xml`** for FlightGear simulation
- **Further init-firewall debugging**

## Bugs fixed
- None new (previous FG bugs resolved in prior session)

## What I learned
- **NPS SYS_TIME_FREQUENCY defaults to 1000 Hz, NOT 2*PERIODIC_FREQUENCY.** PERIODIC_FREQUENCY is a makefile var, not a C #define, so sim steps at ~1 kHz (SIM_DT≈1 ms)
- **Build path confusion:** `build_fw.sh CONF_XML` is relative to `paparazzi/` dir, NOT `/workspace` — pass `conf/airframes/ENAC/conf_enac.xml` not `paparazzi/conf/...`
- **Constructor-based registry pattern:** `__attribute__((constructor))` runs at load time to populate scope registry before `nps_main`
- **WLS guard trick:** `#if !STABILIZATION_MFC_ALLOCATION_PSEUDO_INVERSE` evaluates true at preprocess time even when `TRUE` is set (not numeric in that context) — mirror the header guard verbatim
- Three-layer telemetry architecture: truth, firmware, and scope

## What's next
- Integration with ANTON_MFC and ANTON firmware builds
- Verify scope performance under load (CPU cost of registration + JSON serialization)
- Flight test with real aircraft if available
