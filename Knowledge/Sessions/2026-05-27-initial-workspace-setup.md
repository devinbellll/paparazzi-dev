# Session — 2026-05-27: Initial Workspace Setup

## What changed
- Initial commit: saved everything outside the repos (paparazzi upstream, enac_paparazzi fork)
- Created `sim_anton.py` — NPS simulation launcher for ANTON airframe
- Built foundational NPS simulation monitor (`nps_monitor`) with full firmware telemetry observability
- Implemented pseudo-command reception in `sim_anton.py`
- Added simsitl stdout capture for firmware printf output

## Bugs fixed
- None (initial setup phase)

## What I learned
- Workspace structure: paparazzi upstream is source of truth; enac_paparazzi kept for reference only
- NPS telemetry requires monitoring the firmware output for debugging and verification
- Pseudo-command infrastructure needed early for control/guidance testing

## What's next
- VSCode build tasks and IDE integration (Makefile Tools)
- Testing ANTON firmware compilation and NPS simulation
