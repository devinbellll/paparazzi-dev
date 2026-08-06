# Paparazzi Control System — Knowledge Base

Index of all notes. Designed for Obsidian; internal links use `[[Note Name]]` style.

## Structure

| Note | What it covers |
|------|---------------|
| [[01 - Control System Architecture]] | The full signal chain: Nav → Guidance → Stab → Actuators |
| [[02 - INDI Stabilization Deep Dive]] | How INDI works, all source files, key variables |
| [[03 - INDI Guidance Deep Dive]] | How guidance_indi sits above stabilization |
| [[04 - Airframe XML Configuration]] | XML knobs that drive both systems |
| [[05 - Module System]] | How modules.xml → Makefile → compiled firmware |
| [[06 - Modifying ANTON Stabilization]] | Practical walkthrough for ANTON |
| [[07 - All Touch Points Cheatsheet]] | Quick reference: every file to touch per change type |
| [[08 - NPS Simulation Telemetry]] | Firmware logs in Python: two message paths, adding new messages, format gotchas |
| [[09 - FlightGear 3D Visualization]] | Live 3D view of NPS sim in FlightGear on Mac — setup, gotchas, verification |
| [[11 - In-Flight Controller Switching (Oneloop Pattern)]] | How Paparazzi switches control laws in flight (XML→codegen→runtime); oneloop ANDI/INDI precedent |
| [[Plans/Dual-Controller Shadow Mode]] | Plan: run MFC + INDI together, INDI drives, MFC shadowed for validation |
| [[Plans/Dual-Controller Handover Mode]] | Plan: in-flight bumpless switch of motor authority between MFC and INDI |
| [[Plans/Flatness Trajectory Setpoints (Pos-Vel-Accel-Jerk-Snap + Psi)]] | Plan: extend GUIDED-mode trajectory setpoints to jerk/snap + psi derivatives, generically for INDI/MFC |

## Quick-start question

> "I want to change ANTON's stabilization — where do I start?"

→ Go to [[06 - Modifying ANTON Stabilization]] for the worked example, then use [[07 - All Touch Points Cheatsheet]] as a checklist.
