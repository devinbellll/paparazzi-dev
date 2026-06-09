# Session — 2026-06-04: FlightGear Integration and OpenWolf Initialization

## What changed
- **FlightGear 3D visualization support:** Updated `sim_anton.py --fg` flag to launch NPS with FlightGear visualization on Mac host
- Created `paparazzi/conf/simulator/flightgear/bebop-set.xml` — lightweight FG aircraft model
- Fixed FG-NPS protocol mismatch: added `--fg_fdm` flag to use FDM protocol (v24) instead of GUI (v8)
- Fixed hostname resolution: added `socket.gethostbyname()` in sim_anton.py to resolve `host.docker.internal` (NPS inet_addr doesn't support hostnames)
- **Devcontainer firewall:** Patched `init-firewall.sh` to auto-detect host IP and allow UDP 5501 for FG packets
- **OpenWolf initialization:** Created `.wolf/` directory structure (anatomy.md, cerebrum.md, buglog.json, memory.md)
- Created Knowledge/09 - FlightGear 3D Visualization.md with full setup guide and gotchas

## Bugs fixed
- **bug-002:** FlightGear shows drone but no motion — NPS GUI vs FDM protocol mismatch
- **bug-003:** PermissionError sending UDP to Mac host — firewall OUTPUT DROP policy
- **bug-004:** NPS broadcast address (255.255.255.255) instead of host IP — inet_addr() hostnames broken
- **bug-005:** IPv6 resolution in init-firewall.sh broken iptables — switched to getent ahostsv4

## What I learned
- FlightGear expects FDM protocol (v24) with native-fdm flag; NPS defaults to GUI (v8)
- Docker Desktop Mac host resolves via `host.docker.internal` but maps to `192.168.65.254` — must be resolved at container startup
- Devcontainer firewall has OUTPUT DROP; must explicitly allow host.docker.internal traffic
- OpenWolf project structure helps with cross-session learning and bug tracking

## What's next
- PlotJuggler integration for in-process scope emitter
- Verify FG visualization with multiple aircraft types
