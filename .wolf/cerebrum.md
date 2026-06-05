# Cerebrum

> OpenWolf's learning memory. Updated automatically as the AI learns from interactions.
> Do not edit manually unless correcting an error.
> Last updated: 2026-06-04

## User Preferences

<!-- How the user likes things done. Code style, tools, patterns, communication. -->

## Key Learnings

- **Project:** workspace
- **Description:** Headless firmware build environment for ENAC UAV Lab aircraft. Cross-compiles ARM Cortex-M firmware using the Paparazzi autopilot framework inside a containerized linux/amd64 toolchain.

- **FlightGear NPS integration requires `--fg_fdm` flag.** NPS defaults to GUI protocol (FGNetGUI, version 8). `--native-fdm` in FG expects FDM protocol (FGNetFDM, version 24). Without `--fg_fdm`, FG silently discards every packet. `sim_anton.py --fg` passes this automatically.

- **`nps_flightgear_init` uses `inet_addr()` — hostnames don't work.** Must resolve `host.docker.internal` to an IP in Python before passing to simsitl. `sim_anton.py` uses `socket.gethostbyname()` for this. Docker Desktop Mac host resolves to `192.168.65.254`.

- **Devcontainer firewall blocks outbound to Mac host.** `init-firewall.sh` sets `HOST_IP` = Docker bridge gateway (`172.24.0.1`), not the Mac host (`192.168.65.254`). Patched to auto-detect and allow `host.docker.internal` at container startup. Requires `sudo` which is available in this devcontainer.

- **`getent hosts host.docker.internal` can return IPv6 only** (e.g. `fdc4:f303:9324::254`) depending on Docker Desktop version/network mode. Passing IPv6 to `iptables` (IPv4) causes the rule to fail or be skipped, leaving FG UDP 5501 blocked on container restart. Fix: use `getent ahostsv4 host.docker.internal | awk '{print $1; exit}'` to force IPv4 resolution. The IPv4 address is `192.168.65.254`. After each container restart, if FG packets don't flow, check with `sudo iptables -L OUTPUT -n | grep 192.168.65` — if missing, re-run init-firewall.sh or apply manually.

- **FlightGear set files (`-set.xml`) cannot have `--` inside XML comments.** XML spec forbids `--` inside comment bodies. Breaks IDE syntax highlighting (makes everything appear commented). Use plain prose in comments, not CLI flags with double-dash.

- **NPS FG pipeline verification sequence:** (1) check eth0 TX packets rising ~30/s, (2) `nc -ul 5501 | xxd` on Mac shows binary data, (3) `nc` returns "Address already in use" when FG is listening, (4) decode first 4 bytes of packet with ntohl → should be 24.

## Do-Not-Repeat

<!-- Mistakes made and corrected. Each entry prevents the same mistake recurring. -->
<!-- Format: [YYYY-MM-DD] Description of what went wrong and what to do instead. -->

## Decision Log

<!-- Significant technical decisions with rationale. Why X was chosen over Y. -->
