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

- **Paparazzi guidance has a 6-function plug seam; variants are mutually exclusive.** `guidance_h.c`/`guidance_v.c` are generic drivers that own mode logic, the reference models, AND nav integration (they fill `gv->z_ref` from `nav.nav_altitude` and `gh->ref.pos`). They dispatch to functions the *active* variant supplies: `guidance_h_run_pos/speed/accel`, `guidance_v_run_pos/speed/accel`, `guidance_h_run_enter`, `guidance_v_run_enter`. Each variant (`guidance_pid.c`, `guidance_indi.c`, `guidance_hybrid.c`, `guidance_oneloop.c`, now `guidance_mfc.c`) defines exactly these and `<provides>guidance</provides>` so only one links. To add a variant: implement the six, set `<provides>guidance,attitude_command</provides>`, and `GUIDANCE_PID_USE_AS_DEFAULT=FALSE` in the makefile so PID's defaults (guarded by `#if GUIDANCE_PID_USE_AS_DEFAULT`) don't collide. Select with `<module name="guidance" type="X"/>` → resolves to `guidance_X.xml`.
- **`mfc_core` is a single shared global instance** (`mfc.sample_time/start_time/time`) used by all `mfc_siso_run` calls — the MFC stabilizer's 3 axes AND the MFC guidance's 3 axes. Safe only because the stacks are crossed: MFC is *either* the stabilizer *or* the guidance, never both. Whichever side is MFC must call `mfc_core_init(1/PERIODIC_FREQUENCY)` once: `stabilization_mfc_init` does it for the stab side, `guidance_indi_init` (THRUST_MFC) and now `guidance_mfc_init` for the guidance side. If nobody calls it, `mfc.sample_time=0` → `mfc_siso_run` divides by zero → NaN commands (bug-029).

## Do-Not-Repeat

<!-- Mistakes made and corrected. Each entry prevents the same mistake recurring. -->
<!-- Format: [YYYY-MM-DD] Description of what went wrong and what to do instead. -->

- [2026-06-05] **`build_fw.sh` CONF_XML is relative to `paparazzi/`, NOT `/workspace`.** The workspace CLAUDE.md example (`paparazzi/conf/...`) is misleading — `build_fw.sh` runs `make -C /workspace/paparazzi`, so pass `conf/airframes/ENAC/conf_enac.xml`. Using the `paparazzi/`-prefixed path gives `File_not_found` from the aircraft generator.
- [2026-06-05] **NPS `SYS_TIME_FREQUENCY` defaults to 1000, not `2*PERIODIC_FREQUENCY`.** `PERIODIC_FREQUENCY` in the airframe is a `<configure>` (makefile var), not a C `#define`, so `sys_time.h` doesn't see it and falls to the `#else` 1000 branch. So NPS sim steps at ~1 kHz (SIM_DT≈1ms), confirmed by measuring the scope emitter (~512 datagrams/sim-s at decim 2). Don't assume 2 kHz.

## Cerebrum additions

- **Two independent NPS telemetry paths** (don't conflate): (A) `NPS_*` truth = simulator's `nps_ivy_display()` thread `IvySendMsg` text straight to Ivy at `3*DISPLAY_DT`=10 Hz; (B) firmware telemetry = `pprz_msg_send_*` binary PPRZ over UDP:4242 → `link` decodes → Ivy text → `server`. `server.ml:187` JSON-streams every received msg to `udp_json_stream_addr:9870` event-driven (one datagram per msg at its own rate, walltime-stamped). The AP firmware runs *in-process* in `simsitl`, so `fdm` (truth) and firmware control globals are readable directly from `nps_main_run_sim_step()`.
- **In-process scope emitter** (`nps_scope.c/.h`): reads `fdm` at end of `nps_main_run_sim_step()`, sends one UDP/JSON datagram per decimated sim step to PlotJuggler, `fdm.time` stamp. CLI: `--scope_host/--scope_port/--scope_decim`. Default port 9870 in `sim_anton.py` (the old Python `scope_writer` is now `--debug-scope` on 9871).
- **Firmware-registered scope vars (REDESIGN, supersedes the MFC hardcode/guard above).** `nps_scope.c` is now controller-agnostic: a generic registry (`nps_scope_register`/`nps_scope_register_array`) plus the `truth` block. Firmware files register globals with one line at file scope: `#include "nps_scope_var.h"` then `NPS_SCOPE_VAR("mfc/roll/err", &mfc_roll.error[0], NPS_SCOPE_FLOAT);`. The macro is a `__attribute__((constructor))` that appends `{name,&addr,type}`; emitted as a top-level JSON key (slash names → PlotJuggler tree). **Cross-target trick:** the shim `nps_scope_var.h` lives on the always-on include path (`sw/airborne`, see `sw/airborne/Makefile:34`) and gates the real include + active macro on `USE_NPS` (defined only for sim/nps/hitl by `nps_common.xml`); on `ap` it's a pure no-op so `stabilization_mfc.c` (built for both) compiles everywhere. The old `STABILIZATION_MFC_ROLL_ALPHA`/`NPS_SCOPE_HAS_MFC` guard is GONE.
- **Note:** for ANTON_MFC, `#if !STABILIZATION_MFC_ALLOCATION_PSEUDO_INVERSE` evaluates *true* at preprocess time (so `wls_stab_p` exists and the `wls/*` registration compiles in) even though the airframe sets `ALLOCATION_PSEUDO_INVERSE=TRUE` — `TRUE` isn't a numeric macro in that preprocessor context. Mirror the header's own guard verbatim and it stays consistent.

## Decision Log

<!-- Significant technical decisions with rationale. Why X was chosen over Y. -->

- [2026-06-09] **MFC guidance stack design**: `guidance_mfc.c` runs three independent SISO loops (GX, GY, GZ) and converts the horizontal acceleration commands to a quaternion attitude setpoint via `accel_to_att_sp()` (same R_psi rotation as guidance_indi). Chosen over coupling GX/GY because the SISO independence matches the existing MFC stabilizer architecture.
- [2026-06-09] **Stack-switch via `<control_block>`**: MFC vs INDI guidance is switched by swapping one `<call_block>` line in `anton_mfc_autopilot.xml` + rebuild. Chosen over runtime flag because it eliminates dead code paths in flight and makes the active stack explicit in source control.
- [2026-06-10] **SUPERSEDES the above + the compile-time 2×2**: `guidance_mfc` is now a standard mutually-exclusive guidance variant (provides `guidance`, supplies the six plug functions). The autopilot lost all MFC machinery (no `STACK_USE_MFC_GUIDANCE`, no cond= dual-guidance, no `guidance_mfc_enter`); NAV uses the stock `run_guidance_control` block. Stack select is now a paired 2-line swap in `anton_mfc.xml` (guidance `type` + stabilization `type` together; default MFC->INDI = guidance mfc + stab indi). Trade: can no longer co-compile both guidances (accepted) — gain the stock control block, a maintainable variant, and the z_ref=0 bug disappears structurally (driver fills `gv->z_ref` before `guidance_v_run_pos`). The custom autopilot is kept ONLY for the trimmed mode set; it could be replaced by stock `rotorcraft_autopilot.xml`.
- [2026-06-09] **Makefile.ac caching bug (known limitation)**: `sw/lib/ocaml/aircraft.ml:478` sets `autopilot=true` only for the current `-target`; the file is only copied if newer than sources. After NPS build, AP build reuses the NPS Makefile.ac (which lacks `USE_GENERATED_AUTOPILOT=TRUE` in the AP block). Fix: delete `Makefile.ac` before switching targets. This is a Paparazzi upstream bug.

## Do-Not-Repeat additions

- [2026-06-09] **`Bound(val, lo, hi)` is a statement macro, not an expression.** Using `asinf(Bound(...))` fails. Always use an intermediate variable: `Bound(val, lo, hi); result = asinf(val);`
- [2026-06-09] **Use `&&` not `&amp;&amp;` in autopilot XML `cond=` attributes.** The autopilot code generator copies the attribute value verbatim to C. XML entities like `&amp;&amp;` produce `&amp;&amp;` in generated C — a parse error.
- [2026-06-09] **Always delete `Makefile.ac` when switching from NPS→AP build on a fresh aircraft.** See Makefile.ac caching bug above. `rm paparazzi/var/aircrafts/<AC>/Makefile.ac` before `./build_fw.sh ... ap`.
