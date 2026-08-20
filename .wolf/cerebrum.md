# Cerebrum

> OpenWolf's learning memory. Updated automatically as the AI learns from interactions.
> Do not edit manually unless correcting an error.
> Last updated: 2026-07-01

## User Preferences

<!-- How the user likes things done. Code style, tools, patterns, communication. -->

## Key Learnings

- **Project:** workspace
- **Description:** Headless firmware build environment for ENAC UAV Lab aircraft. Cross-compiles ARM Cortex-M firmware using the Paparazzi autopilot framework inside a containerized linux/amd64 toolchain.

- **Running two controllers / switching in flight = ONE law at a time, not parallel shadow.** Paparazzi's only precedent (oneloop ANDI/INDI, `rotorcraft_oneloop_switch.xml`) is one module with both laws sharing state, switched by an int (`ctrl_type`) re-`enter()`d on each `<on_enter>`. The autopilot XML is a codegen input (`gen_autopilot.ml`), not runtime config. MFC vs INDI is harder: two separate modules with COLLIDING global symbols (`stabilization_attitude_run`, strong `set_rotorcraft_commands` override, `g1g2`/`actuators_pprz`/`act_is_servo`/`stab_thrust_filt`) and both `<provides>commands</provides>`/`guidance` — so they can't naively co-compile. A dual wrapper module owning the contested singletons is required first. See `Knowledge/11 …` + `Knowledge/Plans/`.

- **FlightGear NPS integration requires `--fg_fdm` flag.** NPS defaults to GUI protocol (FGNetGUI, version 8). `--native-fdm` in FG expects FDM protocol (FGNetFDM, version 24). Without `--fg_fdm`, FG silently discards every packet. `sim_anton.py --fg` passes this automatically.

- **`nps_flightgear_init` uses `inet_addr()` — hostnames don't work.** Must resolve `host.docker.internal` to an IP in Python before passing to simsitl. `sim_anton.py` uses `socket.gethostbyname()` for this. Docker Desktop Mac host resolves to `192.168.65.254`.

- **Devcontainer firewall blocks outbound to Mac host.** `init-firewall.sh` sets `HOST_IP` = Docker bridge gateway (`172.24.0.1`), not the Mac host (`192.168.65.254`). Patched to auto-detect and allow `host.docker.internal` at container startup. Requires `sudo` which is available in this devcontainer.

- **`getent hosts host.docker.internal` can return IPv6 only** (e.g. `fdc4:f303:9324::254`) depending on Docker Desktop version/network mode. Passing IPv6 to `iptables` (IPv4) causes the rule to fail or be skipped, leaving FG UDP 5501 blocked on container restart. Fix: use `getent ahostsv4 host.docker.internal | awk '{print $1; exit}'` to force IPv4 resolution. The IPv4 address is `192.168.65.254`. After each container restart, if FG packets don't flow, check with `sudo iptables -L OUTPUT -n | grep 192.168.65` — if missing, re-run init-firewall.sh or apply manually.

- **FlightGear set files (`-set.xml`) cannot have `--` inside XML comments.** XML spec forbids `--` inside comment bodies. Breaks IDE syntax highlighting (makes everything appear commented). Use plain prose in comments, not CLI flags with double-dash.

- **NPS FG pipeline verification sequence:** (1) check eth0 TX packets rising ~30/s, (2) `nc -ul 5501 | xxd` on Mac shows binary data, (3) `nc` returns "Address already in use" when FG is listening, (4) decode first 4 bytes of packet with ntohl → should be 24.

- **Paparazzi guidance has a 6-function plug seam; variants are mutually exclusive.** `guidance_h.c`/`guidance_v.c` are generic drivers that own mode logic, the reference models, AND nav integration (they fill `gv->z_ref` from `nav.nav_altitude` and `gh->ref.pos`). They dispatch to functions the *active* variant supplies: `guidance_h_run_pos/speed/accel`, `guidance_v_run_pos/speed/accel`, `guidance_h_run_enter`, `guidance_v_run_enter`. Each variant (`guidance_pid.c`, `guidance_indi.c`, `guidance_hybrid.c`, `guidance_oneloop.c`, now `guidance_mfc.c`) defines exactly these and `<provides>guidance</provides>` so only one links. To add a variant: implement the six, set `<provides>guidance,attitude_command</provides>`, and `GUIDANCE_PID_USE_AS_DEFAULT=FALSE` in the makefile so PID's defaults (guarded by `#if GUIDANCE_PID_USE_AS_DEFAULT`) don't collide. Select with `<module name="guidance" type="X"/>` → resolves to `guidance_X.xml`.
- **`PERIODIC_FREQUENCY` is part of INDI tuning, not a free knob — yaw breaks first if it's wrong.** A set of INDI gains is only valid at the loop rate it was tuned at. `stabilization_indi.c` bakes the rate into `act_dyn_discrete = 1-exp(-ACT_FREQ/PERIODIC_FREQUENCY)`, all rate/accel estimates (`Δ × PERIODIC_FREQUENCY`), and filter sample times (`1/PERIODIC_FREQUENCY`). The **yaw axis** is driven by the rate-sensitive, low-authority **G2** spin-up compensation (`g2_times_u`, G2≈±150 vs G1 yaw row ±5), so a mismatched rate makes yaw limit-cycle while roll/pitch (8× authority, no G2) stay fine. `anton_mfc.xml` was 500 Hz while its INDI gains were proven on `anton_indi_aruco.xml` at 1000 Hz → persistent yaw oscillation (bug-mfc-yaw-period). When porting INDI gains between airframes, **match `PERIODIC_FREQUENCY` first.** See `Knowledge/02 - INDI Stabilization Deep Dive.md`.
- **`guidance_mfc` must seed `nav.heading` on entry like `guidance_indi`.** `guidance_indi_enter()` sets `nav.heading = stateGetNedToBodyEulers_f()->psi` so the yaw reference starts at the current heading; `guidance_mfc` originally hardcoded yaw sp to `0.0` and never seeded it. Fix: set `nav.heading` in `guidance_h_run_enter` and pass `gh->sp.heading` into `accel_to_att_sp`.
- **`mfc_core` is a single shared global instance** (`mfc.sample_time/start_time/time`) used by all `mfc_siso_run` calls — the MFC stabilizer's 3 axes AND the MFC guidance's 3 axes. Safe only because the stacks are crossed: MFC is *either* the stabilizer *or* the guidance, never both. Whichever side is MFC must call `mfc_core_init(1/PERIODIC_FREQUENCY)` once: `stabilization_mfc_init` does it for the stab side, `guidance_indi_init` (THRUST_MFC) and now `guidance_mfc_init` for the guidance side. If nobody calls it, `mfc.sample_time=0` → `mfc_siso_run` divides by zero → NaN commands (bug-029).

- **PlotJuggler UDP/JSON (server.ml port 9870) crashes on any string field containing a backslash/quote/control char.** `json_of_message` (`sw/ext/pprzlink/lib/v2.0/ocaml/pprzLink.ml` ~L789) emits `char[]`/`string` field values raw — no JSON escaping. `value` (L174) parses `char[]`/`string`/`FixedArrayType char` → `String v`, so clean content (e.g. WLS `loop="stab"`, the git version desc) is valid JSON and never crashes; only actual backslash content (`"PPRZ\p…"`) yields `nlohmann parse_error.101` and PlotJuggler stops the UDP server on the first bad packet. It is **content-dependent, not message-dependent** — removing individual messages (WLS_U/V) is whack-a-mole; the only real fix is a `json_escape` helper applied to the String case + char-array case. (bug-167)

- **A telemetry process with >1 mode auto-generates a `telemetry_mode_<Process>` switch SETTING for target `ap|nps`.** That symbol is only defined by whatever `.c` does `#define PERIODIC_C_<PROCESS>`. For `FlightRecorder` that's `flight_recorder.c`, which is `<makefile target="ap">` + chibios-only deps → on **nps** nothing defines it → `ld: undefined reference to telemetry_mode_FlightRecorder` (bug-168). `mfc_flight_test.xml` FlightRecorder has 2 modes (default+mfc) → fails on nps; `default_rotorcraft.xml` FlightRecorder has 1 mode → no setting → ANTON_MFC nps links. Single-mode non-Main processes are sim-safe; multi-mode ones need an nps owner stub.


- **PlotJuggler shared schema (2026-07-08): every feed is normalized before PlotJuggler.** `pj_json_relay.normalize_obj` rewrites any telemetry packet to root `"uav"` and renames MFC branches (`STAB_MFC`→`MFC_STAB`, `GUIDANCE_MFC`→`MFC_GUIDANCE`, `GUIDANCE_MFC_ACC2ATT`/`ACC2ATT`→`MFC_ACC2ATT`), so one layout (`plotjuggler_mfc.xml` / `plotjuggler_indi.xml`, curves `/uav/...`) works for any aircraft, sim or real. In SITL `sim_anton.py` forwards ONLY the NPS scope (via local port 9871 → normalize → PJ_HOST:PJ_PORT, default Mac:9870); ivy telemetry is captured to .jsonl but forwarded only with `--no-scope`. Real flight: run `pj_json_relay.py` standalone (normalizes by default, `--raw` to disable). pprzlink message names in messages.xml are UNCHANGED — the rename happens only at the relay layer. Firmware NPS_SCOPE_VAR strings now use the MFC_* names directly.

## Do-Not-Repeat

- (2026-07-09) When fixing a discovered modeling/correctness error, do NOT wrap the fix in a compile-time define + runtime toggle + GCS setting for backward compatibility unless asked. User rejected the STABILIZATION_MFC_G2_IN_ALLOCATION flag machinery and chose the direct unconditional fix (drop G2 from MFC allocation entirely). Preference: minimal, decisive fixes; wrong behavior should not remain selectable.

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
- [2026-06-11] **MFC timing is per-SISO-module, not global**: `sample_time`/`time`/`start_time` live in each `struct MfcParameters`; API is `mfc_siso_init(axis, dt)` + `mfc_siso_reset(axis)` (no more global `struct Mfc`/`mfc_core_init`/`mfc_core_start`). Chosen so axes start/reset independently. This made the bug-030/031 global-clock workarounds obsolete — they were removed (bug-033): `guidance_mfc_vert()` now runs raw MFC every cycle when `gz.enabled` (no `in_flight` gating). Trade: ground estimator windup returns (accepted by user). Result: MFC Z guidance flies smooth in NPS. Verified state = commit 24dcae6.
- [2026-06-09] **Makefile.ac caching bug (known limitation)**: `sw/lib/ocaml/aircraft.ml:478` sets `autopilot=true` only for the current `-target`; the file is only copied if newer than sources. After NPS build, AP build reuses the NPS Makefile.ac (which lacks `USE_GENERATED_AUTOPILOT=TRUE` in the AP block). Fix: delete `Makefile.ac` before switching targets. This is a Paparazzi upstream bug.
- [2026-06-11] **Thrust-unit conversion lives in guidance, not the shared stabilizer**: `guidance_mfc` emits its vertical thrust in physical (indi_v[3]-space) units. A single compile-time `GUIDANCE_MFC_THRUST_TO_PPRZ` define (default FALSE) selects packaging: FALSE → raw float (`th_sp_from_thrust_f`) for `stabilization_mfc`'s pseudo-inverse; TRUE → scale by `GUIDANCE_MFC_THRUST_PPRZ_SCALE = 1/sum(Bwls[3][i])` and emit an int (`th_sp_from_thrust_i`) so the **stock** INDI Bwls path round-trips exactly. Chosen over hacking each stabilizer so both `stabilization_indi.c` and `stabilization_mfc.c` keep their stock contracts; guidance adapts to the selected downstream. `stabilization_indi.c` was reverted to stock; `stabilization_mfc.c` now reads `thrust->sp.thrust_f[Z]` directly (OUTPUTS==4). Scale for ANTON = `1000/(4*-1.5)` = -166.67. Supersedes the bug-035 diagnosis "option (2)" hack.


- **PlotJuggler shared schema (2026-07-08): every feed is normalized before PlotJuggler.** `pj_json_relay.normalize_obj` rewrites any telemetry packet to root `"uav"` and renames MFC branches (`STAB_MFC`→`MFC_STAB`, `GUIDANCE_MFC`→`MFC_GUIDANCE`, `GUIDANCE_MFC_ACC2ATT`/`ACC2ATT`→`MFC_ACC2ATT`), so one layout (`plotjuggler_mfc.xml` / `plotjuggler_indi.xml`, curves `/uav/...`) works for any aircraft, sim or real. In SITL `sim_anton.py` forwards ONLY the NPS scope (via local port 9871 → normalize → PJ_HOST:PJ_PORT, default Mac:9870); ivy telemetry is captured to .jsonl but forwarded only with `--no-scope`. Real flight: run `pj_json_relay.py` standalone (normalizes by default, `--raw` to disable). pprzlink message names in messages.xml are UNCHANGED — the rename happens only at the relay layer. Firmware NPS_SCOPE_VAR strings now use the MFC_* names directly.

## Do-Not-Repeat additions

- [2026-06-09] **`Bound(val, lo, hi)` is a statement macro, not an expression.** Using `asinf(Bound(...))` fails. Always use an intermediate variable: `Bound(val, lo, hi); result = asinf(val);`
- [2026-06-09] **Use `&&` not `&amp;&amp;` in autopilot XML `cond=` attributes.** The autopilot code generator copies the attribute value verbatim to C. XML entities like `&amp;&amp;` produce `&amp;&amp;` in generated C — a parse error.
- [2026-06-09] **Always delete `Makefile.ac` when switching from NPS→AP build on a fresh aircraft.** See Makefile.ac caching bug above. `rm paparazzi/var/aircrafts/<AC>/Makefile.ac` before `./build_fw.sh ... ap`.
- [2026-06-11] **`build_fw.sh` CONF_XML is relative to `paparazzi/`, not `/workspace`.** Use `conf/airframes/ENAC/conf_enac.xml` (the script's own header is right; the root CLAUDE.md example `paparazzi/conf/...` fails with `Xml_light_errors.File_not_found`).
- [2026-06-15] **For an INDI yaw oscillation, diff `PERIODIC_FREQUENCY` between airframes FIRST — don't chase thrust scaling/WLS/setpoint paths.** Wasted a long pass on thrust→WLS magnitude, WLS priorities, incremental-vs-absolute thrust, and flat-attitude coupling — all dead ends. The cause was a 500 vs 1000 Hz loop-rate mismatch (yaw is the most rate-sensitive INDI axis via G2). Whole-XML `diff` of the two airframes surfaced it immediately. See bug-mfc-yaw-period.
- [2026-06-11] **The `stabilization type="mfc"` stack does not currently compile** (pre-existing, bug-039): `wls_alloc.h`/`stabilization_mfc.c` include `stabilization_indi.h`, whose `extern float g1g2[INDI_OUTPUTS][INDI_NUM_ACT]` needs INDI macros undefined in an MFC-only build. Don't assume a stab=mfc airframe builds; the active ANTON_MFC stack is MFC->INDI (stab=indi).

## Key Learnings additions

- [2026-06-15] **A Paparazzi module's source must live under `sw/airborne/modules/<dir>/`.** Setting `dir="firmwares/rotorcraft"` on a `<module>` does NOT make the build find a file there — you get `No rule to make target .../modules/firmwares/rotorcraft/<file>.c`. Put module .c/.h under `modules/<name>/` (dir defaults to module name).
- [2026-06-15] **A module that declares a generated `<periodic>`/`<event>`/`<init>` call needs a `<header><file name="x.h"/></header>` element**, or the generated `modules.h` calls the function with no prototype (implicit-declaration warning, and confusing failures).
- [2026-06-15] **NPS scope architecture: truth/* is hardcoded in `sw/simulator/nps/nps_scope.c` (universal); everything else is firmware globals registered via `NPS_SCOPE_VAR`/`NPS_SCOPE_VARN` (no-op off-sim).** For clean SI/deg + fixed-point/union conversion, register a static float mirror refreshed by a sim-only module `<periodic>` (see `modules/nps_scope/nps_scope_state.c`) rather than raw addresses.


- **PlotJuggler shared schema (2026-07-08): every feed is normalized before PlotJuggler.** `pj_json_relay.normalize_obj` rewrites any telemetry packet to root `"uav"` and renames MFC branches (`STAB_MFC`→`MFC_STAB`, `GUIDANCE_MFC`→`MFC_GUIDANCE`, `GUIDANCE_MFC_ACC2ATT`/`ACC2ATT`→`MFC_ACC2ATT`), so one layout (`plotjuggler_mfc.xml` / `plotjuggler_indi.xml`, curves `/uav/...`) works for any aircraft, sim or real. In SITL `sim_anton.py` forwards ONLY the NPS scope (via local port 9871 → normalize → PJ_HOST:PJ_PORT, default Mac:9870); ivy telemetry is captured to .jsonl but forwarded only with `--no-scope`. Real flight: run `pj_json_relay.py` standalone (normalizes by default, `--raw` to disable). pprzlink message names in messages.xml are UNCHANGED — the rename happens only at the relay layer. Firmware NPS_SCOPE_VAR strings now use the MFC_* names directly.

## Do-Not-Repeat additions

- [2026-06-15] **`ap` build is broken on daily-notes branch (bug-050, pre-existing): `nps_v_thrust` used unguarded at stabilization_indi.c:751 but declared only under `#ifdef SITL`.** Don't attribute this to NPS/sim work. Fix is to `#ifdef SITL`-guard line 751.

## Key Learnings additions (2026-06-17 — containerized build/sim)

- Runtime is now an aarch64 sbx sandbox (IS_SANDBOX=1, SANDBOX_VM_ID=claude-paparazzi-dev), NOT the old amd64 Rosetta devcontainer. No local toolchain. A nested arm64 Docker daemon (DinD) is available and shares the sandbox filesystem, so `docker run -v "$WORKSPACE_DIR":/workspace ...` bind-mounts the repo and outputs land back in the tree.
- Firmware is cross-compiled (arm-none-eabi → Cortex-M), so build-host arch does NOT change the .elf. arm64-native builds are valid and faster; the old USE_LTO=no was a Rosetta ICE workaround (kept only for fast incremental dev).
- The paparazzi PPA needs api.launchpad.net, keyserver.ubuntu.com, ppa.launchpadcontent.net — all blocked by the sandbox default-deny policy. ports.ubuntu.com + launchpad.net are allowed. User must `sbx policy allow network ...` on the host before ./build_image.sh. The `sbx policy allow` takes effect on the RUNNING sandbox immediately — NO relaunch needed (verified by curl probe).
- RESOLVED 2026-06-17: the PPA DOES ship arm64. paparazzi-dev 3.31~ubuntu5 (an `all`-arch metapackage) + paparazzi-jsbsim 1.7-1ubuntu2; dry-run install resolves 787 pkgs, 0 broken, on arm64. The OCaml ground segment is PREBUILT for arm64 — no source compile, no amd64 emulation. The old "no way to compile OCaml for arm64" doubt conflated this with native-macOS OCaml (which we are NOT doing). END-TO-END VERIFIED: clean `./build_fw.sh ANTON conf/airframes/ENAC/conf_enac.xml nps` from the sandbox produces a native-arm64 simsitl (ELF machine 0xb7).
- CROSS-ARCH CONTAMINATION (critical): all four actors (host Mac, host Docker build container, Claude sandbox, sandbox DinD container) share ONE workspace tree via /workspace bind mounts. Firmware .o/.elf are cross-compiled (arch-independent, safe), BUT the native host-side OCaml tooling is arch-specific and ALSO lives in the shared tree: sw/ext/pprzlink/build/ocaml/.../myocamlbuild and sw/lib/ocaml/dlllib-pprz.so + *.cma/*.cmi. If host (amd64) and sandbox (arm64) build into the same tree, these collide → "Exec format error" (myocamlbuild) or "cannot load shared library dlllib-pprz" / "file in wrong format". RULE: pick ONE arch for everything. M3 is arm64-native so the HOST Docker Desktop image should ALSO be arm64 (./build_image.sh with NO --amd64; runs native, no Rosetta). To switch a tree that was built amd64 → arm64, run `make clean` in-container (purges sw/lib/ocaml + pprzlink/build) before rebuilding. NEVER run host and sandbox builds concurrently against the tree.
- Platform-mismatch gotcha: pprz_run must `--platform` MATCH the built image's arch. Requesting linux/arm64 against an amd64 image (or vice-versa) makes docker treat the local image as ABSENT and try to PULL it → "Unable to find image ... locally". pprz_docker.sh pprz_platform() now auto-detects via `docker image inspect --format {{.Architecture}}`; PPRZ_PLATFORM overrides.
- unifiedmocaprouter (sw/ext, OptiTrack mocap router) CANNOT build on arm64: its NatNet SDK fetchcontent pulls a vendor x86-64-only libNatNet.so → ld "file in wrong format". It is NOT needed for firmware or NPS sim. Upstream sw/ext/Makefile self-skips it when libboost-program-options-dev is absent (BOOST_INSTALLED check) — so Dockerfile.build deliberately does NOT install boost.
- Host Mac is reachable only as a network host (host.docker.internal); the sandbox cannot run host shell commands, so host-native builds can't be driven from here — ephemeral containers are the path.
- VSCode model chosen = native Mac + Docker Desktop (Option B): VSCode dispatches to its own daemon, Claude to the sandbox's; both write the same shared files. Requires /workspace→host path rewrites for compile_commands.json + build.log + debugger sourceFileMap, and clangd (not MS C/C++ engine) for IntelliSense.

## Decision Log additions (2026-06-17)

- Chose ephemeral arm64 containers over host-Mac builds (can't drive host shell) and over amd64 emulation (no binfmt; slower; firmware identical anyway).
- Consolidated build_active/gen_compile_db/gen_build_log/gen_vscode into pprz.sh subcommands (user request), keeping the active-config picker UX.
- Kept the legacy .devcontainer/ + Dockerfile.paparazzi for reference rather than deleting.

## User Preferences additions (2026-06-17)

- Prefers being shown questions as plain text first (rejected AskUserQuestion tool prompts twice asking to "show/give back the question").
- Wants complete VSCode(host)↔Claude(sbx)↔build integration: real symbols, generated #defines, true linking, fast incremental builds, accurate debugging, token-efficient exploration.

## Key Learnings additions (2026-06-17 — build tooling overhaul)

- Paparazzi's per-aircraft `<target>.compile` (from `make -C paparazzi -f Makefile.ac AIRCRAFT=.. CONF_XML=.. <target>.compile`) is ALREADY incremental/CMake-like: it depends on `<target>.ac_h` (codegen via gen_aircraft.out, which only rewrites airframe.h/modules.h when the XML changed — it compares mtimes via `is_older` + an md5 of conf_aircraft.xml), then `cd sw/airborne && make all` recompiles only changed TUs and relinks only if an object changed. Don't reinvent incrementality — one `<target>.compile` IS the build button. Measured: full nps 51s → no-op rebuild 10s (0 compiles) → 1-file touch 11s (1 TU + relink).
- The ONLY redundant slow step in the old build_fw.sh was `make -j1 -C paparazzi` (ground segment) on EVERY build. Gate it behind a guard (generators/`var/include` present) so it runs once. This alone removes the need for the old "fast" vs "full" split.
- CONF_XML canonical form is `conf/airframes/ENAC/conf_enac.xml` (relative to the `paparazzi/` dir), NOT `paparazzi/conf/...`. gen_aircraft.out parses `-conf` relative to make's cwd, and the build runs `make -C paparazzi`, so the repo-root form resolves to `paparazzi/paparazzi/conf/...` and FAILS. The old docs/build_fw.sh examples using `paparazzi/conf/...` were wrong; pprz.sh only worked because it converted to an absolute path.
- compiledb is the bear-free way to get compile_commands.json: it PARSES make's verbose stdout (needs `USE_VERBOSE_COMPILE=yes Q=''` for the chibios/airborne rules to echo the real `$(CC) -c ...` lines, and `--print-directory` + `--build-dir` for per-TU include resolution). It drops entries whose source file doesn't exist on disk (use `-S/--no-strict` to disable). nps echoes "CC file.o"/"LD ..."; ap/chibios echoes "Compiling file".
- A clangd compile_commands.json only needs the per-FILE compile commands; a final LINK failure is irrelevant. pprz.sh db tolerates it on purpose (ap firmware may legitimately not link mid-dev).


- **PlotJuggler shared schema (2026-07-08): every feed is normalized before PlotJuggler.** `pj_json_relay.normalize_obj` rewrites any telemetry packet to root `"uav"` and renames MFC branches (`STAB_MFC`→`MFC_STAB`, `GUIDANCE_MFC`→`MFC_GUIDANCE`, `GUIDANCE_MFC_ACC2ATT`/`ACC2ATT`→`MFC_ACC2ATT`), so one layout (`plotjuggler_mfc.xml` / `plotjuggler_indi.xml`, curves `/uav/...`) works for any aircraft, sim or real. In SITL `sim_anton.py` forwards ONLY the NPS scope (via local port 9871 → normalize → PJ_HOST:PJ_PORT, default Mac:9870); ivy telemetry is captured to .jsonl but forwarded only with `--no-scope`. Real flight: run `pj_json_relay.py` standalone (normalizes by default, `--raw` to disable). pprzlink message names in messages.xml are UNCHANGED — the rename happens only at the relay layer. Firmware NPS_SCOPE_VAR strings now use the MFC_* names directly.

## Do-Not-Repeat additions

- [2026-06-17] **Don't use `bear` for compile_commands.json in the ephemeral build container.** bear 3.x's gRPC intercept wrapper can't reach its loopback daemon there → "gRPC call failed ... Connection refused", every compile fails, no DB. Use `compiledb --parse` on a verbose build log (bug-051).
- [2026-06-17] **`pprz.sh` interface is `pprz.sh <cmd> AIRCRAFT [TARGET]`** (e.g. `pprz.sh build ANTON_MFC nps`), NOT a conf-path arg and NOT a `fast` mode. It also accepts the VSCode picker's single-arg `"AIRCRAFT (target)"`. `build_fw.sh` was removed.
- [2026-06-17] **In pprz.sh db, mkdir the `-o` output dir AFTER `clean_ac`** — clean_ac wipes `var/aircrafts/<AC>/`, and compiledb validates the output file's parent dir up-front, so creating it before clean_ac → "No such file or directory".

## Decision Log additions (2026-06-17 — build tooling overhaul)

- Removed build_fw.sh; pprz.sh is the single build tool (build/clean/rebuild/db/codegen/bootstrap). User wanted one build button (Shift+Cmd+B) with CMake-like incrementality + a few niche tasks.
- Replaced bear with compiledb for the IntelliSense DB (bear broken in-container).
- Did NOT fix the ANTON_MFC ap link error (nps_scope leak, bug-052) — out of scope of the build-tooling task; flagged to user.

## Key Learnings additions (2026-06-18 — dual-controller Phase 0)

- **MFC+INDI co-compile is purely a link-time duplicate-symbol problem now (bug-039 is moot).** Today's stabilization_mfc.c is self-contained (its own MFC_OUTPUTS-dimensioned g1g2 etc.; does NOT include stabilization_indi.h). The 39 strong globals defined in BOTH stabilization_indi.c and stabilization_mfc.c (g1g2, g1g2inv, g1g2_pseudo_inv, g1/g2, Bwls, act_is_servo, act_pref, actuator_state*, stab_thrust_filt, wls_stab_p, mu1/mu2, the lowpass filter arrays, …) collide at link. None are read cross-TU EXCEPT actuators_pprz (the NPS glue nps_autopilot_rotorcraft.c reads it via generated/modules.h). Fix = make the 38 file-local `static` in stabilization_mfc.c + drop their externs from stabilization_mfc.h; rename the 39th (actuators_pprz -> mfc_actuators_pprz).
- **set_rotorcraft_commands IS compiled for ANTON in both stabilizers** (STABILIZATION_*_COMMANDS is auto-generated from the airframe command/servo map, not hand-defined — my airframe grep found nothing but it's defined). Both are strong (non-WEAK) overrides -> dual link collision. Keep the active law's (INDI), neutralize the other.
- **generated/modules.h is included (transitively) by stabilization_mfc.c.** So a module's <header> file is pulled into EVERY control-task TU, including the stabilizer cores. A dual/wrapper module header must therefore NOT include a stabilizer's full header (it leaks that stabilizer's externs into the other stabilizer's TU). Declare only the specific symbol(s) the external consumer needs.
- **Dual-stabilizer wiring pattern that works:** one wrapper TU (modules/control_dual/control_dual_mfc_indi.c) owns the single stabilization_attitude_run/_enter and calls the cores directly (stabilization_indi_attitude_run active, stabilization_mfc_attitude_run into an inert mfc_shadow_cmd[]). The module makefile compiles stabilization_indi.c + stabilization_mfc.c + the wrapper, and does NOT compile the thin dispatchers stabilization_attitude_quat_{indi,mfc}.c (they each define the contested dispatch symbol). Module: conf/modules/stabilization_dual_mfc_indi.xml, selected via `<module name="stabilization" type="dual_mfc_indi"/>`, defines INDI_OUTPUTS/INDI_NUM_ACT AND MFC_OUTPUTS/MFC_NUM_ACT. Verified: ANTON_MFC nps links one simsitl; nm shows both cores + single dispatch.

## Decision Log additions (2026-06-18)

- **Dual-controller strategy = single shared Phase 0 (de-confliction) -> Shadow -> Handover, Shadow-first.** The two plans are sequential phases, not alternatives (both reuse Phase 0; Shadow is the safe validation gate for Handover). Implemented Phase 0 at the STABILIZATION level first (guidance left as stock INDI) as the smallest verifiable milestone: pure-INDI active path + MFC stabilizer shadowing. Guidance-level shadow and the Handover selector are later increments. Handover will use Option A (selector in the wrapper, GCS+RC), not the oneloop-style autopilot XML.
- **Neutralize MFC's colliding globals as `static` (not macro-rename).** Cleaner than `#define name name_mfc` because the latter fights header includes when a wrapper TU pulls in both stabilizer headers. Only actuators_pprz needed a real rename (it must stay an exported symbol for the NPS glue, but as INDI's; MFC's became static mfc_actuators_pprz).


- **PlotJuggler shared schema (2026-07-08): every feed is normalized before PlotJuggler.** `pj_json_relay.normalize_obj` rewrites any telemetry packet to root `"uav"` and renames MFC branches (`STAB_MFC`→`MFC_STAB`, `GUIDANCE_MFC`→`MFC_GUIDANCE`, `GUIDANCE_MFC_ACC2ATT`/`ACC2ATT`→`MFC_ACC2ATT`), so one layout (`plotjuggler_mfc.xml` / `plotjuggler_indi.xml`, curves `/uav/...`) works for any aircraft, sim or real. In SITL `sim_anton.py` forwards ONLY the NPS scope (via local port 9871 → normalize → PJ_HOST:PJ_PORT, default Mac:9870); ivy telemetry is captured to .jsonl but forwarded only with `--no-scope`. Real flight: run `pj_json_relay.py` standalone (normalizes by default, `--raw` to disable). pprzlink message names in messages.xml are UNCHANGED — the rename happens only at the relay layer. Firmware NPS_SCOPE_VAR strings now use the MFC_* names directly.

## Do-Not-Repeat additions

- [2026-06-18] **A wrapper/dual module's <header> must not `#include` a stabilizer's full header.** generated/modules.h (which includes that <header>) is itself included by the stabilizer .c files, so the externs leak in and collide with the other stabilizer's static globals. Declare only the needed symbol. (bug-068)
- [2026-06-18] **Don't assume STABILIZATION_*_COMMANDS is undefined just because the airframe XML doesn't mention it** — it's generated from the command/servo map, so the set_rotorcraft_commands overrides ARE compiled. (bug-067)

## Key Learnings additions (2026-06-18 — dual-controller Phase 1)

- **Adding a new pprzlink telemetry message needs a header regen step that `pprz.sh build` does NOT do.** The PPRZ_MSG_ID_* macros + per-msg headers live in `var/include/pprzlink/` and are generated ONCE by `make pprzlink_protocol` (→ pprzlink `pymessages`). Editing `sw/ext/pprzlink/message_definitions/v1.0/messages.xml` alone leaves them stale → `PPRZ_MSG_ID_X undeclared` in generated/periodic_telemetry.h. After editing messages.xml: `pprz_run -- make -C /workspace/paparazzi pprzlink_protocol`, THEN `pprz.sh build`. (bug-071)
- **The telemetry message-ID space (msg_class telemetry, id=1) is uint8 and nearly full — 249/255 used, max id reached 255.** Free gaps: 7, 13, 51, 57, 194, 195. New telemetry messages must reuse a gap (DUAL_CTRL took 194). Find gaps by extracting ids in the telemetry class block and diffing against 1..255.
- **A registered periodic-telemetry message only transmits if it's also listed in the aircraft's telemetry XML** (`conf/telemetry/<file>.xml`, here default_rotorcraft.xml) with a period. `register_periodic_telemetry` just binds a callback to a slot; the periodic scheduler fires it from the XML entry. ANTON_MFC uses `telemetry/default_rotorcraft.xml`.
- **`register_periodic_telemetry` allows up to TELEMETRY_NB_CBS callbacks PER message id** (telemetry.c:51-71, returns slot index or -1 if full). So two stabilizers both registering EFF_MAT_STAB/STAB_ATTITUDE doesn't crash — it double-sends (interleaved) that message. In the shadow stack, gate the inactive law's shared-message registrations (STABILIZATION_MFC_SHADOW define) so the active law owns them; the residual rides on the wrapper's own DUAL_CTRL.
- **Shadow-controller fidelity requires feeding the shadow the REAL operating point.** INDI/MFC are incremental: they compute Δu about the modelled actuator_state. A shadow law running into an inert buffer integrates its OWN discarded commands → its operating point drifts from the real motors (driven by the active law). Fix: each tick, after the active law runs (its get_actuator_state updates the real operating point), copy the active law's `actuator_state[]` into the shadow's and have the shadow's get_actuator_state COPY it (like the RPM-feedback branch) instead of integrating. Wired via INDI's exported `actuator_state[]` (indi.h) → `stabilization_mfc_set_shadow_actuator_state()` → MFC's get_actuator_state under STABILIZATION_MFC_SHADOW. Valid only when INDI_NUM_ACT==MFC_NUM_ACT in the same actuator order (`_Static_assert` guards it).
- **bug-050 and bug-052 are the SAME class and now FIXED:** sim-only globals (nps_th_cmd_z/nps_v_thrust in stabilization_indi.c; nps_scope_z_ref/z_sp from guidance_v.c) referenced unguarded in code compiled for `ap`. They blocked the dual stack's FLIGHT build. Fixed with `#ifdef SITL` guards at the assignment sites. ANTON_MFC now builds BOTH nps and ap. (bug-069/070)

## Decision Log additions (2026-06-18 — dual-controller Phase 1)

- **Phase 1 telemetry = one new DUAL_CTRL message (id 194), not extra NPS scope vars only.** Streams active-law id + committed cmd[] (active) + shadow cmd[] (MFC) + residual over the real PPRZ telemetry path (10 Hz in default_rotorcraft.xml), so the agreement band is observable in flight on the GCS, not just PlotJuggler in sim. The wrapper keeps an always-on `dual_committed_cmd[]` mirror (telemetry runs async from the control tick).
- **Fixed the two pre-existing SITL-guard bugs (050/052) as part of Phase 1** rather than deferring: the dual stack's flight (ap) target compiles the INDI core + guidance_indi, so these blockers now sit directly on the Phase-1→flight path. Trivial, well-understood `#ifdef SITL` fixes; no behaviour change (sim-only taps).

## Key Learnings additions (2026-06-18 — dual-controller Phase 2 handover)

- **The dual stack drives motors via TWO paths; a handover must serve both.** (1) ap/hardware: each law writes `cmd[act_to_commands[i]]`, then `set_rotorcraft_commands()` copies `cmd[]`→`commands[]`. (2) NPS sim for ANTON_MFC: `NPS_NO_MOTOR_MIXING=TRUE` and `NPS_USE_COMMANDS` UNDEFINED, so `nps_autopilot_rotorcraft.c:182` drives JSBSim from the GLOBAL `actuators_pprz[]` directly — NOT `commands[]`. INDI writes that global; MFC writes a renamed static `mfc_actuators_pprz[]`. So when MFC is the active law the wrapper must copy MFC's `cmd`→real `cmd[]` AND `mfc_actuators_pprz[]`→global `actuators_pprz[]`, else the sim keeps flying on INDI's output even though the GCS says MFC.
- **INDI/MFC `get_actuator_state()` integrates the law's OWN command (`indi_u`/`mfc_u`), not the committed/global one.** So the ACTIVE law's `actuator_state[]` is the real operating point only while it is active. Made the feed directional: active law integrates its own command, shadow law copies the active law's point. `stabilization_mfc_shadow_mode` (runtime, default TRUE) + new `STABILIZATION_INDI_SHADOW`-gated `stabilization_indi_shadow_mode`. The wrapper flips both on handover and runs the active law FIRST so its fresh point feeds the shadow.
- **GCS in-flight switch = datalink SETTING (msg id 4): index, ac_id, value(float).** The index is the flat settings.h `case N` for that `var`. For ANTON_MFC `dual_ctrl_active` is index **47**; value 0=INDI, 1=MFC. In the GCS it's the **DualCtrl → active_law** dropdown. A `<dl_setting handler="set_active" module="modules/control_dual/control_dual_mfc_indi">` generates `case 47: control_dual_mfc_indi_set_active(_value); _value = dual_ctrl_active;` — the handler is called BEFORE the var is touched, so a `if(want==dual_ctrl_active) return;` bumpless guard works.
- **A module's `<settings>` are collected ONLY if the module file is in the aircraft's `settings_modules=` whitelist** (an attribute in `conf_enac.xml`, normally auto-maintained by the Paparazzi Center). Editing a module XML to add settings is silently ignored until you add the file to that list. (bug-072)
- **Failsafe RC-loss force-to-INDI must be EDGE-triggered, not level.** In NPS there is no RC link so `radio_control.status` is permanently RC_REALLY_LOST; a level check would pin INDI and make the GCS selector untestable. Edge (transition into lost) snaps to INDI once on a real loss while leaving the selector free in sim.

## Decision Log additions (2026-06-18 — Phase 2)

- **Handover = Option A (selector in the wrapper) per the plan, GCS-driven.** Both laws run every tick (warm filters → instant switch); the selector only changes which buffer is committed + which actuator output reaches the motors, and re-`enter()`s the new law for bumpless transfer (oneloop-faithful). RC AUX gate / one-way panic deferred — GCS dropdown + edge RC-loss failsafe implemented now.
- **INDI shadow support gated behind `STABILIZATION_INDI_SHADOW`** (only the dual module defines it) so the stock `stabilization_indi.c` shared by every other ENAC airframe is byte-for-byte unaffected.


- **PlotJuggler shared schema (2026-07-08): every feed is normalized before PlotJuggler.** `pj_json_relay.normalize_obj` rewrites any telemetry packet to root `"uav"` and renames MFC branches (`STAB_MFC`→`MFC_STAB`, `GUIDANCE_MFC`→`MFC_GUIDANCE`, `GUIDANCE_MFC_ACC2ATT`/`ACC2ATT`→`MFC_ACC2ATT`), so one layout (`plotjuggler_mfc.xml` / `plotjuggler_indi.xml`, curves `/uav/...`) works for any aircraft, sim or real. In SITL `sim_anton.py` forwards ONLY the NPS scope (via local port 9871 → normalize → PJ_HOST:PJ_PORT, default Mac:9870); ivy telemetry is captured to .jsonl but forwarded only with `--no-scope`. Real flight: run `pj_json_relay.py` standalone (normalizes by default, `--raw` to disable). pprzlink message names in messages.xml are UNCHANGED — the rename happens only at the relay layer. Firmware NPS_SCOPE_VAR strings now use the MFC_* names directly.

## Do-Not-Repeat additions (2026-06-18 — Phase 2)

- [2026-06-18] **module.dtd element order is `(doc,settings_file*,settings*,dep?,header?,init*,…,makefile*)`** — put `<settings>` right after `<doc>`, not after `<makefile>` (bug-073).
- [2026-06-18] **Incremental codegen does NOT detect module-XML changes** (keys off airframe/conf hash). After editing a module XML (settings/defines), `touch` the airframe XML to force regen, or the generated settings.h/airframe.h stay stale.
- [2026-06-18] **`pprz_run` injects `-e CONF=""`** — sim/test scripts must use `os.environ.get("CONF") or "<default>"`, not `get("CONF", "<default>")`, or the empty string wins (bug-074).
- [2026-06-18] **An Ivy python script with a non-daemon Ivy thread won't exit after main() returns, and block-buffered stdout never flushes** → looks hung with an empty output file. Print with `flush=True` as you go (like sim_anton's snap()), and/or `os._exit()` at the end. Don't write end-only-print Ivy probes.

## Key Learnings additions (2026-06-19 — oneloop_mfc dual stack)
- To run two controllers of the same lineage (MFC forked from INDI) in one
  firmware, the clean approach is a self-contained module with file-local
  (static) globals + an `oneloop_<x>_` public API — NOT static-ifying the shared
  core or adding `#ifdef` shadow branches to the incumbent. `oneloop_mfc.{c,h}`
  (in modules/control_dual/) holds the whole MFC stack (attitude + guidance) this
  way and links beside stock INDI with zero edits to INDI.
- INDI guidance already supports being a non-owner of the framework plug symbols:
  `guidance_indi_run_mode()` is always compiled; the plug fns
  (`guidance_h/v_run_*`) are gated by `#if GUIDANCE_INDI_USE_AS_DEFAULT`. A
  guidance wrapper compiles guidance_indi.c with that FALSE, owns the plug
  symbols, and calls run_mode() directly. INDI's vertical output is the file-
  global `thrust_sp` (extern it; not in the header).
- Framework calls guidance_v_run* BEFORE guidance_h_run* each tick. The V call
  returns the previous tick's thrust (one-tick latency, inherent to the stock
  INDI plug); do the real compute in the H call.
- A module that carries a dl_settings panel must include (via its <header>) the
  header declaring the panel's variables, or generated settings.h fails to
  compile (bug-115). And `dl_setting handler="H" module=".../foo"` calls
  `foo_H()` — the function name must match exactly (bug-116).
- Single-owner symbol contract is self-checking: if two TUs define
  stabilization_attitude_run / guidance_h_run_pos, the LINK fails. A successful
  link proves single ownership; confirm with `nm simsitl` (the NPS binary is
  `var/aircrafts/<AC>/nps/simsitl`, not obj/nps.elf).

## Decision Log additions (2026-06-19)
- Chose independent per-layer switching (guidance{INDI|MFC} × stab{INDI|MFC}, 4
  combos) over fused "oneloop" presets, because the user's required combos include
  the cross pairs (g_mfc→s_indi, g_indi→s_mfc). "oneloop_mfc" therefore means a
  self-contained MFC *module*, not a fused single loop.
- Left stabilization_indi.c/.h untouched (dormant SHADOW #ifdefs, never enabled →
  functionally stock) rather than reverting them, to honour "INDI untouched" with
  zero risk. Kept the inert `actuator_state` export (wrapper reads it).
- INDI-as-shadow integrates its own command (no shadow branch compiled); accepted
  as a documented asymmetry — INDI is the trusted incumbent, its shadow output is
  telemetry-only.


- **PlotJuggler shared schema (2026-07-08): every feed is normalized before PlotJuggler.** `pj_json_relay.normalize_obj` rewrites any telemetry packet to root `"uav"` and renames MFC branches (`STAB_MFC`→`MFC_STAB`, `GUIDANCE_MFC`→`MFC_GUIDANCE`, `GUIDANCE_MFC_ACC2ATT`/`ACC2ATT`→`MFC_ACC2ATT`), so one layout (`plotjuggler_mfc.xml` / `plotjuggler_indi.xml`, curves `/uav/...`) works for any aircraft, sim or real. In SITL `sim_anton.py` forwards ONLY the NPS scope (via local port 9871 → normalize → PJ_HOST:PJ_PORT, default Mac:9870); ivy telemetry is captured to .jsonl but forwarded only with `--no-scope`. Real flight: run `pj_json_relay.py` standalone (normalizes by default, `--raw` to disable). pprzlink message names in messages.xml are UNCHANGED — the rename happens only at the relay layer. Firmware NPS_SCOPE_VAR strings now use the MFC_* names directly.

## Do-Not-Repeat additions (2026-06-19)
- [2026-06-22] **`$(SRC_FIRMWARE)` in a module `<header>` block is NOT expanded** — the generator writes the literal string into a C `#include`. Use the expanded literal: `dir="firmwares/rotorcraft/oneloop"`. Makefile variables are only safe in `<makefile>` blocks.
- [2026-06-22] **When moving a `.c` file, also update its own self-include** — updating wrapper includes is not enough; the moved `.c` includes its own header and needs updating too.
- Don't run `make pprzlink_protocol` / `libpprzlink.update` to regen messages —
  pprzlink is a submodule and `*.update` does `git submodule update`, REVERTING
  uncommitted messages.xml edits. Regen directly:
  `pprz_run -- make -C sw/ext/pprzlink pymessages MESSAGES_INSTALL=/workspace/paparazzi/var PPRZLINK_LIB_VERSION=2.0 VALIDATE_XML=FALSE` (bug-117/bug-096).

- oneloop_mfc has two operating modes via `oneloop_mfc_stab_active`: ACTIVE (integrates its own command in get_actuator_state) vs SHADOW (copies mfc_shadow_act_obs, populated only by the dual wrapper). Any STANDALONE use of oneloop_mfc MUST set oneloop_mfc_stab_active=true, else actuator_state stays zero and the allocator commands runaway thrust. Same trap applies if a new wrapper reuses oneloop_mfc. (bug-122, 2026-06-22)

- CRITICAL (verified in NPS sim): ANTON_DUAL appears to fly MFC->MFC but in the sim it does NOT — the dual stabilization wrapper's RC-really-lost failsafe forces dual_ctrl_active=INDI (NPS has no RC), so DUAL runs INDI stab with MFC as a passive shadow. To actually test/tune MFC->MFC you must run with RC present (or disable the failsafe); use standalone ANTON_MFC (oneloop_mfc) as the real MFC->MFC test bed. The MFC stabilizer's thrust path only supports pprz-int thrust (th_sp_to_thrust_i + Bwls); the physical-float path is a union type-pun bug. (bug-122, 2026-06-22)

## Key Learnings additions (2026-06-24 — MFC core flags now live)
- `mfc_siso_run` now honors `use_trajec_sp` (false → bypass reference-trajectory filter, track raw setpoint, zero accel feedforward) and a new `use_Kd` flag. `use_Kd=true` keeps wn=kp but sets damping ratio zeta=kd: `a = -2*kd*kp` (kd=1 == old critical damping); `use_Kd=false` keeps the hardcoded critical double pole. Both default safe in `mfc_siso_init` (trajec_sp=true, use_Kd=false) so existing airframes are byte-for-byte behavior-unchanged.
- Behavior note: `mfc_gx/gy/gz` set `use_trajec_sp=0`, so honoring the flag now makes guidance horizontal/vertical bypass the trajectory filter (intended, flagged to user). Stabilization + mfc_thrust set 1 → unaffected.

## Key Learnings (appended 2026-06-25)
- NPS `--rc_script` owns the AP **mode switch every RC frame** (`autopilot_static_on_rc_frame` reads the 3-way switch; script sets `nps_radio_control.mode` = MANUAL/-1, AUTO1/0, AUTO2/1). So a flight-plan NAV takeoff and an RC stick script cannot both run unless the script itself emits AUTO2(=NAV) during takeoff. The stock `takeoff` wrapper forces MANUAL for the first 8 s — bypass it for self-contained scripts.
- `AP_MODE_ATTITUDE_Z_HOLD`: horizontal = attitude sticks, vertical = `GUIDANCE_V_MODE_HOVER` which captures z on entry and **ignores the throttle stick**. So you can't "throttle up to climb" in Z-hold; climb must happen in another mode (NAV) first.
- Added `radio_control_script_fp_takeoff_zhold` (index 5) for ANTON_MFC: AUTO2/NAV climb window (NPS_FP_CLIMB_TIME=15s) then AUTO1/ATTITUDE_Z_HOLD with a cycled roll/pitch/yaw step schedule. ANTON_MFC airframe maps AUTO1=ATTITUDE_Z_HOLD.

## Key Learnings (appended 2026-06-25 — host-side flashing)
- Hoops_111_MFC → airframe `hoops_111_indoor.xml` → board **tawaki_2.0** = STM32H743xI (Cortex-M7, Device ID 0x450), flash base 0x08000000. Default `FLASH_MODE ?= DFU-UTIL` (`conf/boards/tawaki_2.0.makefile:60`). ITCM mode (USE_ITCM=1) flashes 0x00200000 and CANNOT use dfu-util (SWD/ST-Link only); default USE_ITCM=0.
- Flashing happens on the **Mac/host**, never the sandbox (USB device is physical). Build runs in container → `.elf/.bin/.hex` land in `var/aircrafts/<AC>/<TARGET>/obj/`.
- DON'T flash the `.elf` with STM32CubeProgrammer — its zero-size RAM segments (0x20000000, 0x2402xxxx) trigger "File corrupted. Two or more segments define the same memory zone". Flash `.hex` or `.bin`+addr instead.
- `make ... -f Makefile.ac ap.upload` fails on the lean Mac with "No rule to make target ap.upload" — NOT because the rule is missing (it's at Makefile.ac:220) but because its prereq chain `%.upload→%.compile→%.ac_h→$(GENERATORS)/gen_aircraft.out` needs the OCaml codegen binary the Mac doesn't have; GNU make drops the unsatisfiable pattern rule. Bypass it: call the airborne upload target directly: `make -C $PPRZ/sw/airborne TARGET=ap AIRCRAFT=<AC> PAPARAZZI_SRC=$PPRZ PAPARAZZI_HOME=$PPRZ upload` (needs only the generated `var/aircrafts/<AC>/Makefile.ac` + the bin). Pick flasher with `FLASH_MODE=` (STLINK→st-flash, DFU_CUBE→STM32_Programmer_CLI, default DFU-UTIL→dfu-util). For DFU_CUBE on Mac also override `CUBE_PROGRAMMER=$(which STM32_Programmer_CLI)` (makefile hardcodes /usr/local path).


- **PlotJuggler shared schema (2026-07-08): every feed is normalized before PlotJuggler.** `pj_json_relay.normalize_obj` rewrites any telemetry packet to root `"uav"` and renames MFC branches (`STAB_MFC`→`MFC_STAB`, `GUIDANCE_MFC`→`MFC_GUIDANCE`, `GUIDANCE_MFC_ACC2ATT`/`ACC2ATT`→`MFC_ACC2ATT`), so one layout (`plotjuggler_mfc.xml` / `plotjuggler_indi.xml`, curves `/uav/...`) works for any aircraft, sim or real. In SITL `sim_anton.py` forwards ONLY the NPS scope (via local port 9871 → normalize → PJ_HOST:PJ_PORT, default Mac:9870); ivy telemetry is captured to .jsonl but forwarded only with `--no-scope`. Real flight: run `pj_json_relay.py` standalone (normalizes by default, `--raw` to disable). pprzlink message names in messages.xml are UNCHANGED — the rename happens only at the relay layer. Firmware NPS_SCOPE_VAR strings now use the MFC_* names directly.

## Do-Not-Repeat (2026-06-25)
- Don't claim NPS flight behavior works from compile-only. The headless CSV/debug logs from sim_anton.py read all-zero/frozen-t in this sandbox (its in-container Ivy client doesn't bind telemetry), so they are NOT a witness. Verify by probing the Ivy bus directly: run a python ivy listener inside a sibling `--network host` container on 127.255.255.255:2010 and decode ROTORCRAFT_STATUS (ap_mode field is the 6th payload value; 9=ATTITUDE_Z_HOLD) + ROTORCRAFT_FP (up*0.0039063 = metres).
- rc_script 5 result: takeoff via NAV + ATTITUDE_Z_HOLD entry CONFIRMED (motors on, armed, in_flight, ap_mode=9). USER CONFIRMED THE FEATURE WORKS in their own test (2026-06-25). My headless probe saw a continuous altitude climb (~164->195m) but that was most likely a probe/altitude-reference misread or transient, NOT a real defect - do not assert it's broken. If altitude hold ever does misbehave, trace whether stabilization_mfc consumes guidance_v's HOVER thrust or its own GUIDANCE_MFC_GZ_* path.
- MFC internals are ALREADY registered in NPS_SCOPE by their owning modules — don't propose adding them or editing `nps_scope_state.c` (it is generic). Stab: `stabilization_mfc.c:370-391` → `mfc/{roll,pitch,yaw}/{sp,sp_traj,meas,err,fk,cmd}`, `mfc/act`, `wls/{v,u}`. Guidance: `guidance_mfc.c:168-192` → `mfc_g/{x,y,z}/{sp,sp_traj,meas,fk,cmd}`, `mfc_g/acc2att/*`. These key names ARE the unified JSON schema contract. (Always grep for existing `NPS_SCOPE_VAR` before claiming a signal is missing.) Note: guidance_mfc has scope vars but NO downlink/SD telemetry message — a `GUIDANCE_MFC` pprzlink message is still needed for the flight/SD path.

## Key Learnings (appended 2026-06-25 — MFC flight-test enablement)
- `GUIDANCE_MFC` telemetry message now EXISTS: **id 57**, mirrors STAB_MFC for the
  guidance layer (per-axis sp/meas/err/fk + cmd). `send_guidance_mfc()` +
  register in `guidance_mfc.c` (gated `#if PERIODIC_TELEMETRY`). Free telemetry
  ids were 7/13/51/57 on feat/shadow-handoff (251/255 used). Don't forget the
  pprzlink header regen (`make ... pymessages`) before building — `pprz.sh build`
  does NOT regen, you get `PPRZ_MSG_ID_GUIDANCE_MFC undeclared`.
- **`stabilization_mfc.c` consumes `GUIDANCE_MFC_TILT_PPRZ_SCALE` /
  `GUIDANCE_MFC_TWIST_PPRZ_SCALE`** (not just guidance) — they set the roll/pitch
  (TILT) and yaw (TWIST) virtual-command u_min/u_max clamps = ±MAX_PPRZ/scale.
  `THRUST_PPRZ_SCALE` sets the gz clamp in guidance_mfc.c. All three are
  structural: derive from the airframe G1 as 1000/(2·roll_eff), 1000/(2·yaw_eff),
  1000/(4·thrust_eff). Convention numerator is 1000 (matches anton_mfc).
- **A stab=mfc airframe needs `STABILIZATION_MFC_COMMANDS` ONLY if it uses named
  motor commands.** `stabilization_mfc.c:856` writes the global `actuators_pprz[]`
  unconditionally; it ALSO sets `cmd[act_to_commands[i]]` only when COMMANDS is
  defined. Airframes whose command_laws read `actuators_pprz[0..3]` directly
  (Hoops, INDI-style) need NO COMMANDS map. anton_mfc DOES (it uses `@FR` laws +
  COMMAND_FR). bug-122's note that stab=mfc "doesn't compile" is stale — Hoops
  builds both ap+nps with stab=mfc+guidance=mfc.
- **`flight_recorder` module `<depends>logger_sd_chibios,pprzlog</depends>`** — one
  `<module name="flight_recorder"/>` line pulls in the whole SD binary-log chain;
  no need to declare logger_sd_chibios separately unless overriding SDLOG_* config.
  It logs the telemetry `FlightRecorder` process; TELEMETRY_FREQUENCY defaults to
  PERIODIC_FREQUENCY (500 Hz cap). Tawaki v2 uses default SDLOG_SDIO=SDCD1.
- **Unified MFC analysis schema is live**: `tools/sdlog2scope.py` maps a decoded
  `.data` (sd2log output) → NPS_SCOPE ndjson keyed `mfc/*`,`mfc_g/*`,`wls/*`,
  `truth/*` (truth from MFC measured attitude/position — no JSBSim truth in
  flight). `analyze_mfc.py` auto-detects CSV vs scope-JSON; `tune_mfc.sh` gained
  `--scope FILE` + `--flight LOG.data`. Unit gotcha: scope truth angles are
  DEGREES but analyze_mfc's phi/theta path applies *R2D, so the JSON loader feeds
  radians (truth_deg/R2D); errors/fk/cmd pass through (radians/unitless).

## Decision Log additions (2026-06-25)
- Defaulted Hoops_111_MFC to the FULL MFC stack (guidance+stab=mfc, == anton_mfc)
  rather than the plan's attitude-only first config, because it's the proven
  buildable config and "everything in place" was the goal; attitude-only is a
  documented 1-line guidance→indi swap. The cross pair guidance=indi+stab=mfc
  risks the bug-122 thrust-unit union path, so it's NOT the default.


- **PlotJuggler shared schema (2026-07-08): every feed is normalized before PlotJuggler.** `pj_json_relay.normalize_obj` rewrites any telemetry packet to root `"uav"` and renames MFC branches (`STAB_MFC`→`MFC_STAB`, `GUIDANCE_MFC`→`MFC_GUIDANCE`, `GUIDANCE_MFC_ACC2ATT`/`ACC2ATT`→`MFC_ACC2ATT`), so one layout (`plotjuggler_mfc.xml` / `plotjuggler_indi.xml`, curves `/uav/...`) works for any aircraft, sim or real. In SITL `sim_anton.py` forwards ONLY the NPS scope (via local port 9871 → normalize → PJ_HOST:PJ_PORT, default Mac:9870); ivy telemetry is captured to .jsonl but forwarded only with `--no-scope`. Real flight: run `pj_json_relay.py` standalone (normalizes by default, `--raw` to disable). pprzlink message names in messages.xml are UNCHANGED — the rename happens only at the relay layer. Firmware NPS_SCOPE_VAR strings now use the MFC_* names directly.

## Do-Not-Repeat additions (2026-06-25 — SITL test of MFC enablement)
- **`ins ext_pose` (OptiTrack) gets NO state feed in stock NPS** → an airframe on
  ext_pose flies on a garbage estimate in SITL (Hoops_111_MFC NPS: `me_z` started
  at -44 m, attitude diverged 60-70°). It is NOT an MFC/controller fault. No NPS
  ext_pose sender exists; `ins_ext_pose.c` has no SITL path; only hoops_111_indoor
  + hexa_tilted_motors use ext_pose, every other ENAC quad uses `ekf2` for nps. To
  test MFC *flight behaviour* in SITL, override the **nps target** INS to `ekf2`
  (keep `ext_pose` on `ap`), or wire a mocap feed. Don't chase MFC gains for an
  ext_pose airframe's NPS divergence until the state estimate is valid.
- **Tap MFC telemetry in NPS via a sibling `--network host` container Ivy listener**
  on `127.255.255.255:2010` binding `(\d+ STAB_MFC .*)` / `(\d+ GUIDANCE_MFC .*)`.
  `link` re-emits firmware binary PPRZ as Ivy text `<ac_id> MSG f1 f2 …`. Reformat
  to paparazzi `.data` (`<t> <ac_id> MSG fields`) — same format `sd2log` emits — to
  exercise tools/sdlog2scope.py + analyze_mfc.py without hardware. VERIFIED 2026-06-25:
  STAB_MFC@20Hz/19f + GUIDANCE_MFC(id57)@10Hz/15f decode, full analysis chain runs.

## Key Learnings additions (2026-06-29 — NPS-scope ↔ ivy PlotJuggler parity)

- The NPS scope JSON now mirrors the ivy-server tree: every datagram is `{ "<AIRFRAME_NAME> (sim)": { "TRUTH":{…}, "<MSG>/<field>":v, … }, "timestamp": fdm.time }`. ivy uses root `"<name> (<id>)"` (Hoops_111_MFC → `Hoops_111_MFC (111)`, NOT 177 — the plan's 177 was stale ANTON). Registration keys are the message schema: `STAB_MFC/sp_phi`, `GUIDANCE_MFC/sp_traj_x`, `WLS_U/u/u_0`, `WLS_V/v/v_0`, acc2att in its own `ACC2ATT/*` branch. nps_scope_state.c extras uppercased to EST/SENSORS/SP/MODE. nps_scope.c includes generated/airframe.h for AIRFRAME_NAME (a string literal → usable in the format via literal concat).
- pprzlink array fields render in PlotJuggler as `field/field_N`, so the scope prefix to match ivy is `WLS_U/u/u_` → `WLS_U/u/u_0` (tree WLS_U→u→u_0). PlotJuggler's UDP/JSON parser prepends a leading `/` to the top-level key, so series names are `/Hoops_111_MFC (sim)/STAB_MFC/sp_phi`.
- STAB_MFC u0..u3 are the WLS solution `mfc_u[]` (float), not the int16 `actuators_pprz` — both the send site and the scope register `mfc_u`.
- To regenerate pprzlink C message headers after editing the submodule messages.xml WITHOUT reverting edits: `make -C sw/ext/pprzlink pymessages MESSAGES_INSTALL=$PAPARAZZI_HOME/var PPRZLINK_LIB_VERSION=2.0 VALIDATE_XML=FALSE` (uses gen_messages.py; writes var/include/pprzlink/* + copies var/messages.xml; no git submodule update). `pprz.sh build` does NOT auto-regen these on a messages.xml change.
- Free pprzlink telemetry ids are scarce (only 7, 13, 51 free as of this branch). GUIDANCE_MFC id=57 is UNIQUE within the telemetry class; TARGET_POS id=57 is in the *datalink* class — different class, so NO real collision. Did NOT reassign GUIDANCE_MFC (would burn a scarce id + break compatibility for no benefit), contra the plan's id-57 item.

## Decision Log additions (2026-06-29)
- Kept GUIDANCE_MFC at telemetry id 57: per-class id space means the datalink TARGET_POS(57) does not collide. Reassigning would waste one of only three free telemetry ids and is unnecessary. Flagged to user.
- Made guidance_mfc.c nps_* acc2att globals + their assignment unconditional (were #ifdef SITL) so the new GUIDANCE_MFC_ACC2ATT flight-test (ap) telemetry/SD message can read them; only the NPS_SCOPE_VAR registrations stay under SITL.

## Key Learnings additions (2026-07-01 — flight-test streaming fix + kd/use_Kd rollout)

- **`server.ml`'s `udp_sockaddr` was bound at module-load time, before `Arg.parse` ran** — a top-level `let udp_sockaddr = Unix.ADDR_INET(...)` evaluates immediately when the .ml file loads, capturing the *default* `!udp_json_stream_addr`/`_port` refs, not whatever `-udp_json_stream_addr` later sets. So the flag was silently a no-op for the entire life of this feature (matches the now-corrected `plotjuggler-server-udp-json` memory claim that "addr/port flags are dead"). Fix: make it a `ref`, and re-resolve it in `main()` right after `Arg.parse` returns. Verified end-to-end against real flight-test hardware — PlotJuggler now receives the stream at the real host IP, not just 127.0.0.1.
- **`mfc_core.c`'s feedforward acceleration was wrongly zeroed whenever `use_trajec_sp=false`.** The trajectory filter and the feedforward derivative are logically separate: even when raw-setpoint tracking is selected (guidance axes normally set `use_trajec_sp=0`), the double-difference feedforward term should still be computed from `setpoint_trajec` history (which becomes `[raw_sp, raw_sp, raw_sp]`-ish under bypass) — it was being hardcoded to `0.f` in the false branch instead, which suppressed feedforward entirely on any step/ramp input for every guidance axis. Fixed by hoisting the derivative computation out of the if/else so it always runs.
- **kd/use_Kd are now first-class tuneable parameters on every MFC axis** (stabilization roll/pitch/yaw, guidance gx/gy/gz, and oneloop_mfc's self-contained copies of both) — compile-time `#ifndef` default + airframe-XML override + runtime `dl_setting`, matching the exact pattern used for kp/alpha/traj (see the 2026-06-30 entry below). `guidance_indi.c`'s `mfc_thrust` (the deprecated `GUIDANCE_INDI_THRUST_MFC` path, superseded by `guidance_mfc`) was deliberately left untouched — the file's own comment says it's hardcoded on the way out.
- Restoring `stabilization_mfc.c`'s roll/pitch `kp` to read from their macros (they'd been hardcoded to literals `6`/`8` during flight tuning) required bumping the module's `PITCH_PROPORTIONAL_GAIN` default from `6.` to `8.` to keep compiled behavior identical — always check the module XML default matches a literal before restoring the macro, or you silently change tuning.

## Decision Log additions (2026-06-30 — MFC runtime tuneability)
- **Allocator + thrust-packaging mode are now RUNTIME dl_settings, not `#if` defines.** Converted `STABILIZATION_MFC_ALLOCATION_PSEUDO_INVERSE` → `bool stabilization_mfc_use_pseudo_inverse` (and `oneloop_mfc_use_pseudo_inverse` in oneloop_mfc.c), and `GUIDANCE_MFC_THRUST_TO_PPRZ` → `bool guidance_mfc_thrust_to_pprz`. Pattern follows `mfc_use_adaptive`: keep the `#ifndef…#define…FALSE` default, init the bool from the macro, branch with a plain `if`. Required **de-guarding both allocator paths** (WLS state `wls_stab_p`/`act_pref`/`set_wls_settings` AND `calc_g1g2_pseudo_inv`/`g1g2_pseudo_inv`) so both compile; `calc_g1g2_pseudo_inv()` is now called unconditionally in init + lms so a live switch has no uninitialised state. Toggles are non-persistent (a bad allocator never saved to flash). Why: flight-test tuning without re-flash.
- **`wls_alloc.h` no longer pulls in `stabilization_indi.h`** (bug-039 was an old version) — it only includes airframe.h + telemetry.h, and stabilization_mfc.c/oneloop_mfc.c already `#include` it unconditionally. So de-guarding the WLS path is link-safe.
- **Left `stabilization_mfc.c:779` (`#if GUIDANCE_MFC_THRUST_TO_PPRZ`) compile-time on purpose.** thrust_to_pprz=TRUE selects the stock-INDI downstream (stabilization_mfc.c not compiled); when the MFC stabilizer IS compiled the mode is FALSE. Making it runtime would force a fragile cross-module extern into a TU that may build without guidance_mfc.o. NOT an oversight.
- **No edits to the `dual_*` module XMLs.** Their MFC side is `oneloop_mfc` (the dual modules `<depends>oneloop_mfc</depends>` and compile only stabilization_indi.c/guidance_indi.c + oneloop), so a dual airframe inherits oneloop_mfc.xml's panel (renamed shortnames + `alloc_pseudo_inv`). The dual panels' own shortnames (active_law/active_guid/kp/kd/max_bank) are already unique.
- **Shortname scheme unified to `<axis>_<field>`** across stabilization_mfc/oneloop_mfc/guidance_mfc panels: traj/intwin/alpha/kp/cfilt, axis ∈ {roll,pitch,yaw} | {gx,gy,gz}. Replaced the colliding TT/IW/ALP/KP/CF in stabilization_mfc.xml and the TT/ALP/KP suffixes in oneloop_mfc.xml. Build-verified: ANTON_MFC ap+nps (stab+guidance mfc), ANTON_DUAL nps (oneloop) — all link; new vars present in generated/settings.h.

## Key Learnings additions (2026-07-03 — sim_anton.py native rework)

- **Ivy ground messages (`JUMP_TO_BLOCK`, `DL_SETTING`) are the real native command path, not hand-rolled pprz frames over UDP.** `server.ml` (`ground_to_uplink`) binds these by name and re-encodes them into the actual binary datalink frame itself — a python script only needs `pprzlink.ivy.IvyMessagesInterface` + `PprzMessage("ground", "JUMP_TO_BLOCK"/"DL_SETTING")` (see `sw/ground_segment/python/multi/collective_tracking_control/ctcStartSync.py` for the reference pattern). No STX/checksum/UDP:4243 needed — `sim_anton.py` used to hand-roll this (bug of reinvention, not a real bug) and it's gone now.
- **`DL_SETTING`/`JUMP_TO_BLOCK`'s `ac_id` field is numeric (as a string), not the aircraft name** — `server.ml` uses it as the `aircrafts` Hashtbl key (same key telemetry sender-name int uses). So a python control script still needs the numeric ac_id from the conf XML even though launching simsitl itself never did (simsitl is addressed by AC_NAME, not ac_id).
- **`sw/lib/python/settings.py`'s `PprzSettingsParser.parse(var/aircrafts/<AC>/settings.xml)` gives name→index lookup** (`PprzSettingsGrp.__getitem__` by shortname, raises `AttributeError` if unknown) — this replaces any hand-maintained flat `dl_setting` index constant (e.g. the old `DUAL_CTRL_IDX=47` in `sim_anton.py`). Only present on dual-controller builds (`dual_ctrl_active`/shortname `active_law`); ANTON_MFC/Hoops_111_MFC (standalone MFC, no dual wrapper) have no such setting — confirmed empty via grep. A control script must handle "setting doesn't exist on this airframe" gracefully, not just index-out-of-range.
- **`pprzsim-launch` (`sw/simulator/pprzsim-launch`) is NOT a superset of invoking `simsitl` directly.** It only translates a handful of flags (`fg_host/rc_script/norc/ivy_bus/js_dev/spektrum_dev/time_factor/fg_fdm/nodisplay`) via `optparse`, which errors on ANY unrecognized long option — so `--scope_host/--scope_port/--scope_decim` (the in-process NPS scope emitter flags) and a gdbserver wrap CANNOT be passed through it. For a workflow that defaults to using the scope, invoke `simsitl` directly (its own native CLI) instead — don't force `pprzsim-launch` in just because a plan says "use the launcher."
- **`pprzlink.ivy.IvyMessagesInterface.subscribe(callback, PprzMessage(class, name))` gives `callback(ac_id, msg)` with named field access** (`msg["phi"]`, `msg["cmd_roll"]`, etc.) — much cleaner than manually `msg.split()`-parsing raw Ivy text (what `sim_anton.py` used to do for every bind). Prefer this for any new Ivy-consuming python tool in this repo.
- **`pj_json_relay.sanitize()` is importable** (`from pj_json_relay import sanitize`) — reusable anywhere a `server.ml` UDP/JSON telemetry stream needs repairing before use, not just its original standalone-relay use case. Used it in `sim_anton.py`'s new telemetry-capture sink (server → localhost → sanitize+tee-to-file → forward to real PlotJuggler host).
- **`timeout -s KILL N ./sim.sh ...` does not stop the container** when `sim.sh` uses `--network host` (sbx sandbox path) — `timeout` only signals the `docker run` client process, and the container (no `-d`, but host networking means no PID namespace cleanup on client death) keeps running orphaned, holding UDP ports. A second smoke-test run then gets `OSError: Address already in use` on any port the first orphan still owns. Always `docker ps --filter ancestor=paparazzi-build:latest` + `docker kill` after a timeout-bounded headless test in this repo.
- **Reconfirmed the pre-existing sandbox Ivy limitation** (see the 2026-06-25 Do-Not-Repeat entry): a `.jsonl`/CSV capture written by a python script running *inside* the same ephemeral container as `server`/`link`/`simsitl` still comes back empty/frozen after ~30s in this sandbox, even though the underlying mechanism (server's own UDP/JSON stream, not even Ivy) is architecturally sound. Don't take an empty capture file as evidence a rewrite is broken — the user confirmed the equivalent old-script behavior works fine from their own machine.

## Key Learnings additions (2026-07-16 — MFC SI units)

- **MFC stack unit contract is SI (2026-07-16, supersedes the 2026-06-11 "thrust-unit conversion lives in guidance" decision).** Virtual commands: N·m (roll/pitch/yaw), N (thrust); G1/G2 in the airframe are SI-per-PPRZ-count ×1000 (MFC_G_SCALING) = INDI-identified accel rows × MODEL inertia/mass. Thrust setpoints are decoded in stabilization_mfc.c via the typed th_sp_to_thrust_i()/th_sp_to_incr_i() accessors keyed on the setpoint's own type/format tags — PPRZ-int (RC-direct/ATTITUDE_DIRECT) and normalized float (guidance) both resolve to counts, then Newtons via the SI G1 thrust row. GUIDANCE_MFC_THRUST_TO_PPRZ / THRUST_PPRZ_SCALE / TILT/TWIST_PPRZ_SCALE and the guidance_mfc_thrust_to_pprz runtime bool are GONE. Clamps: stab torque = ±0.5·MAX_PPRZ·Σ|g1g2 row| computed at init; gz = [−GZ_MAX_THRUST, 0] N; gx/gy = ±g·sin(max_bank).
- **Rescaling alpha, G1 and WLS Wv by the same physical constants (alpha/=I, G1*=I, Wv/=I per axis) preserves the closed loop and the WLS cost balance exactly** — the constants cancel; only absolute meanings (hover=−m·g, clamps) change. So placeholder mass/inertia don't alter SISO behavior, but do set the thrust normalization and envelopes.
- **hoops_111_mfc.xml MODEL section holds MASS/INERTIA_XX/YY/ZZ; values are the JSBSim simple_x_quad_ccw constants (0.381 kg / 0.0068 / 0.0136) — placeholders, TODO bench-measure.** JSBSim's motor model is ~1.46× more effective than the INDI-identified −0.7 thrust row: in NPS the hover gz command reads ≈−2.55 N instead of −m·g=−3.74 N (F-estimator absorbs it). Don't chase that as a units bug.
- **calc_g1g2_pseudo_inv's fixed ×1000 conditioning assumed accel-scale rows; now adaptive (normalize by max element of G·Gᵀ).** SI rows are ~I (≈400×) smaller — a fixed factor would have broken float inversion.
- **anton_mfc.xml is stale on this branch:** still accel-scaled G1 + deleted defines (inert). Port the MODEL pattern before flying ANTON_MFC.

## Do-Not-Repeat additions (2026-07-16 — SI refactor review)

- (2026-07-16) **Do NOT rescale G1/G2 in airframe XMLs to other units.** User correction: G1/G2 stay in the identified acceleration convention (what people understand and measurement tools produce); any unit conversion (e.g. to N·m/N) belongs in code (sum_g1_g2 multiplies rows by STABILIZATION_MFC_INERTIA_*/MASS). Same principle likely applies to other identified quantities: keep the measured form in config, convert in code.
- (2026-07-16) **Don't rewrite stock-INDI-mirrored code paths that the current stack doesn't exercise.** The MFC THRUST_INCR_SP branch mirrors stabilization_indi.c and guidance_mfc emits THRUST_SP — user rejected my rework of the increment branch; keep unused mirrored branches byte-similar to stock so diffs against INDI stay readable.
- (2026-07-16) **Don't over-restrict GCS dl_setting slider ranges to the currently-valid value domain** (e.g. gz u_min/u_max clamped to negative-only). Sliders are UI brackets, not safety logic — keep them permissive; physical clamping lives in the computed defaults.

## Key Learnings additions (2026-07-20 — raw log -> PlotJuggler CSV)

- A Paparazzi `.log` is XML whose `<protocol>/<msg_class>/<message>` section carries the **full
  message + field definitions for that flight**. Parse field names from the log itself — never
  hardcode a message field table (the old sdlog2scope.py did, and drifted from messages.xml:
  it was missing `sp_traj_*` on STAB_MFC/GUIDANCE_MFC).
- `.data` rows are `time ac_id MSG field1 field2 …`, values **raw/unscaled**, array fields
  comma-joined into a single token, string/enum fields quoted (`"stab"`). The GCS CSV export
  keeps these raw values verbatim — only `GPS_lat(deg)`/`GPS_long(deg)` are scaled (GPS_INT
  lat/lon × 1e-7).
- The GCS-exported flight CSV is **resampled onto a fixed grid** (4 Hz here), so its timestamps
  do not exist in the `.data`. To diff a full-rate conversion against it, compare each reference
  row to the nearest *preceding* raw row, not an exact timestamp match.
- `UTC` column = `floor(time_of_day) + t` formatted UTC — the exporter truncates the
  `time_of_day` attribute to whole seconds (using the fraction puts you ~0.7 s off).

## Decision Log additions (2026-07-20)

- sdlog2scope.py was rewritten to emit the **same `/uav/<BRANCH>/<field>` wide CSV as
  convert_sd_to_pj.py** rather than its old NPS_SCOPE ndjson schema — one output contract for
  flight data, and it imports `BRANCH_MAP` from convert_sd_to_pj.py so the renaming has a single
  source of truth. Value-verified: 0 mismatches over 13,005 cells vs a known-good `_pj.csv`.
- Default row policy is one row per distinct timestamp with forward-fill (full message rate,
  ~18k rows vs the exporter's 153). `--trigger MSG` restores one-row-per-message if a single
  message's cadence is wanted.

## Key Learnings additions (2026-07-22 — launcher cleanup)

- **The two telemetry feeds have different packet SHAPES, not just different branches.**
  The NPS scope puts every registered variable in every datagram (fixed key set, decided at
  init); server's ivy stream sends ONE message per datagram. Any code that infers a schema
  from "the first packet" works in sim and silently truncates in flight — this cost an
  11-column CSV where 162 were expected. Buffer a warmup window (live) or two-pass (offline).
- **Which branch comes from which feed** (the real reason PlotJuggler tabs look empty):
  scope-only = `TRUTH/*`, `EST/*`, `SENSORS/*`, `SP/*`, `MODE/*`, `indi/*`, `mfc/z_*`;
  ivy-only = `ROTORCRAFT_FP`, `STAB_ATTITUDE`; both = `MFC_STAB|GUIDANCE|ACC2ATT`, `WLS_U|V`.
  In a real flight only the MFC + WLS tabs draw unless something bridges the gap.
- **`ROTORCRAFT_FP` is raw fixed-point int32 on the wire.** server.ml emits the stored value,
  NOT the `alt_unit_coef` scaling in messages.xml. Scales: position 1/2^8 m, velocity
  1/2^19 m/s, angles 1/2^12 rad. It is **ENU** (east/north/up) while `EST/*` is **NED** from
  `stateGetPositionNed_f()`, so the vertical axis must be NEGATED, not just scaled.
- **The whole `/uav` schema is radians as of today.** `nps_scope.c` / `nps_scope_state.c` used
  to convert to degrees while the controller branches published radians; overlaying them made
  MFC curves look like flat lines. If you ever add a scope variable, do not convert units.
- **`IvyMessagesInterface(start_ivy=True)` starts the bus inside `__init__`** — you can remove
  every `subscribe()` call without breaking `send()`.
- **`pprzsim-launch` cannot replace the direct `simsitl` invocation.** It is a thin `execv`
  wrapper with a fixed flag list (fg/rc_script/norc/js_dev/spektrum/ivy_bus/time_factor/
  nodisplay) and no passthrough — no `--scope_*`, no gdbserver wrap. Calling `simsitl`
  directly IS the native CLI. (An old plan recommended the swap; it was wrong, now retracted.)
- **Headless SITL in the sandbox does not take off** (pre-existing, 2026-07-22): `TRUTH/agl`
  pinned at 0.098 m, `WLS_U` ~0 vs ~1918 in a working run, `MODE/ap`=13 NAV, GPS fix present,
  but `MODE/nav_v`=0 and `SP/guidance/v_z`≈0 — no climb ever commanded. Not caused by the
  telemetry/logging rework; the pre-change script produces an empty capture here too.

## Decision Log additions (2026-07-22)

- **This repo is now the programmatic/headless path only.** Interactive sim and flight run
  from the Paparazzi GUI control panel (`conf/userconf/ENAC/control_panel_mfc.xml`). Anything
  the GUI does well (strips, settings panels, live plots) must NOT be reimplemented here.
  Chosen because the user now runs Paparazzi properly in a Linux VM; tenet was "if normal
  paparazzi does it, strip it".
- **One canonical on-disk format: the `/uav` wide CSV**, produced by BOTH `tools/scope2csv.py`
  (sim) and `tools/sdlog2scope.py` (flight). One PlotJuggler layout, one analyser, both feeds.
- **Layouts state their coverage in the tab name** (`INDI (sim only)`, `States (sim truth)`)
  rather than silently drawing blank on the flight feed. Chosen over maintaining separate
  sim/flight layout files.
- **`--nav` takes block NAMES, not ids** (resolved from the generated `flight_plan.xml`), so a
  flight-plan edit can't silently retarget the sequence.

## Do-Not-Repeat additions (2026-07-22)

- (2026-07-22) **Never grade a signal without checking it is alive.** `analyze_mfc.py` reported
  `GOOD ✓` for a flight whose `MFC_STAB/*` was identically zero (MFC wasn't the active law).
  An all-zero channel is "no signal", and nan must not render as a full progress bar — all
  nan comparisons are False, so a naive clamp shows "pegged at maximum" for missing data.
- (2026-07-22) **Don't drop an `#include` just because the symbol you removed came from it.**
  Removing `DegOfRad()` uses from nps_scope_state.c tempted a removal of `#include "std.h"`,
  which provides much more; restored immediately.

## Do-Not-Repeat additions (2026-07-30)

- (2026-07-30) **When "the sim doesn't take off" — check `MODE/motors_on`/`MODE/arming` and
  `MODE/nav_v` (nav.vertical_mode) via NPS scope BEFORE suspecting the flight plan.** For
  ANTON_MFC, `--nav` (default "Start Engine,Takeoff") printed the block-jump lines locally but
  telemetry showed `nav.vertical_mode` pinned at MANUAL(0) and motors_on/arming/in_flight
  pinned at 0 for 90s straight — the aircraft never left its FIRST block (Wait GPS, which
  calls NavKillThrottle()). Confirmed with a debug Ivy subscription to the aircraft's own
  `ROTORCRAFT_NAV_STATUS` downlink (1.6s period): it never arrived once in 40s, meaning Ivy
  traffic isn't flowing between sim_anton.py and the firmware/server at all in this sandboxed
  docker setup — NOT the vto_survey flight plan (block names/exceptions all check out fine by
  static read) and NOT `modules/checks/preflight_checks` (not compiled into ANTON_MFC at all —
  don't assume that module gates arming without checking the airframe's module list first).
  See bug-227. Left the debug listener in `sim_anton.py` (harmless when no message arrives).
- (2026-07-30) **`accel_to_att_sp()` in guidance_mfc.c has a live scale bug**: the tilt-scaling
  denominator `mfc_thrust_physical` is hardcoded to the nominal-hover constant instead of the
  actual filtered thrust (`filt_thrust.o[0]` is computed then thrown away) — a `// TODO` marks
  it. Left unfixed (pre-existing since before the current regression per `git log`, not
  newly introduced) pending empirical flight verification once bug-227 (no takeoff) is
  resolved — don't silently change control-loop scaling without a way to test it. See
  bug-226. Also fixed a real but currently-inert copy-paste: `mfc_gx`/`mfc_gz.use_trajec_sp`
  were both reading `GUIDANCE_MFC_GY_USE_TRAJECTORY_SP`.

## Key Learnings additions (2026-08-06 — MFC decoupled core port, HEOL stage 1)

- **The MFC gain convention changed.** `mfc_core`'s `kp`/`kd` are now the raw
  coefficients of the closed-loop polynomial `s^2 + kd*s + kp`. They are NOT
  `wn`/`zeta` any more, and `use_Kd` is deleted repo-wide. Conversion from the
  old convention: `kp_new = kp_old^2`, and `kd_new = 2*kd_old*kp_old` when
  `use_Kd=TRUE`, else `kd_new = 2*kp_old`. Any tuning written before 2026-08-06
  is in the old convention and must be converted before use.
- **"Decoupled" in MFC does not mean per-axis.** The six `MfcParameters`
  instances were always independent. `decoupled` selects what drives the
  *estimator*: FALSE (coupled) drives it with the tracking error and folds the
  closed-loop polynomial into `F_hat` as known coefficients `a=-kd`, `b=-kp`;
  TRUE (decoupled) drives it with the measurement, folds nothing (`a=b=0`), and
  an explicit iPD(I) law supplies `kp`/`kd`/`ki` in the command.
- **The flag must gate BOTH the drive signal and the folding, together.**
  Splitting them applies `kp` twice — once inside `F_hat`, once in the explicit
  feedback — and the loop silently runs at double proportional gain. Nothing
  errors.
- **Sign contract, non-negotiable:** `error = measure - setpoint_trajec` and the
  command law SUBTRACTS `fb`: `u = (-F_k + xdd_ref - fb) / alpha`. The folded
  `-kp`, `-kd` are derived against this convention. Feeding `setpoint - measure`
  inverts the folded poles and the loop diverges. `mfc_core.c` already had this
  right — do not "fix" it.
- `ki` is explicit in BOTH structures and is never folded. It uses a trapezoidal
  integral with anti-windup: the integral is frozen (candidate rolled back) in
  any sample where the `u_min`/`u_max` clamp bites.
- The estimator numerator kernel and both `(W^2 + 2W + 1)` IIR smoothers already
  match the upstream Simulink model character-for-character. Only the estimator's
  *input* changed in this port. Do not rederive them.
- **HEOL does not exist in this firmware** (`grep -rni heol` = 0 hits); it lives
  only as a Simulink variant subsystem in the MFC_SISO model repo. Stage 2 builds
  it as a separate library with `mfc_core` as a dependency, NOT by widening
  `mfc_core` — HEOL's estimator is fed `u_fb` (the feedback component only, not
  total `u`), which is exactly what `mfc_core`'s `d2u` term gives when `mfc_core`
  *is* the `u_fb` block. Stage 1's core needs no further change to support it.
- **Verifying a control-law refactor without a toolchain on the host:**
  `mfc_core.c` has only two external deps (`get_sys_time_float`,
  `float_vect_zero`), so old and new versions can be compiled standalone with
  trivial stubs and diffed sample-by-sample. There is no `gcc` in the sbx
  sandbox — run it inside `paparazzi-build:latest` with the harness dir mounted.
  This proved the coupled path bit-for-bit identical over 4000 samples.

## Decision Log additions (2026-08-06)

- **Adopted the upstream model's gain convention rather than keeping wn/zeta.**
  A tuning found in the Simulink model must mean the same thing when typed into
  an airframe XML or a GCS slider; two conventions made that impossible, which is
  why the model tuning was never validated on SITL.
- **`decoupled` is a per-axis runtime `dl_setting`, default FALSE (coupled).**
  The upstream model treats the structure as a block choice, not a parameter.
  Deliberate divergence: a flight controller needs an in-flight A/B against a
  flight-tested tuning, and the coupled path is the flight-tested fallback.
- **Scope held to `mfc_core` + the standalone path.** `oneloop_mfc` got converted
  gains and `use_Kd` removal so it keeps working, but no new settings.
- **Setting ranges widened to `kp,kd in [0,200]`, `ki in [0,50]`.** Under the new
  convention the pitch axis alone needs `kd = 80`; the old `[0,20]`/`[0,10]`
  bounds would have silently clipped valid tunings.

## Do-Not-Repeat additions (2026-08-06)

- Do not treat `mfc_core`'s `kp` as a natural frequency or `kd` as a damping
  ratio. That convention died on 2026-08-06.
- Do not add an explicit P or D feedback term to the coupled path "for symmetry"
  — they are already inside `F_hat`.
- Do not build HEOL by adding a feedforward input to `mfc_core`. See the Key
  Learnings entry above for why it must be its own library.
- When copying `mfc_core`'s per-axis knobs into a new context (e.g. HEOL's
  decoupled-only `u_fb` loop), don't assume `time_trajec` and `int_window`
  are both "reference-filter, coupled-mode only" concepts — they are not.
  `time_trajec` (step 1, gated by `use_trajec_sp`) is; `int_window` (the F_k
  estimator's own integration window, step 3) and `command_filter` (step 4)
  run unconditionally regardless of `use_trajec_sp`/`decoupled`. Dropping
  them "because HEOL doesn't need the reference filter" would have silently
  broken the estimator. Caught by re-reading `mfc_core.c` before implementing
  rather than trusting the plan's own summary of itself.

## Decision Log additions (2026-08-07)

- **HEOL (stage 2/3) is a thin wrapper (`heol.c`/`.h`), not a new estimator.**
  `struct HeolParameters` embeds an `MfcParameters` and drives it with
  `setpoint = 0`, `measure = epsilon` (`= measure - trajectory_ref`), decoupled
  and `use_trajec_sp = false` hardcoded in `heol_init`. `u = u_ff + u_fb`,
  with `u_ff` computed entirely outside `heol.c` by the caller (the guidance
  layer) from the flat trajectory reference — `heol.c` never sees the
  trajectory shape, only the residual. Proven bit-identical to a plain
  decoupled `mfc_core` loop (with `u_ff=0`, `setpoint=trajectory_ref` directly)
  over 4000 samples via the same host-side harness technique as stage 1.
- **`guidance_heol.c` mirrors `guidance_mfc.c` structurally**, same
  plug-function seam (`guidance_h_run_pos/speed/accel`, etc.), same
  Butterworth filtering/mode-entry seeding/thrust packaging — only the per-axis
  struct type and the added `u_ff` term differ. `u_ff` for gx/gy is
  `gh->ref.accel` directly (command unit is already m/s²); for gz it's
  `MASS * (zdd_ref - 9.81)` (command unit is Newtons, so needs the mass/gravity
  inversion) — `gv->zdd_ref`/`gh->ref.accel` come pre-populated by the
  Taylor-extrapolating flat reference model (`guidance_h_ref.c`/
  `guidance_v_ref.c`, built after stage 1's plan was originally sketched).
- **New ANTON_HEOL aircraft** (`anton_heol.xml`, ac_id 219, registered in
  `conf/userconf/ENAC/conf_mfc.xml` — NOT `conf/airframes/ENAC/conf_enac.xml`,
  where ANTON_MFC/ANTON_ONELOOP/Hoops_111_MFC also actually live) rather than
  a flag on ANTON_MFC, so ANTON_MFC stays the untouched flight-tested fallback.
  Stabilization stays `type="mfc"`; only the guidance module changes.
- Module XML element order is DTD-enforced: `dep` must come before `header`
  in `<module>` (`doc,settings*,dep?,header?,init*,...,makefile*`) — putting
  `header` first (as briefly done for `heol.xml`) fails codegen with
  "Unexpected tag : 'DEP'", not an XML syntax error.
- **`heol->mfc.measure` is NOT the real measurement — it's `epsilon`.**
  `heol_run()` passes `epsilon` (not the caller's raw `measure`) into
  `mfc_siso_run()`, so the embedded `MfcParameters.measure` field ends up
  holding the residual, by design (see `heol.h`). Wiring telemetry's `me_*`
  field to `heol_gx.mfc.measure` silently duplicated `err_*` — caught by
  actually inspecting SITL CSV output (`me_z` sat near 0 instead of tracking
  `/uav/EST/z`), not by code review. Fixed by adding a dedicated
  `HeolParameters.measure` field, set at the top of `heol_run()`. General
  lesson: when a wrapper struct embeds another struct and calls its function
  with a *transformed* argument, don't assume the embedded struct's
  same-named field still means what it means in the original context — verify
  wrapper telemetry against a live run, don't just typecheck it.

## Key Learnings additions (2026-08-14 — HEOL feedforward + gain reconciliation)

- `guidance_heol_vert()` (guidance_heol.c) is wired unconditionally into
  `guidance_v_run_pos/speed/accel` — i.e. it always reads `gv->zdd_ref` for
  `z_ff`, even when the active vertical guided mode is plain position hold
  (e.g. Standby's `stay wp=STDBY alt=2.0`), not just the flat-trajectory
  path. Observed in SITL: `sp_traj_z` (telemetry alias for `heol_gz.u_ff`)
  steps to a fixed non-hover value (~-4.71 for ANTON_HEOL) right at the
  flight-plan block handoff away from `Flat_Traj_Demo`, reproducing
  identically even with `FLAT_TRAJ_DEMO_ORDER=0` (no feedforward at all) —
  so it's an existing quirk of the vertical reference-model transition
  between guided sub-modes, unrelated to the flat-trajectory feature.
  Worth a look if MIMO guidance work touches `guidance_v_ref.c`.
- HEOL's decoupled per-axis MFC `u_fb` loop nulls the position residual
  regardless of `u_ff` in SITL (ideal actuators) — turning on the flat-traj
  feedforward did NOT reduce `err_x/y/z` RMS in the sim comparison (both
  ORDER=0 and ORDER=4 gave ~0.007/0.009/0.06 m RMS over the same window).
  The feedforward's expected benefit is less `u_fb` effort / less lag, not
  necessarily a smaller final position error against a sim with no
  actuator/model mismatch — don't expect the tracking-error number alone to
  demonstrate the feedforward is working; check `u_ff` (`sp_traj_*`) shape
  directly instead.
- MFC/HEOL gain convention is the raw coefficients of `s^2 + kd*s + kp`
  (mfc_core.c: `fb = kd*edot + kp*e`), NOT Simulink's second-order form
  (`P=wn^2`, `D=2*zeta*wn`). Convert Simulink -> firmware gains in
  Simulink/MATLAB, never in firmware code.
- `sim.sh`/`sim_anton.py` never exits on its own after the `--nav` sequence
  finishes (loops forever waiting for Ctrl-C) — for a scripted/headless
  capture, launch with `nohup ... &`, poll the stdout log for the expected
  `[nav] block ...` lines to know when the sequence is done, then
  `docker ps -q | xargs -r docker stop` (or `kill` the nohup'd bash PID) to
  end it. Always `docker ps -a` / `rm -f` stray containers before a fresh
  run — a killed `sim.sh` doesn't always take the container down with it.

## Session 2026-08-14 (HEOL flat nominal inputs)

### Key Learnings
- **A constant `alpha` error is self-cancelling in the decoupled iPD/MFC law.**
  With `du = (-F_hat + f_f - fb)/alpha`, the estimator absorbs a constant alpha
  error into `F_hat`, so the closed-loop polynomial stays `s^2 + kd*s + kp`
  regardless of alpha. Consequence: you cannot compensate an alpha change by
  rescaling the PD — verified empirically (÷5 PD made it worse, not better).
  What a smaller alpha really does is amplify *estimator* error by `1/alpha`,
  because `-F_hat/alpha` dominates `-fb/alpha`.
- `heol->mfc.command[]` holds the **correction u_fb**, not the total command —
  unlike `guidance_mfc`, where `mfc.command[0]` IS the total. Any code copied
  between the two must account for this (it caused a 2x-hover mode-entry seed).
- `sat(u* + du)` over `[lo,hi]` == `u* + sat(du)` over `[lo-u*, hi-u*]`. Use the
  shifted form when the anti-windup freeze lives inside the inner clamp.
- `guidance_heol_vert()` runs for EVERY vertical mode (`guidance_v_run_pos` is
  used by NAV altitude hold, GUIDED ZHOLD, GUIDED ALL and GUIDED FLAT), not just
  the flat-trajectory path. Anything it computes must have a sane non-flat
  fallback.
- Vertical reference-model accel limits (`GUIDANCE_V_REF_MIN/MAX_ZDD`, ±0.4 g on
  ANTON_HEOL) show up directly in `sp_traj_z` as `m*(±3.924 - 9.81)` =
  `-10.9855` / `-4.7105` at MASS 0.8. Recognise these before calling them a bug.
- `sim.sh --set` applies settings only AFTER the `--nav` sequence finishes, so it
  cannot be used to change gains before a mid-flight trajectory block. Edit the
  airframe XML and rebuild instead (incremental, ~1 min).
- The NPS sim does not self-terminate after the `--nav` sequence; wrap it in
  `timeout` or it runs until killed (and the CSV grows to hundreds of MB).

### Decision Log
- Flat nominal inputs (`T*`, `phi*`, `theta*`) get their own zero-order-hold
  latch (`guidance_flat_nominal.{c,h}` in `guidance_rotorcraft`), NOT the
  reference model's Taylor extrapolation — they are inputs, not derivatives of
  position, so there is no held higher derivative to extrapolate from. Staleness
  timeout replaces a mode flag. Full rationale:
  `Knowledge/14 - Flat Nominal Inputs Plumbing Decision.md`.
- The Input-Sensitivity Transformation was built WHOLE (emitting both
  `alpha_xy` and `alpha_z`) even though only `alpha_z` has a consumer, so the
  MIMO stage does not have to refactor it out inside a control-structure commit.
- Did NOT land `alpha_z` live despite it being in the plan: it destabilizes the
  vertical loop and the required retune is an estimator-parameter campaign, not
  the PD retune the plan anticipated. Landing an unstable default was judged
  worse than deferring, and the finding itself is the valuable output.

### Do-Not-Repeat
- 2026-08-14: Do not assume a large `alpha` change needs a proportional PD
  rescale in an MFC/HEOL loop. Check whether the estimator absorbs it first
  (see Key Learnings). Three SITL runs were spent confirming the naive
  assumption was wrong.
- 2026-08-14: Do not read `sp_traj_z = -10.9855` as `-9.81*MASS`. With
  MASS=0.8 hover is `-7.848`; `-10.9855` is the MIN_ZDD reference-model
  saturation. An earlier session note made exactly this misreading.

### User Preferences (update 2026-08-14, supersedes the earlier blanket reading)
- **Flags ARE wanted for genuinely open structural questions on a WIP
  controller.** The earlier "prefers direct fixes over compat flags" learning
  (from the rejected G2_IN_ALLOCATION toggle) is narrower than it looked: what
  was rejected there was a *backward-compatibility* toggle preserving old
  behaviour for its own sake. When two alternatives are both live research
  questions — e.g. clamp the total command vs the correction; estimator taps
  pre- vs post-saturation; alpha from the Jacobian vs a constant — the user
  wants BOTH plumbed and selectable from the GCS, defaulting to the spec.
  Ask "is this a compat shim, or an open question?" before deciding.
- **Do not withhold a change because a loop is untuned.** On HEOL the user
  explicitly said none of these controllers are tuned or complete, so a tuning
  cost is not a reason to hold back a structurally correct change. Land it,
  default it to the spec, and document the tuning debt with the measured
  numbers. Backing out `alpha_z` for this reason was over-cautious.
- The user pushes back on inherited concerns that were never load-bearing —
  check where a flagged issue actually *factors in* before amplifying it. The
  ZXY/ZYX warning was inherited from the plan and blown up into a large header
  banner, when in fact it touches exactly one quantity that has no consumer.

### Do-Not-Repeat (added 2026-08-14)
- Do NOT judge a control change by RMS-over-a-window alone. It reported "0.5 m
  RMS, ~10x worse" for a vertical loop that was actually bang-banging between
  both thrust clamps and flying the aircraft 7.7 m off altitude. ALWAYS also
  check: (a) the absolute state excursion, (b) whether the command is pinned at
  its clamps and for what fraction of samples. "Reaches both clamps" in a
  metrics dump is a red flag to investigate, not a footnote to report.
- Telemetry field REUSE across controllers breaks shared plot layouts. HEOL
  sends on GUIDANCE_MFC but sp_traj_* means u_ff (N or m/s^2), not a position
  setpoint — so the shared layout drew newtons against metres on one y-axis.
  When repurposing a message field, check every layout that draws it.

### Do-Not-Repeat (added 2026-08-15)
- **Do not build on a model of the code when the code is available to measure.**
  A numpy float32 model said single precision could not replay the golden traces
  (worst rel err 4.571e+00); the actual compiled C gives 7.3e-4. The model
  rounded every intermediate, the compiler does not. That wrong number justified
  an MFC_FLOAT_T typedef across four file pairs, which then forced float mirrors
  into two more files -- all reverted. The harness that would have falsified it
  in one run already existed.
- When a build's types change, DISTRUST THE LOG BEFORE THE CONTROLLER. A double
  build produced 3e38 telemetry values that looked like divergence; it was the
  scope reading half a double as a float. The flight was fine.
- Eliminated for the HEOL horizontal SITL divergence (do not re-test): sensor
  noise (reproduces with all NPS noise zeroed) and arithmetic precision
  (reproduces with cores in double). Not the same failure as the Simulink
  noise instability, despite looking alike.

## Key Learnings additions (2026-08-17 — ANTON_MFC SITL, rung 2)

- **`mfc_iir_step` has a DOUBLE POLE at `z = W/(W+1)`, so `tau ~= W / f_s`.**
  `time_trajec`, `int_window` and `command_filter` are therefore **sample
  counts**, not time constants — nothing using them transfers between two
  models at different rates without scaling by `f_b/f_a`. At ANTON_MFC's 500 Hz:
  attitude traj W=50 -> 0.10 s, int_window 5 -> 0.010 s, GX int_window
  600 -> 1.20 s, command_filter (one-pole, `(W-1)/W`) 10 -> 0.020 s.
- **The reference filter and the closed loop must be chosen together.** ANTON_MFC
  carried a 0.10 s (~10 rad/s) reference filter driving a `wn = 2 rad/s` loop.
  `mfc_core` feeds forward `dot_dot_setpoint_trajec/alpha`; a 0.235 rad step
  through that filter yields `rddot ~= 22.6 rad/s^2`, which drives the whole
  motion. Measured: 65 % overshoot, ringing at ~7 rad/s after the reference had
  already settled. Slowing traj 50 -> 250 (tau 0.5 s) with **gains unchanged**
  removed the overshoot entirely and cut control effort 9x and estimator
  excursion 10x. Symptom looks exactly like bad gains; it is not.
- **The coupled MFC attitude loop has a bandwidth ceiling between wn 2 and
  wn 6 on ANTON_MFC.** `kp 36/kd 18` (wn 6) and `kp 100/kd 20` (wn 10) BOTH
  diverge, command pinned at the +-5.22 N*m clamp, `fk` to ~4.5e4. Tested both
  via `--set` in flight and baked into the XML from boot — same result, so it is
  a real stability limit, not a bumpless-transfer artefact of changing a folded
  gain mid-flight. Interpretation: in the coupled structure kp/kd raise the
  ESTIMATOR's own gain on e/edot, and the folded-pole construction needs the
  estimator much faster than the loop; with int_window tau 0.01 s, command
  filter 0.02 s and ACT_FREQ 30.5 rad/s that separation is gone by wn 6.
  **The stabilization_mfc.xml module defaults (roll wn 6, pitch wn 8) are ABOVE
  this ceiling — do not adopt them for ANTON_MFC.**
- **`NPS_*_NOISE_STD_DEV_*` are NOT `#ifndef`-guarded.** `nps_sensors.h:6`
  includes `nps_sensors_params_default.h` unconditionally when
  `NPS_SENSORS_PARAMS` is unset. An airframe cannot override them one by one
  from its SIMULATOR section — it must select a params header with
  `<define name="NPS_SENSORS_PARAMS" value="..." type="string"/>` (pattern:
  `jpg_cyclone.xml:63`). ANTON_MFC now uses
  `conf/simulator/nps/nps_sensors_params_anton_mfc.h`, which restates every
  stock default x a scale, so `NPS_NOISE_SCALE=1` is identical to the old
  implicit config and `=0` is perfect sensors (per-source
  `NPS_NOISE_SCALE_ACCEL/_GYRO/_MAG/_BARO/_GPS/_SONAR`).
- **The stock NPS gyro white noise is already ZERO.** The gyro's only stochastic
  content is the 0.5 deg/s bias random walk in `nps_sensors_params_common.h`
  (defined bare -> needs `#undef` to rescale). Don't look for gyro white noise.
- **`--rc_script 1/2/3` are NOT an attitude instrument on ANTON_MFC.** They set
  `MODE_SWITCH_AUTO2`, and anton_mfc.xml maps AUTO2 to `AP_MODE_NAV` — full
  guidance, attitude sticks ignored. The attitude step instrument for this
  airframe is **`--rc_script 5`** (`fp_takeoff_zhold`): NAV climb 15 s, then
  AUTO1 = `AP_MODE_ATTITUDE_Z_HOLD` with +-0.3 stick steps cycling
  pitch -> roll -> yaw, 4 s each (+-0.3 x SP_MAX_PHI 45 deg = +-13.5 deg).
  **Yaw is stick-RATE commanded, so it ramps and a step analyser finds no yaw
  edges — that is correct, not a bug.**
- **guidance_mfc's gx/gy virtual command is a FORCE [N], not an acceleration.**
  It is divided by `mfc_thrust_physical` [N] in `accel_to_att_sp()` and clamped
  by `9.81*MASS*sin(MAX_BANK)*0.7` [N]. The anton_mfc.xml comment calling it
  "NED acceleration [m/s^2] ... clamped to +-g*sin(MAX_BANK)" is wrong on both
  counts. Harmless (clamp and denominator carry the same MASS), documentation
  only. The `0.7` is an undocumented 70 % derating of GUIDANCE_H_MAX_BANK.
- **Hover `gz` on ANTON_MFC reads -9.02 N against -m*g = -7.848 N** (14.9 %).
  Per the airframe's own comment this measures G1-thrust-row/mass identification
  error: JSBSim's motors are ~13 % LESS effective than the identified -1.5 row
  (implied true row ~ -1.305). Absorbed by the F-estimator; err_z RMS 0.012 m.
  Note this is the OPPOSITE sign to Hoops, where JSBSim was ~1.46x MORE
  effective — the deviation is per-airframe, don't generalise it.

## Do-Not-Repeat additions (2026-08-17)

- **Do not attribute a sim CSV with `ls -t`.** `sim.sh`/`sim_anton.py` never
  self-terminate, and `timeout -s INT` leaves the capture relay listening on
  127.0.0.1:9871 — a second sim started while it lives is ingested into the
  FIRST run's file. Two captures were contaminated this way and had to be
  re-run. Serialize (`docker rm -f` + settle before AND after each sim), take
  the CSV name from that run's own stdout, and validate every capture: expect
  ~500 rows/s, zero duplicate timestamps, zero backward time steps.
  (bug-270)
- **Do not launch overlapping background sim tasks in this sandbox.** Several
  `run_in_background` waits stacked up and re-entered the sim concurrently; that
  is what caused bug-270 both times.
- **Do not trust a step-response analyser that segments edges across axes.**
  The first version merged roll and pitch edges and reported "-100 %" and
  "+232 %" overshoot on a well-behaved run. Segment per axis, take the dwell as
  the span to the next edge of the SAME axis, and baseline against the settled
  post-edge setpoint.
- **Before concluding gains are wrong on an MFC axis, check the reference
  filter's bandwidth against the loop's.** `tau_ref = W/f_s` vs `1/wn`. If the
  reference is faster, the `rddot` feedforward drives the response and the
  overshoot is not a gain problem. See the Key Learnings entry above.

## Key Learnings additions (2026-08-17 part 2 — MFC position loop works)

- **In the COUPLED MFC structure, `int_window` is the ESTIMATOR BANDWIDTH, not a
  noise filter — the estimator IS the feedback path** (poles folded into F_hat).
  Raising ANTON_MFC's `GZ_INTEGRATION_WINDOW` 4 → 50 (the "obvious" fix for a
  noisy z) detuned the controller 12× and made the vertical loop bang-bang
  between both thrust clamps, 10–30 m excursions on a 3 m setpoint. Simulink can
  run 50 there only because **its z axis is DECOUPLED**, where the poles are in
  an explicit iPD and the estimator only cancels disturbance. Check `decoupled`
  before reasoning about any window.
- **Before filtering an MFC measurement, compare the filter's lag against that
  axis's `int_window` and `ref_window` IN MILLISECONDS.** Adding the x/y 3 Hz
  Butterworth (~60 ms lag) to gz — whose ref filter and estimator window are both
  8 ms — produced the same clamp-to-clamp limit cycle. If the lag is comparable
  or larger, slow the whole channel coherently instead. `guidance_mfc.c` now
  documents why z stays on the raw measurement.
- **`est_use_presat_command` was `true` (pre-saturation) on all six MFC axes;
  the upstream Simulink feeds the POST-EMA, POST-CLAMP command.** Pre-saturation
  makes F_hat drift to cover a command the plant never received and drives the
  command further into the rail — a limit-cycle generator on any saturating axis.
  Default flipped to `false` in `mfc_core.c`. Same defect had been found
  independently on HEOL.
- **Simulink `kd` is `zeta`, NOT `2*zeta`.** Correct conversion:
  `kp_fw = kp_sim^2`, `kd_fw = 2*kd_sim*kp_sim`. (An earlier same-day inference
  from second-hand quoted values dropped the 2 — corrected against measured
  Simulink applied coefficients φ/θ = 4/12, ψ = 4/6, thrust = 16/5.6.)
- **The real cause of ANTON_MFC's 65 % attitude overshoot was roll/pitch `Kd`
  under-damped by exactly 2×** (6 vs the reference's 12), not the reference-filter
  bandwidth. `Kd = 12` gives 34 % overshoot AND keeps the 0.175 s rise;
  `TIME_TRAJECTORY` stays at its flight-validated 50. Raising Kd further (20)
  only reaches 25 % for 40 % more command effort — not worth it. Note raising
  `kp` still hits the wn 2–6 stability ceiling; raising `kd` alone is safe.
- **ANTON_MFC position control WORKS** (2026-08-17): hover hold x/y rms
  4.8/4.2 cm with noise, 0.62/0.45 cm without, command at 5 % of the ±1.879 N
  rail; vertical alone err_z rms 1.7 mm, F̂_gz rms 57.5; `Flat_Traj_Demo`
  tracking err x/y/z rms 6.1/5.7/0.8 cm. Reached rung 5. The two changes that did
  it were the estimator tap and roll/pitch Kd — **nothing on the z or horizontal
  channels was touched.**
- **Horizontal `Kp 2 / Kd 25` vs Simulink `75 / 150` is a real 37×/6× gap**
  (same structure, same alpha 18.75 — the one axis pair where a bare gain
  comparison is legitimate). Kept the firmware values deliberately: firmware
  holds 4.5 cm at 5 % rail, Simulink holds 2.5 cm with its command ON THE RAIL
  73–92 % of the time. Don't chase the reference's rail fraction.

## Do-Not-Repeat additions (2026-08-17 part 2)

- Do NOT change several parameters on one MFC axis at once. Changing gz
  structure + gains + both windows + command filter together broke the vertical
  loop and cost three runs to bisect. One knob per run on a channel that works.
- Do NOT assume a "too short" estimator window is starving a channel without
  first checking whether that channel is coupled or decoupled — the sign of the
  argument flips.
- Do NOT reach for a measurement filter as the first response to "channel X is
  noisy" in MFC. On a fast coupled channel it is destabilising, not smoothing.

## Key Learnings additions (2026-08-18 — flat traj from the GCS)

- **`autopilot_set_mode(AP_MODE_GUIDED)` from a flight plan does NOT survive with
  an RC link connected.** `autopilot_static_on_rc_frame()` re-derives the mode
  from the 3-way switch every RC frame and stamps it back to NAV (AUTO2). The
  block keeps running but `guidance_h_from_nav()` then serves the setpoint from
  `nav.carrot` -- the previous block's leftover waypoint. **Use the NAV sub-modes
  instead**: `nav.horizontal_mode = NAV_HORIZONTAL_MODE_GUIDED` and
  `nav.vertical_mode = NAV_VERTICAL_MODE_GUIDED`, re-asserted every tick. They
  reach the same guided runners and the RC switch cannot touch them (this is what
  the stock `NavGuided()` macro does). **SITL has no RC link, so this class of bug
  is GCS/flight-only and cannot be reproduced headless.**
- **Fixed-point round-trips inside a per-tick integrator integrate their own
  quantization error.** `gv_update_ref_from_flat_ref()` pushed `gv_z_ref` (Q37.26)
  through Q23.8 each tick; `BFP_OF_REAL` truncates toward zero and NED altitude is
  negative, so it ratcheted UP one full LSB (3.9 mm) per 2 ms tick = +1.95 m/s
  phantom climb, independent of the true step size. Ask of any such loop: (a) does
  the read/write pair preserve the stored resolution, (b) does it round or
  truncate, and is the quantity signed. Truncation toward zero is a *biased*
  estimator for negatives, and bias inside an integrator is drift.
- **A float cannot hold a Q37.26 position** (3 m * 2^26 = 2e8 > 24-bit mantissa).
  Accumulate the small STEP at full resolution instead of round-tripping the
  absolute value. See `Knowledge/17 - The Flat Reference Fixed-Point Ratchet.md`.
- **Anything that double-differentiates its setpoint turns a benign reference
  wobble into clamp saturation.** MFC and HEOL both do (`dot_dot_setpoint_trajec`).
  A 0.06 m snap in one 2 ms sample is `rddot` ~1e4 m/s^2. A PID would ignore the
  same signal entirely -- which is why these bugs survive in shared guidance code.
- **The PlotJuggler tell for a sawtoothing setpoint is a "hatched"/thick line.**
  Decimated plots average it into a clean line at the right value. Sample the raw
  CSV at the loop tick before concluding a setpoint is clean.

## Do-Not-Repeat additions (2026-08-18)

- **Do not diagnose a GCS/flight-only bug from source alone.** I proposed three
  candidate mechanisms from code reading and shipped a fix for one without
  reproducing; the user identified the right one from operational experience.
  When the symptom is only observable on the real system, ASK FOR THE LOG FIRST.
  A per-tick CSV settled in one pass what code reading had not in several.
- **Do not copy `GZ_MAX_THRUST` (or any `G1`-derived constant) between airframes.**
  It must track that airframe's own G1 thrust row: `guidance_mfc_vert()` normalizes
  by it and the stabilizer decodes through its own SI row, so a mismatch scales
  every thrust command by the ratio (1.5 vs 0.7 = 2.14x, enough to stop it
  climbing). Gains/windows/structure DO transfer between airframes at the same
  loop rate; anything derived from G1, mass or inertia does NOT.
- [2026-08-20] **Darko (and Paparazzi tailsitters generally) use the hover body frame, not the fuselage frame.** `theta = 0` IS hover (rotors up, quadrotor-like); forward flight is `theta = TRANSITION_MAX_OFFSET = -75 deg`. The `cyclone` JSBSim model is built in the same frame (both motor `<force>` blocks point along body `-Z`; elevon moments are constant-coefficient about that frame's Y/Z, airspeed-independent). FlightGear is fed the FDM's body attitude directly, so a *level* aircraft render in hover is CORRECT and is not evidence of an axis mismatch — and because FG shows the same theta the controller uses, the render can never reveal a frame-convention error.
- [2026-08-20] **Darko G1 vs `cyclone` JSBSim static effectiveness cross-check** (INDI_G_SCALING=1000, pprz full scale 9600 → alpha_full = G1*9.6 rad/s^2; Ixx/Iyy/Izz = 0.0179/0.00339/0.0203 kg*m^2): roll matches well (G1 144 vs model ~130 rad/s^2), but pitch (G1 ~40 vs model ~17) and yaw (G1 ~37 vs model ~23 per elevon) are OVER-estimated in G1 by ~2.4x and ~1.6x versus the sim plant. All signs agree with the model. Relevant to the unresolved ~1.2 Hz +/-35 deg pitch limit cycle. Static check only — not verified in flight.
