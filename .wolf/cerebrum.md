# Cerebrum

> OpenWolf's learning memory. Updated automatically as the AI learns from interactions.
> Do not edit manually unless correcting an error.
> Last updated: 2026-06-04

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
- [2026-06-11] **MFC timing is per-SISO-module, not global**: `sample_time`/`time`/`start_time` live in each `struct MfcParameters`; API is `mfc_siso_init(axis, dt)` + `mfc_siso_reset(axis)` (no more global `struct Mfc`/`mfc_core_init`/`mfc_core_start`). Chosen so axes start/reset independently. This made the bug-030/031 global-clock workarounds obsolete — they were removed (bug-033): `guidance_mfc_vert()` now runs raw MFC every cycle when `gz.enabled` (no `in_flight` gating). Trade: ground estimator windup returns (accepted by user). Result: MFC Z guidance flies smooth in NPS. Verified state = commit 24dcae6.
- [2026-06-09] **Makefile.ac caching bug (known limitation)**: `sw/lib/ocaml/aircraft.ml:478` sets `autopilot=true` only for the current `-target`; the file is only copied if newer than sources. After NPS build, AP build reuses the NPS Makefile.ac (which lacks `USE_GENERATED_AUTOPILOT=TRUE` in the AP block). Fix: delete `Makefile.ac` before switching targets. This is a Paparazzi upstream bug.
- [2026-06-11] **Thrust-unit conversion lives in guidance, not the shared stabilizer**: `guidance_mfc` emits its vertical thrust in physical (indi_v[3]-space) units. A single compile-time `GUIDANCE_MFC_THRUST_TO_PPRZ` define (default FALSE) selects packaging: FALSE → raw float (`th_sp_from_thrust_f`) for `stabilization_mfc`'s pseudo-inverse; TRUE → scale by `GUIDANCE_MFC_THRUST_PPRZ_SCALE = 1/sum(Bwls[3][i])` and emit an int (`th_sp_from_thrust_i`) so the **stock** INDI Bwls path round-trips exactly. Chosen over hacking each stabilizer so both `stabilization_indi.c` and `stabilization_mfc.c` keep their stock contracts; guidance adapts to the selected downstream. `stabilization_indi.c` was reverted to stock; `stabilization_mfc.c` now reads `thrust->sp.thrust_f[Z]` directly (OUTPUTS==4). Scale for ANTON = `1000/(4*-1.5)` = -166.67. Supersedes the bug-035 diagnosis "option (2)" hack.

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
