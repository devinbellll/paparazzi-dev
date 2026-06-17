# OpenWolf

@.wolf/OPENWOLF.md

This project uses OpenWolf for context management. Read and follow .wolf/OPENWOLF.md every session. Check .wolf/cerebrum.md before generating code. Check .wolf/anatomy.md before reading files.


# ENAC Paparazzi Firmware Workspace

Headless ARM Cortex-M firmware development environment for ENAC UAV Lab aircraft using the Paparazzi autopilot framework. Primary focus: **control system development** — INDI stabilization, INDI guidance, new control algorithms.

## Workspace layout

```
/workspace/
├── paparazzi/              # Upstream Paparazzi (github.com/paparazzi/paparazzi)
│   ├── conf/airframes/ENAC/
│   │   ├── conf_enac.xml              # Active fleet — aircraft registry
│   │   ├── fixed-wing/                # Fixed-wing airframe XMLs
│   │   ├── hybrid/                    # Hybrid (VTOL) airframe XMLs
│   │   └── quadrotor/                 # Rotorcraft airframe XMLs
│   ├── sw/airborne/
│   │   ├── firmwares/rotorcraft/
│   │   │   ├── stabilization/         # Stabilization algorithms (INDI, PID, ANDI…)
│   │   │   └── guidance/              # Guidance algorithms (INDI guidance, hybrid…)
│   │   └── math/wls/                  # Weighted Least Squares allocator
│   └── conf/modules/                  # Module XML definitions (build + settings wiring)
├── Knowledge/              # Developer notes — Obsidian-compatible, start here
├── enac_paparazzi/         # ENAC fork (github.com/enacuavlab/paparazzi) — older, not used for builds
├── pprz.sh                 # The build tool: build / clean / rebuild / db / codegen / bootstrap
├── sim.sh                  # NPS sim runner (ephemeral container)
├── pprz_docker.sh          # Ephemeral-container dispatcher library
├── build_image.sh          # Build the paparazzi-build toolchain image
├── Dockerfile.build        # arm64 toolchain image (standalone, no Claude layers)
├── Dockerfile.paparazzi    # Legacy: toolchain layered on the old devcontainer base
└── .devcontainer/          # Legacy VS Code devcontainer config (pre-sandbox)
```

The active ENAC airframe definitions live in `paparazzi/conf/airframes/ENAC/`. Treat `paparazzi/` as the source of truth; `enac_paparazzi/` is an older fork kept for reference.

## Knowledge Base

**Start here for any control system work:**

```
Knowledge/
├── 00 - Index.md                         # Navigation hub
├── 01 - Control System Architecture.md   # 4-layer stack: Nav→Guidance→Stab→Actuators
├── 02 - INDI Stabilization Deep Dive.md  # Algorithm, all parameters, G1/G2/WLS
├── 03 - INDI Guidance Deep Dive.md       # guidance_indi, G-matrix, variants
├── 04 - Airframe XML Configuration.md    # Every XML section for control tuning
├── 05 - Module System.md                 # How modules.xml drives the build
├── 06 - Modifying ANTON Stabilization.md # 6 worked examples (gains → new algorithm)
└── 07 - All Touch Points Cheatsheet.md   # File checklist per change type
```

Before ending any non-trivial session, ALWAYS write Knowledge/Sessions/<YYYY-MM-DD>-<topic>.md. Check if one exists for today before stopping.

## Building firmware

Claude now runs in a lean sbx sandbox with **no local toolchain**. Builds and the
NPS sim run in **ephemeral arm64 containers** (`Dockerfile.build`), with the repo
bind-mounted at `/workspace` so artifacts land back in the tree. The firmware is
cross-compiled (`arm-none-eabi` → Cortex-M), so an arm64-native build produces the
same `.elf` as the old amd64/Rosetta image — without emulation.

```bash
./build_image.sh                 # one-time: build the paparazzi-build:latest image
./pprz.sh build AIRCRAFT [TARGET]   # TARGET defaults to ap
```

`pprz.sh` runs the work natively when a cross-compiler is on PATH (inside the
container), otherwise it dispatches itself into an ephemeral container
automatically. The aircraft and target are **separate args** — no CONF path to
pass (the fleet XML is fixed to `conf/airframes/ENAC/conf_enac.xml`; override with
the `CONF` env var, given relative to the `paparazzi/` dir).

```bash
./pprz.sh build ANTON              # ap target
./pprz.sh build ANTON_MFC nps      # nps (sim) target
./pprz.sh build PANACHE_1 ap
```

`build` is **incremental and CMake-like**: it re-runs codegen only when the
airframe/conf XML changed (the generator compares mtimes + an md5 of the config),
and recompiles only the C files that changed — make drives the dependency graph.
The first build of an aircraft (or after `clean`) compiles everything; subsequent
builds are fast.

Other commands:

```bash
./pprz.sh clean    ANTON_MFC ap    # wipe var/aircrafts/ANTON_MFC
./pprz.sh rebuild  ANTON_MFC ap    # clean, then build
./pprz.sh db       ANTON_MFC ap    # regenerate compile_commands.json for clangd
./pprz.sh codegen  ANTON_MFC ap    # regenerate airframe.h / modules.h only
./pprz.sh bootstrap                # (re)build the OCaml ground segment + generators
```

Output: `paparazzi/var/aircrafts/<AIRCRAFT>/<TARGET>/obj/<TARGET>.elf`

**Image build network prerequisite:** the paparazzi PPA needs three domains that
the sandbox blocks by default. Allow them on the **host** before `./build_image.sh`:
```
sbx policy allow network api.launchpad.net,keyserver.ubuntu.com,ppa.launchpadcontent.net
```
If the PPA has no arm64 binaries, fall back to `./build_image.sh --amd64` (qemu
emulation; keeps `USE_LTO=no`). Scripts are unchanged — only the image platform differs.

**Build flags that must stay:**
- `USE_LTO=no` (ap target only) — was a Rosetta LTO-ICE workaround; kept for fast
  incremental dev rebuilds (flip to `USE_LTO=yes` for release). arm64-native no
  longer hits the ICE. Applied automatically by `pprz.sh` for the `ap` target.
- Ground-segment bootstrap uses `make -j1` — parallel builds hit a git submodule
  lock conflict in the dronecan submodule. `pprz.sh` only runs this once (when the
  generators / `var/include` are missing), not on every build.

**A successful compile (`.elf` produced) is the only automated correctness check.** Functional correctness requires flight test.

## Tooling scripts

| Script | Purpose |
|--------|---------|
| `build_image.sh [--amd64]` | Build the `paparazzi-build` toolchain image. |
| `pprz_docker.sh` | Dispatcher library: `pprz_run` runs a command in an ephemeral container (repo → `/workspace`). |
| `pprz.sh <cmd> AIRCRAFT [TARGET]` | The build tool. `build` (incremental), `clean`, `rebuild`, `db` (compile_commands.json for clangd, `/workspace`→host rewritten), `codegen`, `bootstrap`. Runs native in-container, else dispatches into one. Accepts the VSCode picker form `"AIRCRAFT (target)"` too. |
| `sim.sh [--no-build] <sim_anton flags>` | Build the `nps` target (build-if-needed via `pprz.sh build`) and run the NPS sim in a container (IVY local; PlotJuggler/FlightGear stream to the Mac). |

## VSCode integration (native Mac + Docker Desktop)

VSCode runs natively on the Mac and dispatches builds to its own Docker Desktop
daemon (image built there too); Claude uses the sandbox's daemon. Both write the
same shared workspace files. IntelliSense is **clangd** driven by a host-path
`compile_commands.json` (the MS C/C++ engine is disabled but its config-name picker
still drives the tasks). The single **Build** task (Shift+Cmd+B) runs
`./pprz.sh build "<active config>"` — incremental. Regenerate the DB after
XML / file-set / define changes: the *IntelliSense: regenerate compile DB* task
(`./pprz.sh db "ANTON_MFC (ap)"`). Sim C debugging: run the
*Sim: ANTON_MFC (gdb wait :1234)* task, then the *Attach: NPS sim C* launch
config (sourceFileMap maps `/workspace`→repo).

## Aircraft fleet

Registered in `paparazzi/conf/airframes/ENAC/conf_enac.xml`:

| Aircraft  | ac_id | Type       | Airframe file                                 | Stabilization |
|-----------|-------|------------|-----------------------------------------------|---------------|
| CYFOAM    | 6     | Hybrid     | airframes/ENAC/hybrid/cyfoam.xml              | INDI |
| ZAGI      | 12    | Fixed-wing | airframes/ENAC/fixed-wing/zagi_test.xml       | Adaptive FW |
| RoBoBee   | 2     | Quadrotor  | airframes/ENAC/quadrotor/robobee.xml          | INDI |
| ANTON     | 217   | Quadrotor  | airframes/ENAC/quadrotor/anton_indi_aruco.xml | INDI (Tawaki) |
| CobraV2   | 11    | Quadrotor  | airframes/ENAC/quadrotor/cobraV2.xml          | INDI |
| MAYA      | 1     | Quadrotor  | airframes/ENAC/quadrotor/maya_outdoor.xml     | INDI |
| FALCON_V2 | 3     | Hybrid     | airframes/ENAC/hybrid/falcon_v2.xml           | INDI hybrid |
| PANACHE_1 | 4     | Fixed-wing | airframes/ENAC/fixed-wing/panache_1.xml       | Adaptive FW |
| CROW      | 5     | Quadrotor  | airframes/ENAC/quadrotor/crow_indoor.xml      | INDI |
| GOOSE     | 7     | Quadrotor  | airframes/ENAC/quadrotor/goose.xml            | INDI |

## Control system source locations

```
# Stabilization algorithms
paparazzi/sw/airborne/firmwares/rotorcraft/stabilization/
  stabilization_indi.c/h            ← core INDI algorithm (G1/G2, WLS, filters)
  stabilization_attitude_quat_indi.c ← outer attitude-to-rate PD loop
  stabilization_andi.c              ← ANDI variant
  stabilization_rate_indi.c         ← rate-only INDI
  stabilization_attitude_quat_int.c ← classical integer PID (reference/fallback)

# Guidance algorithms
paparazzi/sw/airborne/firmwares/rotorcraft/guidance/
  guidance_indi.c/h                 ← base INDI guidance (velocity→attitude)
  guidance_indi_quadrotor.c         ← quadrotor G-matrix (used by ANTON, MAYA, etc.)
  guidance_indi_hybrid.c/h          ← VTOL/hybrid variant
  guidance_v.c                      ← vertical axis (classical, all vehicles)

# WLS control allocator
paparazzi/sw/airborne/math/wls/wls_alloc.c/h

# Module XML (build wiring + GCS settings panels)
paparazzi/conf/modules/stabilization_indi.xml
paparazzi/conf/modules/guidance_indi.xml
paparazzi/conf/modules/guidance_indi_base.xml
paparazzi/conf/modules/guidance_indi_quadrotor.xml

# State / sensor fusion output (read-only by control)
paparazzi/sw/airborne/state.h       ← stateGetNedToBodyQuat_f(), stateGetBodyRates_f(), etc.
```

## Control system development workflow

### Changing only gains (no recompile)
→ Edit the XML section, rebuild, or tune live via GCS settings panel.

### Changing the control law parameters (G1, ACT_FREQ, filters)
1. Edit the airframe XML (`conf/airframes/ENAC/quadrotor/<aircraft>.xml`)
2. Rebuild with `./pprz.sh build <AIRCRAFT> ap` (codegen reruns automatically on XML change)
3. Parameters become `#define` via `generated/airframe.h` at compile time

### Writing a new stabilization module
1. Add `.c`/`.h` in `sw/airborne/firmwares/rotorcraft/stabilization/`
2. Add `conf/modules/stabilization_<type>.xml`
3. Wire `<module name="stabilization" type="<type>"/>` in the airframe XML
4. Implement: `stabilization_<type>_init()`, `stabilization_<type>_run(in_flight, sp, thrust, cmd)`

### Key INDI parameters in airframe XML

```xml
<section name="STABILIZATION_ATTITUDE_INDI" prefix="STABILIZATION_INDI_">
  <define name="REF_ERR_P/Q/R"  .../>  <!-- outer PD attitude gains -->
  <define name="REF_RATE_P/Q/R" .../>  <!-- inner rate damping gains -->
  <define name="G1" type="matrix">...  <!-- [outputs × actuators] effectiveness -->
  <define name="G2" value="..."/>      <!-- propeller inertia coupling -->
  <define name="ACT_FREQ" value="..."/> <!-- actuator bandwidth rad/s -->
  <define name="WLS_PRIORITIES" .../>  <!-- roll/pitch/yaw/thrust priority -->
</section>
```

The `WLS_N_U_MAX` / `WLS_N_V_MAX` defines must be inside the `<module>` tag (not the section), since they set compile-time matrix sizes.

## Airframe XML structure

```xml
<airframe name="AIRCRAFT">
  <firmware name="rotorcraft">
    <target name="ap" board="..."/>
    <module name="stabilization" type="indi">
      <define name="WLS_N_U_MAX" value="4"/>  <!-- compile-time matrix sizes -->
      <define name="WLS_N_V_MAX" value="4"/>
    </module>
    <module name="guidance" type="indi"/>
    <module name="ins" type="ekf2"/>
  </firmware>
  <servos>...</servos>
  <commands>...</commands>
  <command_laws>...</command_laws>
  <section name="IMU">...</section>
  <section name="STABILIZATION_ATTITUDE_INDI">...</section>
  <section name="GUIDANCE_INDI">...</section>
  <section name="GUIDANCE_V">...</section>
</airframe>
```

## Build environment (current vs legacy)

**Current:** Claude runs in an aarch64 sbx sandbox with a nested Docker daemon and
no toolchain. Builds/sims run in ephemeral `paparazzi-build` containers from
`Dockerfile.build` (arm64-native, no Rosetta). See *Building firmware* and
*VSCode integration* above.

**Legacy (pre-sandbox):** `.devcontainer/` + `Dockerfile.paparazzi` ran the build
inside a linux/amd64 devcontainer under Rosetta 2 (VSCode lived at `/workspace`).
Kept for reference; the active path is the ephemeral-container model.
