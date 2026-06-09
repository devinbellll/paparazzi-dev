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
├── build_fw.sh             # Firmware build wrapper
├── Dockerfile.paparazzi    # Container toolchain image
└── .devcontainer/          # VS Code devcontainer config
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

```bash
./build_fw.sh AIRCRAFT CONF_XML [TARGET]
```

CONF_XML is a path relative to `/workspace`. Default target is `ap`.

```bash
./build_fw.sh ANTON paparazzi/conf/airframes/ENAC/conf_enac.xml
./build_fw.sh PANACHE_1 paparazzi/conf/airframes/ENAC/conf_enac.xml ap
```

Output: `paparazzi/var/aircrafts/<AIRCRAFT>/ap/obj/ap.elf`

**Build flags that must stay:**
- `USE_LTO=no` — permanent workaround for a gcc-arm-none-eabi 13.2 LTO internal compiler error triggered by Rosetta 2 emulation. Remove only if the toolchain is upgraded and the ICE is confirmed fixed.
- Host tools use `make -j1` — parallel builds hit a git submodule lock conflict in the dronecan submodule.

**A successful compile (`.elf` produced) is the only automated correctness check.** Functional correctness requires flight test.

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
2. Rebuild with `./build_fw.sh`
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

## Devcontainer

The build runs in a linux/amd64 container (`Dockerfile.paparazzi`) built on top of a Claude Code base image. Apple Silicon Macs use Rosetta 2 emulation — builds are slower but correct. The two-stage build and `make up` shortcut are documented in README.md.
