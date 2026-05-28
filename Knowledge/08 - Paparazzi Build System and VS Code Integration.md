# Paparazzi Build System — Makefile Chain and VS Code Integration

## Overview

The build system has three layers of Makefiles that hand off to each other. Understanding this chain is the key to wiring up VS Code Makefile Tools correctly.

---

## Layer 1 — Entry point: `paparazzi/Makefile.ac`

This is the file passed to every `make` invocation at the top level.

**Required inputs (from the command line):**
| Variable | Example | Purpose |
|---|---|---|
| `AIRCRAFT` | `ANTON_MFC` | Selects which aircraft to build |
| `CONF_XML` | `.../conf_enac.xml` | Path to the fleet registry XML |
| `TARGET` | `ap` or `nps` | Firmware target (hardware or simulation) |

**What it does per target:**

### `TARGET.ac_h` (e.g. `ap.ac_h`)
Runs the code generator (`gen_aircraft.out`) against the XML. Produces:
- `var/aircrafts/AIRCRAFT/TARGET/generated/airframe.h` — all `#define`s from the XML (servo indices, AC_ID, stabilisation params, G1/G2, etc.)
- `var/aircrafts/AIRCRAFT/TARGET/generated/flight_plan.h`
- `var/aircrafts/AIRCRAFT/TARGET/generated/settings.h`
- **`var/aircrafts/AIRCRAFT/Makefile.ac`** — the per-aircraft generated Makefile (see Layer 2)

### `all_ac_h`
Writes `var/aircrafts/AIRCRAFT/TARGET_srcs.list`. This file contains the fully-resolved compiler flags and source list — everything the compiler sees. It is produced by including `sw/airborne/Makefile` (Layer 3) and dumping the result.

**Trigger condition:** `all_ac_h` only includes `sw/airborne/Makefile` when `MAKECMDGOALS == all_ac_h` (line 109–113 of `Makefile.ac`). This means the per-aircraft `Makefile.ac` (Layer 2) must already exist before `all_ac_h` is run.

### `TARGET.compile` (e.g. `ap.compile`)
The full build. Chains in order:
1. `TARGET.ac_h` — code generation
2. `make TARGET -f Makefile.ac all_ac_h` — resolves and writes `_srcs.list`
3. `cd sw/airborne && make TARGET=TARGET all` — actual compilation

### `TARGET.upload`
Calls `TARGET.compile` then flashes the firmware.

---

## Layer 2 — Per-aircraft generated Makefile: `var/aircrafts/AIRCRAFT/Makefile.ac`

This file is **generated** by `gen_aircraft.out` from the airframe XML. It must not be edited manually.

**What it contains:**
- `ifeq ($(TARGET), ap)` / `ifeq ($(TARGET), nps)` blocks — separate sections per target
- `$(TARGET).CFLAGS +=` assignments for all `-D` defines and `-I` include paths relevant to that aircraft/target
- `$(TARGET).srcs +=` assignments listing every source file
- `include` statements for board and firmware makefiles (e.g. `conf/boards/tawaki_1.0.makefile`, `conf/firmwares/rotorcraft.makefile`)

**Key point:** Most values inside are still Make variable references (e.g. `$(ARCH)`, `$(PERIODIC_FREQUENCY)`). They are not yet resolved to literal strings. Resolution happens in Layer 3.

---

## Layer 3 — Airborne firmware Makefile: `sw/airborne/Makefile`

This is the Makefile that actually compiles source files. It is invoked by `Makefile.ac` (Layer 1) with `cd sw/airborne && make TARGET=TARGET all`.

**What it does:**
1. Includes `var/aircrafts/AIRCRAFT/Makefile.ac` (Layer 2) — this imports all the `$(TARGET).CFLAGS` and `$(TARGET).srcs` assignments.
2. Includes `conf/Makefile.local` — sets `PAPARAZZI_HOME`, `PAPARAZZI_SRC`, etc.
3. Includes either `conf/Makefile.chibios` (for `ap`) or `conf/Makefile.sim` (for `nps`) — this defines `ARCH`, compiler paths, linker flags, etc.
4. Appends git version defines to `$(TARGET).CFLAGS`.
5. Resolves all Make variables to produce the final CFLAGS and source list.
6. Compiles everything with gcc/arm-none-eabi-gcc.

---

## The Resolved Output: `TARGET_srcs.list`

After `all_ac_h` runs, `var/aircrafts/AIRCRAFT/TARGET_srcs.list` contains:

```
TARGET:  ap
CFLAGS:  -mcpu=cortex-m7 -O2 ... -DUSE_CHIBIOS_RTOS ... -DWLS_N_V_MAX=4 -I/workspace/paparazzi/var/aircrafts/ANTON_MFC/ap ...
LDFLAGS: ...
srcs:  arch/chibios/mcu_arch.c firmwares/rotorcraft/stabilization/stabilization_mfc.c ...
```

This is the **only** place where all Make variable references are fully resolved into literal strings. Every `-D` and `-I` that the compiler actually sees is here.

---

## The Problem for VS Code Makefile Tools

Makefile Tools uses `make -n` (dry-run) to capture compiler invocations for IntelliSense. For this to work:

1. **The target must exist.** `Makefile.ac` has no `all` target — only `ap.compile`, `nps.compile`, `all_ac_h`, `clean_ac`, `print_version`, `generate_keys`, `build_rust_modules`, `hitl.*`. Running `make all` returns exit code 2.

2. **The generated files must exist first.** The dry-run of `ap.compile` internally calls `ap.ac_h`, which needs `gen_aircraft.out` built. And `all_ac_h` needs `var/aircrafts/AIRCRAFT/Makefile.ac` to already exist.

3. **`AIRCRAFT` and `CONF_XML` must be set.** Without these, `Makefile.ac` errors immediately.

The correct `make -n` invocation for IntelliSense would be:
```
make -n \
  -f /workspace/paparazzi/Makefile.ac \
  -C /workspace/paparazzi \
  AIRCRAFT=ANTON_MFC \
  CONF_XML=/workspace/paparazzi/conf/airframes/ENAC/conf_enac.xml \
  USE_LTO=no \
  ap.compile
```

But this requires the generated files to already exist in `var/aircrafts/ANTON_MFC/`.

---

## Current VS Code Setup

### `settings.json` — Makefile Tools configurations

```json
"makefile.makefilePath": "${workspaceFolder}/paparazzi/Makefile.ac",
"makefile.makeDirectory": "${workspaceFolder}/paparazzi",
"makefile.configureOnOpen": false,
"makefile.configurations": [
  {
    "name": "ANTON_MFC (ap)",
    "makeArgs": ["AIRCRAFT=ANTON_MFC", "CONF_XML=...", "USE_LTO=no"],
    "buildTarget": "ap.compile"
  },
  ...
]
```

`makeArgs` provides `AIRCRAFT`, `CONF_XML`, and `USE_LTO=no` (required for ARM builds). `buildTarget` sets the compile target when the configuration is selected in the status bar.

### `tasks.json` — Build tasks (reliable alternative to Makefile Tools build button)

`Ctrl+Shift+B` → `build: AIRCRAFT TARGET` calls `build_fw.sh` which runs the correct full chain.

### `gen_vscode.sh` — Codegen prebuild helper

Runs steps 1–2 of the chain (no compile): creates the generated headers and `var/aircrafts/AIRCRAFT/Makefile.ac`. Run this after changing any airframe XML before triggering a Makefile Tools configure.

```bash
./gen_vscode.sh AIRCRAFT [CONF_XML] [TARGET]
```

### `c_cpp_properties.json` — IntelliSense config

Currently delegates to Makefile Tools via `configurationProvider: "ms-vscode.makefile-tools"`. Falls back to no IntelliSense if Makefile Tools configure hasn't succeeded.

---

## What Needs to Work for IntelliSense

The minimal sequence for IntelliSense to function:

1. **Run codegen first:**
   ```bash
   ./gen_vscode.sh ANTON_MFC  # creates var/aircrafts/ANTON_MFC/Makefile.ac + generated/
   ```

2. **Select the configuration in VS Code:**
   Click the Makefile Tools configuration item in the status bar → pick `ANTON_MFC (ap)`.
   This activates `makeArgs` AND sets `buildTarget` to `ap.compile`.

3. **Trigger configure:**
   `Ctrl+Shift+P → Makefile: Configure`
   This runs `make -n ... ap.compile` and extracts compiler commands.

4. **IntelliSense is now populated** from the dry-run output.

The open question for someone with Makefile Tools expertise: **is there a `settings.json` key that sets the initial build target on workspace open, so step 2 is not required manually?** The extension's internal workspace state (not `settings.json`) controls the active build target between sessions.
