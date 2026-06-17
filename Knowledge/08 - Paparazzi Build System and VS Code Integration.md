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

## Current VS Code Setup (2026-06-17 — clangd + ephemeral containers)

> The earlier Makefile-Tools/`make -n` approach (above) is superseded. It never
> worked reliably because Paparazzi's source list isn't resolved until a real
> build runs (so `make -n` can't see the TUs). We now drive IntelliSense from a
> real `compile_commands.json` captured with `bear`, and run all builds in
> ephemeral containers. Builds happen in a Linux container, but VSCode reads files
> at the Mac repo path, so container `/workspace` paths are rewritten to the host
> path in every emitted artifact.

**One picker drives everything.** The C/C++ status-bar config name
(`"AIRCRAFT (target)"`, from `c_cpp_properties.json`) feeds the tasks via
`${command:cpptools.activeConfigName}`. The MS C/C++ IntelliSense engine is
**disabled**; clangd provides IntelliSense from `compile_commands.json`.

### `pprz.sh` — consolidated build/IDE tool

Replaces the former `build_active.sh` / `gen_compile_db.sh` / `gen_build_log.sh` /
`gen_vscode.sh`. Each subcommand runs inside the toolchain container at `/workspace`:

```bash
./pprz.sh build   "ANTON_MFC (ap)" [fast]   # full, or fast airborne-only incremental
./pprz.sh db      "ANTON_MFC (ap)"          # compile_commands.json (clangd) + host rewrite
./pprz.sh log     "ANTON_MFC (ap)"          # verbose build.log + host rewrite
./pprz.sh codegen "ANTON_MFC (ap)"          # regenerate airframe.h / modules.h only
```

### `tasks.json`

`build: active config (fast|full)`, `db/log/codegen: … active config`,
`image: build toolchain`, and `sim:` tasks — all call `pprz.sh` / `sim.sh` /
`build_image.sh` against the active config.

### `compile_commands.json` (the IntelliSense source)

`./pprz.sh db "<config>"` runs a clean build under `bear` in the container, writes
`var/aircrafts/<AC>/<target>/compile_commands.json`, copies it to the repo root,
and rewrites `/workspace` → the host repo path. clangd (`.clangd`,
`--compile-commands-dir`) reads the root copy. Re-run `db` after changing the
airframe XML, adding/removing files, or changing build-affecting defines.

### System-header fidelity (optional)

The DB's compile commands use the ARM cross-compiler. For full bare-metal system
headers, install a Mac `arm-none-eabi-gcc` (`brew install --cask gcc-arm-embedded`)
and regenerate with `PPRZ_HOST_CC=/path/to/arm-none-eabi-gcc ./pprz.sh db "<config>"`
— the rewrite then points clangd at it via `--query-driver`. Without it, project
headers and generated `#define`s still resolve.

---

## What needs to work for IntelliSense (current)

1. Build the image once (`./build_image.sh`; see CLAUDE.md for the PPA allowlist).
2. `./pprz.sh db "ANTON_MFC (ap)"` → host-path `compile_commands.json` at repo root.
3. clangd indexes it automatically; open any firmware `.c` and generated `#define`s
   and the include graph resolve. (Re-run `db` after XML/file/define changes.)
