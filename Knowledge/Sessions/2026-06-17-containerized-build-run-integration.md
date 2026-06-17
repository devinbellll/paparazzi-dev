# 2026-06-17 — Containerized build/sim + VSCode integration

## Goal

Stop running Claude *inside* the Paparazzi build environment. Claude now runs in a
lean aarch64 **sbx sandbox**; builds and the NPS sim should run in **ephemeral
containers** instead, with full VSCode(host) ↔ Claude(sbx) ↔ build-system integration.

## What the runtime actually is (verified)

- aarch64-native sandbox (`IS_SANDBOX=1`, `SANDBOX_VM_ID=claude-paparazzi-dev`),
  **no toolchain** (`arm-none-eabi-gcc` absent). Repo mounted at its real host path,
  direct mode (edits shared host↔sandbox instantly).
- Nested **arm64 Docker daemon (DinD)** sharing the sandbox filesystem → containers
  can bind-mount the repo and drop outputs back in the tree. Image pulls work.
- No qemu/binfmt (amd64 emulation needs a privileged install).
- Host Mac reachable only as a network host (`host.docker.internal`), not a shell →
  host-native builds can't be driven from the sandbox.

## Decisions

- **Ephemeral arm64-native containers** per build/sim. Firmware is cross-compiled
  (`arm-none-eabi` → Cortex-M), so the `.elf` is identical to the old amd64/Rosetta
  build; faster, no emulation. `USE_LTO=no` kept only for fast incremental dev.
- **NPS**: run sim + OCaml ground segment + IVY together in one container; stream
  only PlotJuggler(:9870)/FlightGear(:5501) to the Mac (as `sim_anton.py` already
  assumed). No native-macOS OCaml install.
- **VSCode = native Mac + Docker Desktop (Option B)**: VSCode dispatches to its own
  daemon, Claude to the sandbox's; both write the same shared files. Needs
  `/workspace`→host path rewrites + clangd IntelliSense.
- Consolidated the four config-driven scripts into `pprz.sh`.

## Deliverables

- `Dockerfile.build`, `build_image.sh`, `pprz_docker.sh` (dispatcher).
- `build_fw.sh` self-dispatches into the container when no local toolchain.
- `sim.sh` (+ `sim_anton.py --gdb` switched from `qemu-x86_64` to `gdbserver`).
- `pprz.sh {build|db|log|codegen}` — replaces build_active/gen_compile_db/
  gen_build_log/gen_vscode; `db` rewrites `/workspace`→host paths for clangd.
- `.clangd` + `.vscode/{tasks,launch,settings,c_cpp_properties}.json` — clangd
  IntelliSense (MS engine disabled), active-config picker still drives the tasks,
  gdbserver-attach debug with `/workspace`→repo `sourceFileMap`.
- CLAUDE.md build/run + IDE sections updated.

## BLOCKER — image not yet built

`./build_image.sh` fails at `add-apt-repository ppa:paparazzi-uav/ppa` (SSL EOF):
the sandbox default-deny policy blocks the PPA domains. Probe results:
`ports.ubuntu.com`/`launchpad.net` allowed; **blocked**: `api.launchpad.net`,
`keyserver.ubuntu.com`, `ppa.launchpadcontent.net`.

**Unblock on the host:**
```
sbx policy allow network api.launchpad.net,keyserver.ubuntu.com,ppa.launchpadcontent.net
```
Then `./build_image.sh`. If the PPA has no arm64 binaries → `./build_image.sh --amd64`.

## Next / verification (after unblock)

1. `./build_image.sh` succeeds (or `--amd64` fallback).
2. `./build_fw.sh ANTON paparazzi/conf/airframes/ENAC/conf_enac.xml` → `ap.elf`.
3. `./pprz.sh db "ANTON_MFC (ap)"` → host-path `compile_commands.json`; clangd
   resolves a generated `#define` in `stabilization_indi.c`.
4. Touch one `.c`, `pprz.sh build "... (ap)" fast` → only that TU recompiles.
5. `./sim.sh --mfc` → IVY telemetry + PlotJuggler :9870 on the Mac.
6. `sim.sh --mfc --gdb` + publish :1234, attach via *Attach: NPS sim C*.

## Open runtime questions (couldn't verify from inside)

- PPA arm64 availability (api.launchpad.net blocked).
- Sim UDP egress to the Mac from the sbx sandbox vs Mac Docker Desktop networking.
- Mac-side debugger: needs a Linux-arm64-capable gdb (gdb-multiarch) for the attach.
- Best clangd system-header fidelity wants a Mac `arm-none-eabi-gcc` (PPRZ_HOST_CC).

## Verified outcomes (end of 2026-06-17)

- **arm64 PPA confirmed**: `paparazzi-dev` 3.31 + `paparazzi-jsbsim` 1.7 resolve on arm64 (787 pkgs, 0 broken). OCaml ground segment is prebuilt for arm64 — no source build, no amd64 emulation, no Rosetta. The old "can't compile OCaml for arm64" worry was a conflation with native-macOS OCaml.
- **End-to-end build works from the sandbox**: clean `./build_fw.sh ANTON conf/airframes/ENAC/conf_enac.xml nps` → native-arm64 `simsitl` (ELF machine 0xb7), owned by uid 1000, in the shared tree.
- **Host parity verified by user**: same commands (`./build_image.sh`, `./build_fw.sh`, `./sim.sh`) work on the Mac (Docker Desktop, arm64-native). User confirmed building on host, running sims on host, and running the sandbox-built binary all "worked perfectly".
- **Cross-arch contamination is the main hazard**: all four actors share one tree; native OCaml tooling (`sw/ext/pprzlink/build/.../myocamlbuild`, `sw/lib/ocaml/dlllib-pprz.so`) is arch-specific. Mixing amd64 (host) + arm64 (sandbox) → "Exec format error" / "cannot load shared library". RULE: everyone arm64; `make clean` in-container to recover; never build host+sandbox concurrently.
- **Fixes made this session**: `pprz_docker.sh` `pprz_platform()` auto-detects image arch (was hardcoded arm64 → "Unable to find image locally" on amd64 images); `Dockerfile.build` drops `libboost-program-options-dev` so `make ext` self-skips `unifiedmocaprouter` (NatNet SDK is x86-64-only, unbuildable on arm64); `sim.sh` TTY-detects `-t` for headless runs and points the sandbox container's `host.docker.internal` at the Mac's real IPv4 (sbx injects `169.254.1.1`) instead of the dead-end DinD bridge gateway.

### Next (cleanup)
- Standardize host on arm64 image (drop `--amd64` path or keep only as documented fallback).
- Streaming telemetry from the SANDBOX run to the Mac is deprioritized (sandbox runs are for isolated behaviour checks / future automated tests, not visualization).
