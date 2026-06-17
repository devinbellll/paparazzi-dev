# 2026-06-17 — Build tooling overhaul: one build button + working IntelliSense

## Goal

The post-containerization tasks were clunky: a fast/full build split, a `db`
command that "just errored", and ambiguous conf paths. Wanted: (1) clangd
IntelliSense that matches the real defines/includes for the selected config,
(2) a single Shift+Cmd+B build button with CMake-like incrementality, (3)
launch-after-build-if-needed, (4) a clean button. Existing build scripts treated
as expendable; re-assess how Paparazzi is actually meant to be built.

## Key findings

- **`db` errored because of `bear`.** bear 3.x intercepts compilers via a wrapper
  that phones home to a local gRPC daemon; inside the build container that loopback
  fails (`gRPC call failed ... Connection refused`), so every compile under bear
  fails and no `compile_commands.json` is written (only an orphan
  `compile_commands.events.json`). Fixed by dropping bear for **`compiledb`** (pure
  Python, parses make's verbose stdout). (bug-051)
- **Paparazzi's `<target>.compile` is already CMake-like.**
  `make -C paparazzi -f Makefile.ac AIRCRAFT=.. CONF_XML=.. <target>.compile`
  → `<target>.ac_h` (gen_aircraft regenerates `airframe.h`/`modules.h` only when the
  XML changed — `is_older` mtime check + md5 of `conf_aircraft.xml`) → `sw/airborne`
  incremental make. The only redundant slow step in the old `build_fw.sh` was
  `make -j1 -C paparazzi` (ground segment) on *every* build — that's what the
  fast/full split was working around. Gate it behind a guard and the split
  disappears.
- **Conf path was inconsistent and the common docs form was wrong.** gen_aircraft
  resolves `-conf` relative to make's cwd (`paparazzi/`), so the canonical form is
  `conf/airframes/ENAC/conf_enac.xml`. The `paparazzi/conf/...` form in
  CLAUDE.md/README/build_fw.sh would resolve to `paparazzi/paparazzi/conf/...`.

## Changes

- **`pprz.sh` rewritten** as the single build tool: `pprz.sh <cmd> AIRCRAFT [TARGET]`
  (also accepts the VSCode picker's `"AIRCRAFT (target)"`). Commands: `build`
  (incremental), `clean`, `rebuild`, `db` (compiledb), `codegen`, `bootstrap`.
  - `_ensure_core` builds the ground segment only when generators/`var/include` are
    missing — not per build.
  - `_guard_target_switch` records the last target per aircraft
    (`var/aircrafts/<AC>/.pprz_last_target`) and forces a clean codegen of the shared
    `Makefile.ac` only on ap↔nps switch, keeping same-target rebuilds fully
    incremental (replaces the manual `rm Makefile.ac` ritual).
  - `db` does `clean_ac` → verbose build (`USE_VERBOSE_COMPILE=yes Q=''`) teed to
    `build_verbose.log` → `compiledb --parse`. Tolerates a final link failure
    (per-file compile commands are all clangd needs).
- **`build_fw.sh` removed**; `sim.sh` now calls `pprz.sh build <AC> nps`.
- **`Dockerfile.build`**: `bear` → `compiledb` (pip, `--break-system-packages`).
- **`.vscode/tasks.json`**: single **Build** (default, Shift+Cmd+B) + Clean,
  Rebuild, IntelliSense: regenerate compile DB, Codegen only, Bootstrap, Build
  image, Sim tasks. No more fast/full, no `log` task.
- **Docs**: CLAUDE.md, README.md, Knowledge 06/07/08/10, build_image.sh,
  pprz_docker.sh updated to the new interface + canonical conf path.

## Verified

- `pprz.sh db "ANTON_MFC (ap)"` → **228 entries**, exit 0, `/workspace`→host paths
  rewritten and resolving to real files, MFC config defines (`-DMFC_NUM_ACT=4`,
  `-DSTABILIZATION_ATTITUDE_INDI_FULL=true`) captured.
- Incremental build: full nps **51s** → no-op rebuild **10s (0 compiles)** →
  one-file touch of `guidance_indi.c` **11s (exactly 1 TU recompiled + relink)**.

## Flagged (not fixed — out of scope)

- **ANTON_MFC `ap` does not link** (bug-052): `nps_scope_z_sp`/`nps_scope_z_ref`
  defined unconditionally in `guidance_v.c:65-66`, used in `guidance_indi.c:523-524`,
  but on `ap` the `NPS_SCOPE_VAR` registration that anchors them is compiled out
  (`USE_NPS` undefined) so `--gc-sections` drops the definitions. Same class as
  bug-050. `nps` links fine. Fix: guard the `guidance_indi.c` uses with
  `#if defined(USE_NPS)` (or keep the `guidance_v.c` defs).

## Next

- Optionally fix bug-052 so `ap` links (small, in the user's MFC code).
- On the Mac, set `PPRZ_HOST_CC` to a brew `arm-none-eabi-gcc` for full system-header
  fidelity in clangd (optional; project headers + generated defines already resolve).
