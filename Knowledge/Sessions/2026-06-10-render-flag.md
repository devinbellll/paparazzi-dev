# Session: 2026-06-10 — sim_anton render flag

## What changed

**`sim_anton.py`** — made the TUI dashboard opt-in.

- Added `--render` flag (`_USE_RENDER`).
- Default mode (no flag): plain stdout debug log — one line per 100 ms, showing either the latest `printf` line captured from `simsitl` or a bare state summary (`alt / phi / theta / psi`).
- `--render` mode: unchanged rich TUI with bar graphs, INDI table, MFC table.
- Updated the module docstring and usage line to reflect the new flag.

**Rationale:** now that the in-process NPS scope (`nps_scope.c`) streams high-rate truth + controller internals directly to PlotJuggler, the terminal TUI is redundant for observability. The scope is the default instrument; the TUI is a fallback/demo. Plain log output is quieter and keeps the terminal clean when PlotJuggler is the active viewer.

## What was learned

Nothing architecturally new — this was a UX trim to match the workflow now that the scope emitter exists.

## Context from cerebrum

- The in-process scope emitter (`nps_scope.c`) runs by default (`--no-scope` disables it). It streams JSBSim truth + all firmware-registered vars at ~500 Hz (decim 2 at 1 kHz SYS_TIME_FREQUENCY) to PlotJuggler on `host.docker.internal:9870`.
- The legacy Python `scope_writer` (Ivy-derived, 50 Hz, walltime-stamped) is behind `--debug-scope` on port 9871. It still exists but is rarely needed.
- MFC guidance stack (`guidance_mfc.c`) with three independent SISO loops (GX, GY, GZ) was the last major feature checked in (branch `mfc-guidance-stack`, commit `65e98f5`). It is untested at flight — first sim run will validate the guidance loop integration.

## Latest changes this session

**`SITL_PRINT` now includes call-site file and line** — modified `SITL_PRINT_HZ` in `paparazzi/sw/airborne/paparazzi.h` to prefix every print with `[filename:line]`. Uses `__FILE_NAME__` (GCC 12+ extension, available on the 13.2 toolchain) for a short filename rather than the full path. Output: `[stabilization_indi.c:247] in_flight: 1`.

## Further changes this session

**Raw firmware printf streaming** — replaced the polling/deduplication approach with direct printing in `sim_stdout_reader`. Each line from simsitl stdout now prints immediately as it arrives (no `[dbg]` prefix, no dedup). The idle main loop in non-render mode just sleeps. Confirmed working: JSBSim startup banner + `stabilization_indi.c - in_flight:` loop lines stream in real time.

**Ivy connection callback fix** — `IvyInit` 4th/5th args changed from `None` to `lambda a, b: None` to prevent `TypeError: 'NoneType' object is not callable` on connection events.

## Additional changes this session

**Debug log deduplication** — default (no `--render`) mode now only prints when the firmware `printf` line changes, not every 50 ms tick. Tracks `last_dbg` and skips identical lines. Eliminates the spam of repeated `[dbg]` lines at 20 Hz.

**Ivy `NoneType` callback fix** — `IvyInit` fourth and fifth args (on-die, on-connect) were `None`; Ivy calls them with two positional args on connection events, crashing with `TypeError: 'NoneType' object is not callable`. Fixed by passing `lambda a, b: None` for those two slots.

## Fast-build version-macro fix (`build_active.sh`)

**Symptom:** intermittent compile failure during `build_active.sh "<AC> (nps)" fast`:
```
pprz_version.h:58 error: invalid type argument of unary '*' (have 'int')
  #define PPRZ_VERSION_INT (PPRZ_VER_MAJOR * 10000 + PPRZ_VER_MINOR * 100 + PPRZ_VER_PATCH)
```
Hit only in `autopilot.c` (`send_autopilot_version`), and only "sometimes — when more files are affected."

**Root cause:** the `fast` path runs `make -C paparazzi/sw/airborne all` directly, bypassing `Makefile.ac`, which is where `GIT_DESC` / `PPRZ_VER` / `PPRZ_VER_MAJOR|MINOR|PATCH` are computed and exported (`Makefile.ac:80-95`). Without them, `sw/airborne/Makefile:59-64` emits `-DPPRZ_VER_MAJOR=` (empty) into CFLAGS, so the macro expands to `( * 10000 + * 100 + )`. Only `autopilot.c` expands `PPRZ_VERSION_INT`, so the broken define is latent until an edit forces `autopilot.o` to recompile — hence the intermittency.

**Fix:** `build_active.sh` fast block now recreates the version vars (mirroring `Makefile.ac`, including the `PPRZ_VER_PATCH=0` guard since `v7.0_unstable` has no patch field) and passes `GIT_SHA1/GIT_DESC/PPRZ_VER/PPRZ_VER_MAJOR/MINOR/PATCH` on the make command line. User verified the build manually. Logged as `bug-020` (sibling of `bug-013` — both are fast/incremental builds not getting `Makefile.ac`'s environment for the target).

## What is next

1. **First sim run of MFC guidance stack** — run `sim_anton.py --mfc` (or the guidance variant), watch PlotJuggler for guidance/position response, verify `mfc_guidance` commands feed into the attitude setpoint correctly.
2. **Verify `--render` default vs. scope** — confirm the plain debug log is readable when the scope is also streaming; no interleaving issues expected since both are independent paths.
3. **AP build test** — `./build_fw.sh ANTON_MFC conf/airframes/ENAC/conf_enac.xml ap` (remember: delete `Makefile.ac` first when switching from NPS→AP target on this aircraft).
