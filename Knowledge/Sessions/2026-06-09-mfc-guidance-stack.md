# Session: MFC Guidance Stack Implementation (2026-06-09)

## What Was Built

Full MFC position guidance layer for ANTON_MFC, plus a minimal custom autopilot that wires it to the MFC stabilizer. Both `ap` and `nps` targets compile cleanly.

### New files

| File | Purpose |
|---|---|
| `sw/airborne/firmwares/rotorcraft/guidance/guidance_mfc.c` | MFC SISO guidance for X, Y, Z axes |
| `sw/airborne/firmwares/rotorcraft/guidance/guidance_mfc.h` | Public API declaration |
| `conf/modules/guidance_mfc.xml` | Module XML — build wiring + GCS settings for all 15 gain parameters |
| `conf/autopilot/anton_mfc_autopilot.xml` | Minimal 4-mode autopilot (KILL/FAILSAFE/ATT_DIRECT/NAV) with MFC/INDI stack switch |

### Modified files

| File | Change |
|---|---|
| `sw/airborne/firmwares/rotorcraft/stabilization/mfc_core.h` | Added `bool enabled;` field to `struct MfcParameters` |
| `sw/airborne/firmwares/rotorcraft/stabilization/stabilization_mfc.c` | Wire attitude setpoint in `stabilization_mfc_attitude_run()`; init `enabled=true`; guard `mfc_siso_run()` calls |
| `conf/airframes/ENAC/quadrotor/anton_mfc.xml` | Added autopilot reference, guidance_mfc module, GUIDANCE_MFC section, MODE_AUTO1 fix |
| `conf/airframes/ENAC/conf_enac.xml` | Added guidance_mfc.xml to ANTON_MFC settings_modules |

---

## Design Decisions

### `bool enabled` on MfcParameters
Lets guidance_mfc disable a SISO axis at runtime without crashing stabilization (which still runs for the other axes). Needed for testing altitude-only or horizontal-only without separate code paths.

### `accel_to_att_sp()` in guidance_mfc.c
Converts horizontal MFC acceleration commands to a quaternion attitude setpoint, using the same R_psi rotation pattern as guidance_indi. Includes `Bound()` guards before `asinf()` since `Bound` is a statement macro, not an expression.

### Stack-switch pattern
Two `<control_block>` elements in the autopilot XML (`run_mfc_stack` / `run_indi_stack`) let the developer swap stacks with one line change + rebuild. No runtime branches.

### `guidance_mfc_enter()` in NAV on_enter
Resets SISO integrators on NAV entry to prevent windup from pre-flight. Called even in INDI stack mode (harmless no-op for INDI).

---

## Bugs Found and Fixed

### 1. `Bound()` is a statement, not an expression
`asinf(Bound(val, -1.f, 1.f))` fails to compile — `Bound` expands to `{ if (...) {...} }`.
**Fix:** use intermediate variable: `Bound(val, -1.f, 1.f); result = asinf(val);`

### 2. `&amp;&amp;` in autopilot XML conditions
XML `&amp;&amp;` in `cond=` attributes was passed verbatim to generated C, causing parse errors.
**Fix:** use plain `&&` — the autopilot XML parser is not strict-XML.

### 3. `AP_MODE_ATTITUDE_Z_HOLD` in anton_mfc.xml
`MODE_AUTO1` referenced a mode that doesn't exist in the 4-mode custom autopilot.
**Fix:** Changed to `AP_MODE_ATTITUDE_DIRECT`.

### 4. Makefile.ac caching: AP build reuses NPS-generated Makefile.ac
Generator skips overwriting `Makefile.ac` if it's newer than the XML sources. After an NPS build, the AP build reused the NPS file (which lacks `USE_GENERATED_AUTOPILOT=TRUE` in the AP block) → `autopilot_static.c` compiled alongside generated header → duplicate `case` values.

**Root cause:** `sw/lib/ocaml/aircraft.ml:478` only sets `autopilot=true` for the target specified with `-target`. The generator at `gen_aircraft.ml:351` skips copying the newly-generated Makefile.ac if the existing one is newer.

**Fix:** `rm paparazzi/var/aircrafts/ANTON_MFC/Makefile.ac` before switching from NPS→AP builds.

---

## Key Learnings

### Paparazzi autopilot system
- Custom autopilots activated by `<autopilot name="..."/>` at `<firmware>` level
- Mode integers are sequential (0, 1, 2, …) in XML declaration order  
- `USE_GENERATED_AUTOPILOT=TRUE` must appear inside the target's ifeq block in Makefile.ac
- Makefile.ac is shared across all targets but regenerated per-target; caching prevents correct regeneration on target switch

### build_fw.sh / Makefile.ac lifecycle
- `build_fw.sh AIRCRAFT CONF XML TARGET` calls `gen_aircraft.out -all -target TARGET`
- Generator writes `USE_GENERATED_AUTOPILOT=TRUE` only for the current target's block
- If Makefile.ac exists and is newer than XML sources, it is NOT overwritten
- Always delete Makefile.ac when switching from one target build to another for a fresh aircraft

---

## What's Next

1. **NPS functional test** with `sim_anton.py --mfc`: verify hover stability and waypoint tracking in simulation before any flight hardware changes
2. **Gain tuning**: GZ_ALPHA=32.7 (g/hover_throttle), GZ_KP=0.5 are starting points; GX/GY alpha and kp need in-sim tuning
3. **Flight test on ANTON**: after NPS validation, flash ap.elf to Tawaki board
4. **Merge to main branch**: changes are in worktree `mfc-guidance-stack`; after flight validation, merge to `main`

---

## Final verified state

| Target | Output | Size |
|---|---|---|
| `ap` (ChibiOS/ARM Cortex-M) | `var/aircrafts/ANTON_MFC/ap/obj/ap.elf` | 393 832 text + 24 828 data |
| `nps` (Linux sim) | `var/aircrafts/ANTON_MFC/nps/obj/simsitl` | links cleanly |

Both targets build cleanly in both directions (NPS→AP and AP→NPS) with the `build_fw.sh` fix in place.

### Makefile.ac caching fix (follow-up)

The NPS build failed after the AP build because the generator stamps `USE_GENERATED_AUTOPILOT=TRUE` only in the current target's `ifeq` block. Alternating builds always left the other target's block without the flag, causing `autopilot_static.c` + `autopilot_core_ap.h` to be compiled together → duplicate `case` values.

**Fix:** `build_fw.sh` now deletes `var/aircrafts/<AC>/Makefile.ac` before every build. The generator always regenerates it fresh for the current target. One-line addition to the build script.

Files are in `/workspace/paparazzi/` (main workspace). Not yet merged to `main` branch.
