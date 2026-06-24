# 2026-06-24 — MFC Gain Propagation

## Summary

After successful MFC tuning, propagated all hardcoded gains from the truth sources (`stabilization_mfc.c`, `guidance_mfc.c`) to every MFC-related file so airframe XML overrides actually work.

## Root Cause (why this was broken)

All three `.c` init functions used local `#define` macros (e.g., `MFC_ATT_ALPHA`, `MFC_XY_POLES`) or literal values that completely bypassed the `#ifndef`-guarded `STABILIZATION_MFC_*` / `GUIDANCE_MFC_*` public defines. This meant:
- Airframe XML `<define>` overrides had zero effect at runtime
- The `#ifndef` defaults were stale documentation, not operational code
- `oneloop_mfc.c` had silently drifted from the tuned values

## Files Changed

### `stabilization_mfc.c`
- Updated `#ifndef` defaults: ROLL/PITCH TIME_TRAJECTORY=50, INTEGRATION_WINDOW=5, ALPHA=2, KP=6; YAW TIME_TRAJECTORY=0, ALPHA=1, KP=1, IW=1
- Removed local `MFC_ATT_*` macro block
- Replaced all init assignments to use `STABILIZATION_MFC_ROLL/PITCH/YAW_*` defines
- Added `kd` and `use_trajec_sp` fields from defines (default use_trajec_sp=1)

### `guidance_mfc.c`
- Updated `#ifndef` defaults: GX/GY TIME_TRAJECTORY=10; GZ TIME_TRAJECTORY=4, IW=4, CF=8
- Removed local `MFC_XY_*` macro block
- Replaced GX/GY/GZ inits with `GUIDANCE_MFC_GX/GY/GZ_*` defines
- Fixed `mfc_thrust_physical = -12` → `GUIDANCE_MFC_GZ_NOMINAL_HOVER_THROTTLE`

### `oneloop_mfc.c`
- Updated stab `#ifndef` defaults to match `stabilization_mfc.c`
- Updated guidance `#ifndef` defaults to match `guidance_mfc.c`
- Removed local `MFC_ATT_*` macro block (was: ALPHA=147, FHAT_FILTER=1, POLES=4)
- Updated stab init: all axes now use `STABILIZATION_MFC_*` defines; `use_trajec_sp=1`; yaw enabled
- Updated guidance init: all axes use `GUIDANCE_MFC_*` defines; `enabled` from define
- Fixed `mfc_thrust_physical = -12` → `GUIDANCE_MFC_GZ_NOMINAL_HOVER_THROTTLE`

### `stabilization_mfc.xml` (module doc defaults)
- ROLL/PITCH ALPHA: 147.0588 → 2., KP: 4. → 6.
- YAW TIME_TRAJECTORY: 50. → 0., IW: 5. → 1., ALPHA: 73.5294 → 1., KP: 4. → 1.

### `guidance_mfc.xml` (module doc defaults)
- GX/GY TIME_TRAJECTORY: 600. → 10.
- GZ TIME_TRAJECTORY: 250. → 4., IW: 5. → 4., CF: 1. → 8.

### `anton_mfc.xml` (airframe explicit defines)
- STAB: ROLL/PITCH ALPHA: 147.0588 → 2., KP: 4. → 6.; YAW TT: 50.→0., IW: 5.→1., ALPHA: 73.5294→1., KP: 4.→1.
- GUID: GX/GY TT: 1.→10.; GZ TT: 1.→4., IW: 5.→4., CF: 1.→8.

## Not Changed

- `anton_mfc_thrust.xml`: uses `THRUST_MFC_*` for `GUIDANCE_INDI_THRUST_MFC` (separate subsystem, independent of MFC guidance stack)
- `stabilization_dual_mfc_indi.xml` / `guidance_dual_mfc_indi.xml`: no hardcoded MFC defaults — they delegate to the shared defines

## Tuned Values (truth)

| Axis | alpha | kp  | time_trajec | int_window | cmd_filter |
|------|-------|-----|-------------|------------|------------|
| Roll | 2     | 6   | 50          | 5          | 1          |
| Pitch| 2     | 6   | 50          | 5          | 1          |
| Yaw  | 1     | 1   | 0           | 1          | 1          |
| GX   | 10    | 0.8 | 10          | 5          | 1          |
| GY   | 10    | 0.8 | 10          | 5          | 1          |
| GZ   | 5     | 4   | 4           | 4          | 8          |

## What's Next

- Flight test to verify `oneloop_mfc.c` behaves identically to standalone `stabilization_mfc.c` + `guidance_mfc.c` now that gains are in sync
- Consider removing the explicit `<section name="STABILIZATION_MFC">` / `<section name="GUIDANCE_MFC">` blocks from `anton_mfc.xml` if they exactly match module defaults (optional cleanup)
