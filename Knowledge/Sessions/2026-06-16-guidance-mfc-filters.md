# 2026-06-16 — guidance_mfc: Butterworth filters on NED measurement and thrust

## What changed

Added `Butterworth2LowPass` filters to `guidance_mfc.c`, mirroring the filtering pattern in `guidance_indi.c`.

**File:** `sw/airborne/firmwares/rotorcraft/guidance/guidance_mfc.c`

### Filters added

| Filter | Variable | Applied to | Used by |
|--------|----------|------------|---------|
| `filt_pos_ned[0]` | x NED position | raw `pos->x` before `mfc_siso_run(gx)` | gx MFC |
| `filt_pos_ned[1]` | y NED position | raw `pos->y` before `mfc_siso_run(gy)` | gy MFC |
| `filt_thrust` | thrust output | raw `thrust_cmd` after `mfc_siso_run(gz)` | `accel_to_att_sp()` only |

**gz (vertical) deliberately gets raw NED z** — no position filter on the altitude measurement.

**The stabilizer receives the raw `thrust_cmd`** — the `filt_thrust` output goes only to `mfc_thrust_physical`, which `accel_to_att_sp()` uses for the tilt-scaling denominator (divides desired acceleration by T/m to get attitude angle).

### New define

```c
#ifndef GUIDANCE_MFC_FILTER_CUTOFF
#define GUIDANCE_MFC_FILTER_CUTOFF 3.0f  // Hz, same default as guidance_indi
#endif
```

Overridable per-airframe in the `GUIDANCE_MFC` XML section.

### Mode-entry re-seeding

- `guidance_v_run_enter()`: re-inits `filt_thrust` at 0 to avoid transient
- `guidance_h_run_enter()`: re-inits `filt_pos_ned[0/1]` at current `pos->x/y` to avoid step kick

## Current known hack (not fixed this session)

Line 419 has a debug hardcode left from commit `9af03cff2`:
```c
mfc_thrust_physical = -12; // filt_thrust.o[0];
```
`-12` is `GUIDANCE_MFC_GZ_NOMINAL_HOVER_THROTTLE` — this pins `accel_to_att_sp()` to a constant denominator, which bypasses the filter entirely. This is intentional for now while the attitude saturation issue (noted in the commit) is being investigated. The filter infrastructure is in place; uncommenting `filt_thrust.o[0]` and removing `-12` activates it.

## Context from commits

- `c8996b618`: MFC X working — specifically required filter on NED measurement + command filter + trajectory filter
- `9af03cff2`: Three-axis MFC guidance functional but attitude saturation still present; hack on `accel_to_att_sp` denominator in place

## Next steps

- Decide whether to enable `filt_thrust.o[0]` for the `accel_to_att_sp` denominator (remove the `-12` hack)
- Investigate attitude saturation in three-axis mode
- Consider whether gz also needs a position filter once vertical loop is more stable
