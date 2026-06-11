# Session: MFC Per-Axis Timing + Workaround Removal (2026-06-11)

## Outcome

**MFC guidance on the Z axis flies — perfectly smooth in NPS (user-confirmed).** The
state was frozen with commit `24dcae6` ("save: MFC Z guidance working beautifly").
This session refactored the MFC clock from a global singleton into per-SISO-module
timing, then removed the timing workarounds that the global clock had required.

## What Changed

### 1. Timing moved into each SISO module

Previously `mfc_core.c` held a single global `struct Mfc mfc` with the shared
`sample_time` / `time` / `start_time`. Every axis read the same clock.

Now each axis owns its clock — `struct MfcParameters` (in `mfc_core.h`) gained:

```c
float sample_time;   // 1 / control_frequency for THIS axis
float time;          // seconds since this axis was last reset
float start_time;    // latched on reset
```

The global `struct Mfc` and `extern mfc` were deleted.

### 2. API: per-module init + reset

| Old (global)        | New (per-axis)                                   |
|---------------------|--------------------------------------------------|
| `mfc_core_init(dt)` | `mfc_siso_init(struct MfcParameters*, dt)`       |
| `mfc_core_start()`  | `mfc_siso_reset(struct MfcParameters*)`          |

`mfc_siso_reset()` latches `start_time`, zeros `time`, and clears the time-dependent
history (`error`, `estimator_num/den`, `command`, `estimator`) so the estimator
restarts clean. `mfc_siso_run()` now reads `mfc_stt->time` / `->sample_time` instead
of the globals; its algorithm body is otherwise unchanged.

### 3. Callers updated

- `stabilization_mfc.c` — `init` calls `mfc_siso_init` for roll/pitch/yaw; `enter`
  calls `mfc_siso_reset` for each.
- `guidance_indi.c` — `mfc_siso_init`/`mfc_siso_reset` on the `mfc_thrust` axis.
- `guidance_mfc.c` — `mfc_siso_init` for gx/gy/gz; reset on the respective enters.

### 4. Removed the global-clock timing workarounds (bug-033)

Because the clock is now per-module, the bug-030 / bug-031 hacks were obsolete and
were removed (user chose "full raw MFC"):

- **bug-030 reverted** — dropped the "only `guidance_v_run_enter()` starts the clock,
  `guidance_h_run_enter()` is a no-op" split. Now `guidance_v_run_enter()` resets
  `gz`, `guidance_h_run_enter()` resets `gx`/`gy`.
- **bug-031 reverted** — dropped the `!in_flight` ground-gating (per-cycle clock
  re-latch + zero-cmd + nominal-hover output). `guidance_mfc_vert()` now runs MFC
  every cycle whenever `mfc_gz.enabled`; the `else` is just the gz-disabled fallback
  (`GUIDANCE_MFC_GZ_NOMINAL_HOVER_THROTTLE`).

### 5. bug-032 resolved

The vertical thrust-units blocker (`v_thrust.z = thrust->sp.thrust_f[THRUST_AXIS_Z]`
at `stabilization_indi.c:741`) is resolved by **tuning, not reversion**: the edit is
retained (option 2 from the bug-032 diagnosis) and GZ was retuned so the MFC output
enters the WLS `indi_v[3]` objective at the correct scale and sign.

## Files Touched

| File | Change |
|---|---|
| `stabilization/mfc_core.h` | moved timing fields into `MfcParameters`; deleted `struct Mfc`; new `mfc_siso_init`/`mfc_siso_reset` decls |
| `stabilization/mfc_core.c` | implemented init/reset; `mfc_siso_run` uses per-axis timing; `#include math/pprz_algebra_float.h` for `float_vect_zero` |
| `stabilization/stabilization_mfc.c/.h` | per-axis init/reset; updated stale comments |
| `guidance/guidance_indi.c` | per-axis init/reset on `mfc_thrust` |
| `guidance/guidance_mfc.c` | per-axis init/reset; removed v/h-enter split and in_flight ground gating |

## Known Trade-offs (intentional)

1. **Ground windup is back.** Raw MFC pre-liftoff lets the algebraic estimator wind up
   on the ground (the original bug-031 symptom). Accepted per user request.
2. **gx/gy reset every ground cycle.** `guidance_h_run_enter()` is called every cycle
   while `!in_flight`, so the horizontal axes' clocks are pinned near 0 on the ground
   (estimator gated until liftoff). Inert today — gx/gy are `enabled = false`.

## What's Next

1. Flight test on ANTON hardware (NPS Z is validated).
2. Enable/tune horizontal MFC (gx/gy); revisit the every-cycle reset noted above if so.
3. Merge `mfc-guidance-stack` → `main` after flight validation.
