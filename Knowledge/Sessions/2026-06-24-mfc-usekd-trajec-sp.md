# 2026-06-24 — MFC core: honor `use_trajec_sp`, add guarded `use_Kd`

## What changed
`mfc_core.c/h` (`paparazzi/sw/airborne/firmwares/rotorcraft/stabilization/`).

Two previously-dead fields of `struct MfcParameters` are now functional:

1. **`use_trajec_sp`** — was set by every caller but ignored. `mfc_siso_run` now gates
   the reference-trajectory filter on it. When false: `setpoint_trajec[0] = setpoint`
   (raw), and `dot_dot_setpoint_trajec = 0` (no accel feedforward).
2. **`kd` via new `use_Kd` flag** — the core hardcoded critical damping
   (`a = -2*kp, b = -kp²`). New `uint8_t use_Kd` guards a damping-ratio path:
   `use_Kd=true → a = -2*kd*kp` (ωₙ=kp unchanged, ζ=kd; `kd=1` reproduces critical).
   `use_Kd=false → unchanged critical double pole`.

`mfc_siso_init` now defaults `use_trajec_sp=true`, `use_Kd=false`, so any axis that
doesn't set them keeps current/safe behavior.

## Why
User: both fields were stored but did nothing; wanted `use_trajec_sp` to actually
switch trajectory shaping, and `kd` to be usable instead of assumed critical damping —
without breaking flying airframes (hence the guard + safe defaults).

## Design decision
Kd semantics = **damping ratio ζ** (Option 3): chosen over "raw e˙ coefficient" because
`kd=1` cleanly maps to today's behavior and the number is bounded/intuitive (~0.5–1.5).

## Behavior change to watch
`mfc_gx/gy/gz` (guidance_mfc.c) set `use_trajec_sp=0`. Now honored → guidance
horizontal/vertical **bypass** the trajectory filter. Intended. Stabilization
(roll/pitch/yaw) and `mfc_thrust` set `1` → unaffected. All MFC axes keep `use_Kd=false`,
so damping is byte-for-byte unchanged until someone opts in.

## Verification
`./pprz.sh build ANTON_MFC nps` and `./pprz.sh build ANTON_ONELOOP nps` — both produce
`nps.elf` clean (only pre-existing unrelated warnings). Compile is the only automated
check; flight/sim test still needed for the guidance trajectory-bypass change.

## Next
- Optional: expose `kd` / `use_Kd` / `use_trajec_sp` as GCS `dl_setting`s for live tuning
  (currently not exposed in stabilization_mfc.xml / guidance_mfc.xml).
- NPS sanity-check the guidance horizontal trajectory-bypass behavior.
