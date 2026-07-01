# 2026-06-30 — MFC runtime tuneability (allocator + thrust toggles, unified shortnames)

## Goal
Flight testing the MFC stack: make the GCS settings panels actually tunable.
1. Expose the allocator select (`pseudo_inverse`) and thrust packaging (`thrust_to_pprz`)
   as **runtime** dl_settings instead of compile-time `#if` defines.
2. Give every MFC settings field a **clear, unique shortname** (roll/pitch/yaw were
   all showing the same `TT`/`IW`/`ALP`/`KP`/`CF` chips).

## What changed

### Runtime toggles (C)
Followed the existing `mfc_use_adaptive` pattern: keep the `#ifndef…#define…FALSE`
compile-time default, initialise a `bool` from it, branch with a plain `if`.

- **`stabilization_mfc.c/.h`** — added `bool stabilization_mfc_use_pseudo_inverse`.
  De-guarded both allocator paths so they compile together: WLS state
  (`wls_stab_p`, `act_pref`, `stabilization_mfc_set_wls_settings`, the `WLS_V/WLS_U`
  NPS-scope regs + WLS telemetry) AND the pseudo-inverse (`calc_g1g2_pseudo_inv`,
  `g1g2_pseudo_inv`). The run-loop allocator is now `if (use_pseudo_inverse) {…} else {…}`.
  `calc_g1g2_pseudo_inv()` is called unconditionally in init **and** in lms, so a live
  switch never hits uninitialised state. `.h`: `wls_alloc.h` include + `extern wls_stab_p`
  are now unconditional.
- **`oneloop_mfc.c/.h`** — identical change set, var named `oneloop_mfc_use_pseudo_inverse`
  (file-local globals stay `static` except the new bool, which is externed for settings).
- **`guidance_mfc.c/.h`** — added `bool guidance_mfc_thrust_to_pprz`; the thrust-packaging
  branch (int-pprz vs float) is now runtime. **Left `stabilization_mfc.c:779`
  (`#if GUIDANCE_MFC_THRUST_TO_PPRZ`) compile-time** on purpose — see Decisions.

### Settings panels (XML)
Unified shortname scheme `<axis>_<field>` (traj/intwin/alpha/kp/cfilt; axis ∈
roll/pitch/yaw or gx/gy/gz):
- `stabilization_mfc.xml` — replaced colliding TT/IW/ALP/KP/CF; added `alloc_pseudo_inv`
  (`WLS|PSEUDO_INV`) to the `mfc` group.
- `guidance_mfc.xml` — gx/gy/gz already conformed; added a `guidance_mfc_thrust` group
  with `thrust_to_pprz` (`FLOAT|PPRZ`).
- `oneloop_mfc.xml` — renamed TT/ALP/KP → traj/alpha/kp (stab + guidance); added
  `alloc_pseudo_inv`.
- `dual_*` XMLs — **no change**: their MFC side is `oneloop_mfc` (via `<depends>`), so a
  dual airframe inherits oneloop's panel; their own shortnames are already unique.

## Decisions / learnings
- `wls_alloc.h` no longer includes `stabilization_indi.h` (bug-039 was an old version);
  both MFC `.c` already include it unconditionally → de-guarding the WLS path is link-safe.
- The thrust_to_pprz consumer in `stabilization_mfc.c:779` stays compile-time because
  TRUE selects the stock-INDI downstream (MFC stabilizer not compiled), so a runtime
  cross-module extern would be fragile for no benefit.
- Toggles are **non-persistent** — a stray allocator/thrust choice is never saved to flash.

## Verification
A successful `.elf` is the only automated check (functional = flight test).
- `./pprz.sh build ANTON_MFC nps` ✅ (stab=mfc + guidance=mfc → stabilization_mfc.c + guidance_mfc.c)
- `./pprz.sh build ANTON_DUAL nps` ✅ (oneloop_mfc.c)
- `./pprz.sh build ANTON_MFC ap`  ✅ (flight build)
- `generated/settings.h` contains `stabilization_mfc_use_pseudo_inverse`,
  `guidance_mfc_thrust_to_pprz` (ANTON_MFC) and `oneloop_mfc_use_pseudo_inverse` (ANTON_DUAL).

## Next
- Flight-test the live `alloc_pseudo_inv` switch on an over-/exactly-actuated frame (WLS↔pseudo-inv).
- Consider giving the per-axis stab gains the same `module=` wiring consistency review.
