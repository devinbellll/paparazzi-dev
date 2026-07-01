# Session — 2026-07-01: Flight-Test Streaming Fix + kd/use_Kd Tuneability

## What changed

**1. Committed the working real-flight-test PlotJuggler streaming fix** (already present as uncommitted changes at session start, verified working against real Hoops_111_MFC flight-test hardware):

- `sw/ground_segment/tmtc/server.ml`: `udp_sockaddr` was bound at module-load
  time (before `Arg.parse`), so `-udp_json_stream_addr` was silently a no-op.
  Made it a `ref`, re-resolved after arg parsing.
- `sw/airborne/firmwares/rotorcraft/stabilization/mfc_core.c`: the feedforward
  acceleration term was zeroed whenever `use_trajec_sp=false`, killing
  feedforward on every guidance axis (they all set `use_trajec_sp=0`). Fixed
  to always derive it from `setpoint_trajec` history.
- `stabilization_mfc.c`: roll/pitch kp/kd flight-tuned (kp 6/8, kd 1.5/5,
  use_Kd=true).
- Telemetry (`mfc_flight_test.xml`), control panel IPs, `conf_enac.xml`
  settings_modules, and `sim_anton.py`/`plotjuggler_indi_ivy.xml` wired to match.
- Committed as 3 commits: submodule fix, outer-repo wiring, OpenWolf bookkeeping.

**2. Added `kd` and `use_Kd` as tuneable parameters on every MFC axis**, per
user request, following the exact pattern established 2026-06-30 for
kp/alpha/traj (compile-time `#ifndef` default → airframe XML `<define>` →
runtime `dl_setting`):

- `stabilization_mfc.{c,xml}`: roll/pitch/yaw. Also restored `kp` to read
  from macros again (it had been hardcoded to literals during flight tuning);
  bumped the module's `PITCH_PROPORTIONAL_GAIN` default 6→8 to match, so
  compiled behavior is unchanged.
- `guidance_mfc.{c,xml}`: gx/gy/gz. `kd` was hardcoded to `0.f` with no
  `use_Kd` at all; now both are macro-driven (default kd=0/USE_KD=FALSE,
  unchanged behavior).
- `oneloop_mfc.{c,xml}`: roll/pitch/yaw + gx/gy/gz, same treatment.
- Deliberately left `guidance_indi.c`'s deprecated `mfc_thrust`
  (`GUIDANCE_INDI_THRUST_MFC`) alone — its own comment says it's hardcoded on
  the way out, superseded by `guidance_mfc`.

## Verification

Build-verified (compile + link only, no flight/SITL behavior check this
session): `ANTON_MFC ap`, `ANTON_MFC nps`, `Hoops_111_MFC ap`, `ANTON_DUAL
nps`. Confirmed `mfc_roll.use_Kd` etc. get distinct dl_setting case indices in
generated `settings.h` (not colliding with the existing `kd`/`kp` cases).

## What's next

- Flight-test the kd/use_Kd panel live (GCS dropdown) to confirm the new
  settings actually reach the airframe and behave as expected — this session
  only verified compile/link, not runtime.
- Consider whether `kp` being macro-driven-again (vs. the flight-tuned
  literals) should be re-validated against the same flight-test conditions
  that produced the 6/8, 1.5/5 numbers — values are unchanged, but worth a
  sanity flight before trusting it fully.
