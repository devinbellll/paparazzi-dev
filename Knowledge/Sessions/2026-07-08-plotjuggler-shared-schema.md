# Session — 2026-07-08: One PlotJuggler schema for all feeds (sim + real, any aircraft)

## Problem

1. `./sim.sh ANTON_MFC` streamed **both** the ivy telemetry JSON and the NPS scope
   to PlotJuggler — every curve drawn twice, from two different clocks (walltime
   vs sim time). In SITL we want the scope (ground truth + full rate); in real
   flight only ivy exists.
2. Layouts hardcoded the aircraft root (`/Hoops_111_MFC (111)/…`, `/… (sim)/…`),
   so each aircraft × feed combination needed its own template (4 files).

## Solution — normalize at the relay layer

`pj_json_relay.py` gained `normalize_obj()` + a `normalize=` arg on `sanitize()`
(default **on**; `--raw` CLI flag to disable). Every packet is rewritten to a
shared schema before PlotJuggler sees it:

- root key (`"NAME (id)"` from server.ml, `"NAME (sim)"` from nps_scope) → fixed **`uav`**
- branch renames so MFC things sort together:
  `STAB_MFC` → `MFC_STAB`, `GUIDANCE_MFC` → `MFC_GUIDANCE`,
  `GUIDANCE_MFC_ACC2ATT` / `ACC2ATT` → `MFC_ACC2ATT`
- pprzlink `messages.xml` names are **unchanged** (renaming ids there would
  ripple through firmware/GCS/sdlog tooling); the map covers the ivy names.

## Feed selection in `sim_anton.py`

- NPS scope now emits to **local port 9871**; a new `scope_relay` thread
  normalizes and forwards to `PJ_HOST:PJ_PORT` (env-overridable; default
  `host.docker.internal:9870`). `sim.sh` passes `PJ_HOST`/`PJ_PORT` into the
  container.
- The ivy telemetry stream (local 9870) is still captured — normalized — to the
  `.jsonl` log, but forwarded to PlotJuggler **only with `--no-scope`** (the
  real-flight-like fallback). Never both.
- Real flight: run `pj_json_relay.py` standalone as before; it normalizes by
  default, so the same layouts apply.

## Other changes

- Firmware `NPS_SCOPE_VAR` strings renamed at the source to `MFC_STAB/…`,
  `MFC_GUIDANCE/…`, `MFC_ACC2ATT/…` (`stabilization_mfc.c`, `guidance_mfc.c`,
  `oneloop_mfc.c`). ANTON_MFC nps rebuilt clean.
- `plotjuggler_mfc.xml` rewritten to `/uav/...` curves (127 curves, 63
  sim/ivy duplicates collapsed). `plotjuggler_mfc_ivy.xml` and
  `plotjuggler_indi_ivy.xml` **deleted**.
- `plotjuggler_indi.xml` was a stale copy of the MFC layout — regenerated as a
  true INDI layout: SPvMEAS, INDI (`indi/roll_cmd…`, `indi/act0-3`, thrust-MFC
  `mfc/z_*`), WLS, States/Estim/Sensors. Note: `indi/*` branches are **sim-only**
  for now (no ivy parity for INDI internals yet).

## Verified live (in-container, PJ_HOST=127.0.0.1 PJ_PORT=9872)

- default run: 10 012 packets / 20 s (~500 Hz), root `uav` only, branches
  `MFC_STAB`, `MFC_GUIDANCE`, `MFC_ACC2ATT`, `WLS_U/V`, `TRUTH`, `EST`,
  `SENSORS`, `SP`, `MODE`, `guidance_v` — **no ivy duplicates**.
- `--no-scope`: only ivy messages (`ROTORCRAFT_FP`, `INS_EKF2`, `STAB_ATTITUDE`,
  …), all normalized under `uav`. `.jsonl` capture confirmed normalized.

## Next

- ANTON_MFC's ivy telemetry section doesn't currently send STAB_MFC /
  GUIDANCE_MFC (they never appeared in the `--no-scope` run) — add them to its
  telemetry XML if real-flight MFC plots are wanted on this airframe.
- Consider ivy parity for `indi/*` internals so the INDI tab works in real flight.
