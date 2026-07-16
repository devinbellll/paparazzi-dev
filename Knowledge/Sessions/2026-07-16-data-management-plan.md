# Session 2026-07-16 — MFC Data Management & Comparison plan

## What changed
- **New plan:** `Knowledge/Plans/MFC Data Management & Comparison.md` — post-flight-test
  data pipeline: three sources (real-flight SD/ivy, NPS sim `.jsonl`, Simulink `.mat`
  Dataset) → one canonical **wide CSV** per run (`/uav/...` columns, PlotJuggler- and
  pandas-loadable) in `flight_data/runs/<id>/` with `meta.yaml` provenance; new
  `tools/mfcdata/` CLI (`sd`/`sim`/`simulink`/`compare`/`report`) with matplotlib
  report figure sets. Phases A–D; final deliverable = three-source report of the
  2026-07-10 flight (fr_0001).
- **Old plan marked:** `MFC Flight-Test Enablement.md` got a STATUS header — phases 0–2
  done, Phase 3b/4 superseded by the new plan.

## Decisions (user-confirmed)
- Simulink logging exports as **MATLAB .mat Dataset**; model signals to be named with
  dot convention `uav.MFC_STAB.sp_phi` (or `map.yaml` translation).
- Canonical run format: **wide CSV** (not ndjson/parquet).
- Report output: **matplotlib PNG/PDF figures** (PJ layouts stay for interactive use).

## Audit findings driving the plan
- Live path (`pj_json_relay.py` BRANCH_MAP → `/uav/MFC_STAB|MFC_GUIDANCE|MFC_ACC2ATT`)
  and offline path (`tools/sdlog2scope.py` + `analyze_mfc.py`, lowercase `mfc/roll/...`)
  use **two divergent schemas**; sdlog2scope field maps are stale (no `sp_traj_*`,
  no MFC_ACC2ATT).
- `convert_sd_to_pj.py` (made the fr_0003/fr_0004 `*_pj.csv`) was never committed —
  lived in an ephemeral job tmp dir. Must be recreated as `mfcdata sd`.
- `tune_mfc.sh` default loop is dead: still greps `sim_logs/mfc_sim_*.csv`, but
  sim_anton writes `/uav`-schema `.jsonl` now.
- fr_0001 (07-10 flight) and fr_0002 never converted past `.data`.
- Zero Simulink tooling in-repo; model external (see 07-07 WLS MATLAB-port session).

## Next
- Execute Phase A: `tools/mfcdata/` skeleton, keymap from messages.xml + imported
  BRANCH_MAP, `mfcdata sd`, backfill fr_0001–fr_0004.
- Before Phase C: get the Simulink model's logged-signal inventory.
