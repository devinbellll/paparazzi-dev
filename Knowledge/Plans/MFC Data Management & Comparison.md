# Plan — MFC Data Management & Comparison (flight ↔ sim ↔ Simulink)

**Goal:** one pipeline that turns every MFC data source — **real-flight SD logs / Ivy
telemetry**, **NPS sim captures**, and **Simulink model logs** — into a single canonical
per-run format, so runs are directly comparable and produce report-quality figures.

Date drafted: 2026-07-16. Supersedes Phase 3/4 of
`Knowledge/Plans/MFC Flight-Test Enablement.md` (flights flown 2026-07-05 and 2026-07-10;
SD fast-logs captured under `SUCCESSFULL_FLIGHTS_SD/`; the shared PlotJuggler schema is live).

---

## Current-state audit (what exists, what's broken)

**The shared schema exists — but only on the live path.** `pj_json_relay.py`
(`normalize_obj()`, `BRANCH_MAP` at line 79) rewrites both the Ivy/server.ml feed and the
NPS-scope feed into one tree rooted at `uav`:

| pprzlink message | canonical branch |
|---|---|
| `STAB_MFC` | `MFC_STAB` |
| `GUIDANCE_MFC` | `MFC_GUIDANCE` |
| `GUIDANCE_MFC_ACC2ATT` / `ACC2ATT` | `MFC_ACC2ATT` |
| everything else (`WLS_U/V`, `ROTORCRAFT_FP`, `TRUTH`, `EST`, `SENSORS`, `SP`, …) | pass-through unchanged |

`plotjuggler_mfc.xml` (127 curves) is written entirely against `/uav/...` keys.
`sim_anton.py` captures the normalized Ivy stream to `sim_logs/mfc_sim_<TS>.jsonl` every run.

**The offline path is out of sync:**

- `tools/sdlog2scope.py` + `analyze_mfc.py` still speak the retired lowercase schema
  (`mfc/roll/sp`, `truth/agl`) — nothing bridges them to `/uav/MFC_STAB/...`. The
  `sdlog2scope.py` field maps are also stale vs. current `messages.xml` (missing
  `sp_traj_*` on GUIDANCE_MFC, all of `MFC_ACC2ATT`).
- `convert_sd_to_pj.py` — the script that produced the `*_pj.csv` files in
  `SUCCESSFULL_FLIGHTS_SD/.../fr_0003` and `fr_0004` (SD tab-CSV → `/uav`-schema comma
  CSV) — lives only in an ephemeral sandbox job dir. **Not committed. Must be recreated.**
- The `.tlm → sd2log → .log/.data → tab-CSV` export step is manual and undocumented.
  `fr_0001` (the 2026-07-10 flight, the best dataset) and `fr_0002` were never converted
  past `.data`.
- `tune_mfc.sh`'s default sim path still greps for `sim_logs/mfc_sim_*.csv`, which
  `sim_anton.py` no longer writes (it writes `/uav`-schema `.jsonl`) — the default loop
  is dead.
- **Simulink: zero tooling.** The model lives outside this repo (see Daily Notes
  2026-07-01/03/07/09 and `Sessions/2026-07-07-pprz-wls-matlab-port.md`); no log format,
  exporter, or converter exists.

**SD CSV shape** (fr_0004, tab-separated, `MSG:field` headers): `Time`, `UTC`,
`GPS_lat(deg)`, `GPS_long(deg)`, then `GUIDANCE_MFC_ACC2ATT:*` (10 fields),
`GUIDANCE_MFC:*` (18 fields incl. `sp_traj_*`), `STAB_MFC:*` (22 fields),
`ROTORCRAFT_FP:*` (15 fields). All covered by the branch map above (ROTORCRAFT_FP and
GPS pass through under their own names).

---

## Design decisions (settled 2026-07-16)

1. **Canonical schema = the PJ schema.** `/uav/<BRANCH>/<field>` is the single key
   contract for everything: live streams, on-disk runs, analysis, reports. The lowercase
   `mfc/roll/...` schema is retired; `sdlog2scope.py` is superseded, not fixed.
2. **Canonical on-disk run format = wide CSV.** One time-indexed comma-CSV per run,
   `time` column (seconds, run-relative) + canonical `/uav/...` column names — the same
   shape as the existing `*_pj.csv`. Loads directly into PlotJuggler *and* pandas,
   human-inspectable.
3. **Simulink export = MATLAB `.mat` Dataset** (logsout / Simulation Data Inspector
   export), read on the Python side.
4. **Report output = matplotlib figures** (overlays of flight vs sim vs Simulink per
   signal group, PNG/PDF), publication-ready.

## The run store

```
flight_data/
├── runs/
│   └── <run_id>/                  # e.g. 2026-07-10_hoops111_flight_fr0001
│       ├── run.csv                # canonical wide CSV (the only file tools read)
│       └── meta.yaml              # provenance
└── reports/
    └── <report_name>/             # figures from `mfcdata report`
```

`meta.yaml` fields: `source` (flight | sim | simulink), `date`, `aircraft`, `ac_id`,
`firmware_commit`, `description`, `original_files` (paths to `.tlm`/`.jsonl`/`.mat`),
`t0_event` (how run-relative time zero was chosen), free-form `notes`.

Naming convention for `run_id`: `<date>_<aircraft>_<source>[_<tag>]`.

## The tool: `tools/mfcdata/` (python package, single CLI)

```
mfcdata sd       <file.data|file.csv>  -> flight_data/runs/<id>/   # SD flight ingest
mfcdata sim      <capture.jsonl>       -> flight_data/runs/<id>/   # NPS sim ingest
mfcdata simulink <run.mat> [--map map.yaml] -> flight_data/runs/<id>/
mfcdata compare  <runA> <runB> [...]   # aligned overlay plots, interactive/quick-look
mfcdata report   <runs...> --name X    # report figure sets -> flight_data/reports/X/
```

Shared internals:
- **One message→key map**, generated from (or checked against)
  `sw/ext/pprzlink/message_definitions/v1.0/messages.xml` + `BRANCH_MAP` imported from
  `pj_json_relay.py` — a single source of truth so live and offline paths can never
  diverge again. Fixes the stale-field problem structurally.
- **Resampler**: sources arrive at mixed rates (SD FlightRecorder periods, sim scope
  decimation, Simulink solver steps). Each converter forward-fills onto the
  highest-rate trigger (as sdlog2scope did) or `--grid HZ` for a uniform grid.
- **Time alignment** (compare/report): run-relative `t0` anchored on an event —
  default auto-detect (first throttle-up / first `sp_traj` step), overridable with
  `--t0 SEC` per run; recorded in `meta.yaml`.

### Converter specifics

**`mfcdata sd`** — accepts either the sd2log `.data` text log (preferred: full rate,
no manual export step) or the hand-exported tab-CSV. Recreates the committed version of
the lost `convert_sd_to_pj.py` header mapping (`MSG:field` → `/uav/<branch>/<field>`).
Document the full chain in the tool README: `.LOG (card) → sd2log → .log/.data →
mfcdata sd`. Backfill task: convert fr_0001–fr_0004.

**`mfcdata sim`** — the `.jsonl` is already `/uav`-normalized nested JSON; converter
flattens `{"uav": {"MFC_STAB": {"sp_phi": v}}}` → column `/uav/MFC_STAB/sp_phi` and
writes the wide CSV. Also accept an NPS-scope capture file (same shape post-relay).

**`mfcdata simulink`** — reads a `.mat` Dataset via `scipy.io.loadmat` /`mat73` for
v7.3. Signal naming convention **in the model**: name logged signals with the canonical
key using dots (`uav.MFC_STAB.sp_phi`, since `/` is illegal in Simulink signal names);
converter translates dots → slashes. For existing models that can't be renamed, a
`map.yaml` (`<simulink signal name>: /uav/...` pairs) does the translation; ship a
template with the MFC signal set. Model states (true attitude/position in the plant
block) map to `/uav/TRUTH/*` — the Simulink analogue of NPS truth.

### Comparison & report

**`mfcdata compare`** — quick-look overlay: given N runs and a signal (or group),
aligned time-series plot in a matplotlib window; `--save` for PNGs. Handles missing
columns gracefully (a Simulink run won't have `SENSORS/*`; a flight has no `TRUTH/*`
unless OptiTrack — plot what intersects, list what's missing).

**`mfcdata report`** — predefined figure sets, consistent styling (fixed colors per
source: flight/sim/simulink), saved PNG+PDF:
1. *Attitude tracking* — per axis: `sp`, `sp_traj`, `me` overlay + `err` subplot.
2. *Disturbance estimates* — `fk_{phi,theta,psi}` and `MFC_GUIDANCE fk_{x,y,z}`.
3. *Commands & allocation* — `cmd_*`, `WLS_V`, `WLS_U`, `u0..u3` (+ saturation shading).
4. *Guidance* — position `sp/sp_traj/me/err` per axis, `MFC_ACC2ATT` internals.
5. *Summary table* — the `analyze_mfc.py` metrics per run, rendered to the figure set
   (RMS err, max err, F_k range, saturation %, per axis).

### Retargeting existing tools

- `analyze_mfc.py` — replace `_scope_row_to_csv()`'s lowercase-key map with a
  canonical-CSV reader (`run.csv` → the same internal dataframe); keep all metric math.
  Becomes the metrics engine `mfcdata report` imports.
- `tune_mfc.sh` — default loop: run timed sim → `mfcdata sim` on the fresh `.jsonl` →
  `analyze_mfc.py` on `run.csv`. `--flight` flag now takes a `.data` file through
  `mfcdata sd`. Delete the dead `sim_logs/mfc_sim_*.csv` grep.
- `tools/sdlog2scope.py` — retire (superseded by `mfcdata sd`); note the supersession
  in its docstring or delete it.

---

## Phasing

**Phase A — canonical run store (unblocks everything)**
1. Create `tools/mfcdata/` skeleton + the messages.xml-derived key map (import
   `BRANCH_MAP` from `pj_json_relay.py`).
2. `mfcdata sd`: `.data` and tab-CSV ingest → `run.csv` + `meta.yaml`.
3. Backfill: convert fr_0001 (07-10 flight), fr_0002, fr_0003, fr_0004 into
   `flight_data/runs/`. Verify each `run.csv` loads in PlotJuggler against
   `plotjuggler_mfc.xml`.

**Phase B — sim parity**
4. `mfcdata sim`: `.jsonl` → `run.csv`.
5. Fix `tune_mfc.sh` default loop; retarget `analyze_mfc.py` onto `run.csv`.
6. Gate: one sim run of `Hoops_111_MFC nps` graded end-to-end through the new path.

**Phase C — Simulink ingest**
7. Define the dot-naming convention + `map.yaml` template for the current model's
   signal names; document required signal list (mirror of `MFC_STAB`/`MFC_GUIDANCE`
   fields + model states → `TRUTH`).
8. `mfcdata simulink`: `.mat` Dataset → `run.csv`. Gate: first Simulink run loads in
   PlotJuggler next to a flight run.

**Phase D — compare & report**
9. Time-alignment (auto-anchor + `--t0`), `mfcdata compare`.
10. `mfcdata report` figure sets 1–5.
11. **Deliverable: the first three-source report** — the 2026-07-10 flight (fr_0001)
    vs. an NPS run vs. a Simulink run of the same maneuver, into
    `flight_data/reports/2026-07-10_hoops111/`.

## Touch points (checklist)

- [ ] NEW `tools/mfcdata/` package: `keymap.py` (messages.xml + BRANCH_MAP), `sd.py`,
      `sim.py`, `simulink.py`, `align.py`, `compare.py`, `report.py`, `cli.py`, README
- [ ] NEW `flight_data/` run store + `meta.yaml` convention
- [ ] Backfill runs from `SUCCESSFULL_FLIGHTS_SD/FLIGHT_RECORDER/extracted/fr_0001..4`
- [ ] `analyze_mfc.py` — canonical-CSV reader, keep metrics
- [ ] `tune_mfc.sh` — new default loop via `mfcdata sim`; fix dead CSV grep
- [ ] `tools/sdlog2scope.py` — retire/mark superseded
- [ ] Simulink model (outside repo): adopt dot-naming or fill `map.yaml`; export `.mat`
- [ ] `plotjuggler_mfc.xml` — no change expected; used as the load-check for every converter

## Risks & open questions

- **Simulink signal inventory unknown** — which MFC-internal signals the model actually
  logs (F_k? WLS internals?) determines how much of the report set it can populate.
  → Get the model's logged-signal list before Phase C; the converter must tolerate
  partial coverage anyway.
- **.mat time-base** — Dataset timeseries carry their own time vectors per signal;
  converter must merge onto one grid (same forward-fill machinery as SD).
- **Units** — SD/ivy fields are floats in message units (rad, pprz cmd counts); the
  Simulink model may log SI or normalized values. The `map.yaml` gets an optional
  per-signal `scale:` to absorb this. Sim/flight sides are unit-identical by
  construction (same firmware code).
- **`TRUTH` on real flights** — only exists via OptiTrack (`EST` otherwise); compare
  tooling must not assume it.
- Carried-over gaps (out of scope here, tracked): ANTON_MFC Ivy telemetry section
  doesn't send STAB_MFC/GUIDANCE_MFC (Sessions/2026-07-08); NPS_RATE_ATTITUDE /
  NPS_POS_LLH staleness in sim Ivy telemetry (Sessions/2026-07-16-mfc-conf-cleanup).
