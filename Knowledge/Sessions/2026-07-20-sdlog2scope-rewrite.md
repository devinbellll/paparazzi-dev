# 2026-07-20 — sdlog2scope.py rewritten to the /uav CSV schema

## What changed

`tools/sdlog2scope.py` now produces **the same output as `tools/convert_sd_to_pj.py`**, but
straight from the raw `.log` + `.data` pair instead of a GCS-exported CSV.

Before: it emitted newline-delimited JSON in the old NPS_SCOPE key schema (`mfc/roll/sp`,
`mfc_g/x/meas`, `truth/*`), driven by a hardcoded field-order table that had already drifted
from `messages.xml` (missing the `sp_traj_*` fields).

After: it emits a wide CSV with `/uav/<BRANCH>/<field>` columns — identical contract to
`convert_sd_to_pj.py`, which it imports `BRANCH_MAP` from so the `STAB_MFC → MFC_STAB` /
`GUIDANCE_MFC → MFC_GUIDANCE` / `GUIDANCE_MFC_ACC2ATT → MFC_ACC2ATT` renaming has one home.

```bash
python3 tools/sdlog2scope.py FLIGHT.data                    # -> FLIGHT_pj.csv
python3 tools/sdlog2scope.py FLIGHT.data --ac 177
python3 tools/sdlog2scope.py FLIGHT.data -m STAB_MFC,GUIDANCE_MFC,WLS_U
python3 tools/sdlog2scope.py FLIGHT.data --trigger STAB_MFC
```

The `.log` is found automatically next to the `.data` (`--log` to override).

## How it works

1. Parse the `.log` XML `<protocol>` section → `{message: [field names]}`. Field names come
   from the flight's own log, so there is no message table to keep in sync with `messages.xml`.
   The `telemetry` msg_class wins on name clashes.
2. Pre-scan the `.data` for which messages actually occur; columns are built from those
   (or from `-m`).
3. Stream the `.data`, forward-filling a rolling row; emit one row per distinct timestamp
   (or per `--trigger` message). Values are written verbatim — raw units, arrays left
   comma-joined in one quoted cell, quotes stripped from string/enum fields.
4. Lead columns `Time, UTC, GPS_lat(deg), GPS_long(deg)` mirror the exporter: UTC is
   `floor(time_of_day) + t`, lat/lon are GPS_INT `lat`/`lon` × 1e-7.

## Verification

Converted `SUCCESSFUL_NAV_FLIGHTS/.../fr_0002/26_07_19__07_34_42_SD.data` and diffed against the
known-good `26_07_19__07_34_42_SD_pj.csv`:

- All 89 reference columns present (new output has 213 — it keeps every message in the log,
  not just the ones the manual export happened to include).
- **0 mismatches across 13,005 compared cells.** The reference is resampled onto a 4 Hz grid,
  so each reference row was compared to the nearest preceding full-rate row.
- Full rate: 18,039 rows vs the exporter's 153, over the same flight.
- All six flight `.data` files in `SUCCESSFUL_NAV_FLIGHTS/` and `SUCCESSFULL_FLIGHTS_SD/`
  convert cleanly, including the no-GPS one.

## Notes / next

- `ESC` is logged once per motor at the same timestamp, so its forward-filled columns show the
  last motor of each burst. Use `-m` to exclude it, or split by `motor_id` if it ever matters.
- The Data Management plan (`Knowledge/Plans/MFC Data Management & Comparison.md`) says
  sdlog2scope.py is retired in favour of `tools/mfcdata/`. That CLI still doesn't exist; this
  rewrite makes sdlog2scope.py the working SD→CSV path in the meantime, and its schema is
  already the canonical wide-CSV one the plan calls for.
