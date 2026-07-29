# Session 2026-07-22 — Launcher / sim-stack cleanup

Big strip-down of the command-line launching stack, now that real work happens in the
Paparazzi GUI (Linux VM, `conf/userconf/ENAC/control_panel_mfc.xml`). This repo's job is
narrowed to two things: **programmatic build-run-observe for coding agents**, and a **fast
SITL "does my controller behave" loop**.

Plan: `/home/agent/.claude/plans/create-a-plan-for-serialized-yao.md`. Both pre-existing
plans were audited and corrected in place (they had drifted).

## Deleted

| File | Reason |
|---|---|
| `pprz_ctrl.py` | Never used. Wrote to `/proc/1/fd/0` of a guessed container to reach a stdin shell that no longer exists. |
| `tune_mfc.sh` | Never used. Default loop grepped for a CSV nothing writes. |
| `tools/convert_sd_to_pj.py` | Superseded by `sdlog2scope.py` (raw `.data`, full rate, no manual GCS export). |
| `sim_anton.py --render` TUI | Its data sources update once per run — see bug 1 below. |
| `sim_anton.py` stdin `cmd_loop` | Replaced by `--nav` / `--set`. |

`BRANCH_MAP` now lives **only** in `pj_json_relay.py`; `sdlog2scope.py` imports it.

## Three bugs found (all logged in `.wolf/buglog.json`)

1. **`--render` showed frozen values.** It bound `NPS_RATE_ATTITUDE` / `NPS_POS_LLH`, which
   arrive **once per run** — measured 1 occurrence each across 22 150 packets in
   `sim_logs/mfc_sim_20260708_110052.jsonl`. Every attitude/position/AGL number it drew was
   a startup value. Deleted rather than slimmed (reverses the old plan's Phase E decision).
2. **The sim logged the wrong feed.** PlotJuggler was fed the NPS **scope** (port 9871) while
   the `.jsonl` capture teed the **ivy** stream (port 9870) — so the log never contained the
   ground truth being watched. Confirmed: a 32 477-line capture with no `TRUTH`/`EST`/`SENSORS`.
   Both consumers now read one stream.
3. **Units mismatch made MFC curves look flat.** `nps_scope.c` emitted `TRUTH/*` angles in
   degrees and `nps_scope_state.c` did `DegOfRad()` on `EST/*` and `SP/stab/*`, while
   `MFC_STAB/*` publishes raw **radians**. The `SPvMEAS` tab overlays them on shared axes, so
   controller curves sat flat next to a truth curve 57× larger. Both files now emit radians.

## Changed

- **`sim_anton.py`** — `--nav "Start Engine,Takeoff,+15,Nav"` runs flight-plan blocks **by
  name**, resolved from the generated `var/aircrafts/<AC>/flight_plan.xml` (`<block name=…
  no=…>`); `+N` waits N seconds; an unknown name lists what exists and exits. `--set
  NAME=VALUE` (repeatable) sends `DL_SETTING` by shortname — the "same flight, different
  gains" knob. Run capture is now `sim_logs/mfc_sim_<TS>.csv`.
- **NEW `tools/scope2csv.py`** — scope packets → canonical `/uav` wide CSV, the same contract
  `sdlog2scope.py` produces for flights. Live sink + standalone `.jsonl` converter.
- **`pj_json_relay.py`** — `FIELD_ALIAS` projects `ROTORCRAFT_FP` → `EST/*` so the state tabs
  fill on the **flight** feed, not just in sim.
- **`analyze_mfc.py`** — reads the canonical CSV (legacy key-translation deleted); all metric
  math kept. Missing columns degrade to `n/a`; an identically-zero channel now reports
  **"no signal"** instead of a green `GOOD ✓`.
- **PlotJuggler layouts** — `Sensors` tab dropped from both; tabs renamed to state their
  coverage honestly (`INDI (sim only)`, `States (sim truth)`); header comments rewritten.

## Gotchas worth remembering

- **The ivy feed is one message per datagram**, the scope feed is all-variables-per-datagram.
  My first `scope2csv` took the header from packet 1 and silently truncated the ivy feed to
  **11 columns instead of 162**. Fixed with a warmup window (live) / two-pass (offline) plus
  forward-fill. Never assume one packet reveals the schema.
- **`ROTORCRAFT_FP` is raw fixed-point on the wire** — server.ml emits the stored int32, NOT
  the `alt_unit_coef` scaling in `messages.xml`. Scales: pos 1/2⁸ m, vel 1/2¹⁹ m/s, angles
  1/2¹² rad. It is also **ENU** while `EST/*` is **NED**, so the vertical axis is *negated*,
  not just scaled. Verified: `up=252` → `EST/z = −0.984 m`.
- **The two converters name the time column differently on purpose** — `sdlog2scope.py` keeps
  `Time` so its output stays diffable against the GCS CSV export; `scope2csv.py` uses `time`.
  `analyze_mfc.py` accepts either.
- `IvyMessagesInterface(start_ivy=True)` starts the bus in `__init__`, so dropping all
  `subscribe()` calls does **not** break sending. Checked before removing them.

## Verified

- `./pprz.sh build Hoops_111_MFC nps` → `.elf` produced (the only automated check on the
  firmware units change).
- `./sim.sh Hoops_111_MFC --no-build --nav "Start Engine,Takeoff,+12,Standby"` → names
  resolved to blocks 2/3/4 with the 12 s gap; **33 388-row CSV** containing `TRUTH`(26),
  `EST`(15), `SP`(29), `SENSORS`, `MODE`, `MFC_*`, `WLS_*` — i.e. the scope feed the old
  capture never had.
- Units: `EST/phi` and `MFC_STAB/me_phi` now have **identical ranges** (−0.0054…+0.0087 rad).
- Real flight `fr_0001` → `sdlog2scope.py` → 75 116 rows/213 cols → `analyze_mfc.py` runs
  clean, reporting `n/a` for absent `TRUTH/*` and `no signal` for its all-zero `MFC_STAB`
  (MFC was not the active law on that flight).
- Flight-feed alias verified offline: converting an ivy capture yields a populated `EST/*`.

## Known-unresolved (pre-existing, NOT from this cleanup)

**Headless SITL does not take off.** `TRUTH/agl` stays at 0.098 m and `WLS_U` outputs stay
near zero (vs ~1918 in the working 2026-07-16 run), even with `--rc_script 0` which
auto-takes-off independently of the flight plan. `MODE/ap` = 13 (`AP_MODE_NAV`), GPS has a
valid fix, `MODE/nav_v` = 0 and `SP/guidance/v_z` ≈ 0 — no climb is ever commanded, so
throttle stays idle. Ruled out as a regression: my changes touch only telemetry emission,
logging, and command *sending*; the pre-change `sim_anton.py` produces an **empty** capture in
this environment (matching the two 0-byte `.jsonl` files already in `sim_logs/` from before
the session), and `IvyMessagesInterface` auto-starts its bus so removing the subscribes cannot
affect delivery. Needs its own investigation — likely a nav/throttle-resurrect config issue in
the headless sandbox.

## Next

- Chase the no-takeoff issue above (blocks the "quickly see how my thing behaves" use case).
- The `tools/mfcdata/` package + `flight_data/` run store + compare/report remain open in
  `Knowledge/Plans/MFC Data Management & Comparison.md` (Phases B-partial, C, D). The
  canonical-CSV precondition they were blocked on is now satisfied by both paths.
