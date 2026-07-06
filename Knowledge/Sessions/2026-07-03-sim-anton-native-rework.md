# 2026-07-03 — sim_anton.py Paparazzi-native rework

Implemented `Knowledge/Plans/sim_anton Paparazzi-Native Rework.md`.

## What changed

- **Command delivery (Phase B, the real payoff):** deleted the hand-rolled pprz
  binary frame encoder (`_pprz_frame`/`pprz_block_frame`/`pprz_setting_frame`,
  STX/checksum, raw UDP:4243 sends) and the hardcoded `DUAL_CTRL_IDX=47`. Now
  uses `pprzlink.ivy.IvyMessagesInterface` + `PprzMessage("ground", ...)` to
  send native `JUMP_TO_BLOCK` / `DL_SETTING` messages — the same path the GCS
  strip buttons and settings panel use. `server` decodes them into the wire
  format; we never touch it.
- **Settings by name, not flat index:** `var/aircrafts/<AC>/settings.xml` is
  parsed with `sw/lib/python/settings.py`'s `PprzSettingsParser` so `setting
  <name> <value>` and `switch <indi|mfc>` resolve the index at runtime. `switch`
  checks for an `active_law` (dual_ctrl_active) setting first and prints a
  friendly message on aircraft that don't have one (i.e. every non-dual
  build — confirmed empty on ANTON_MFC/Hoops_111_MFC, present only on
  ANTON_DUAL) instead of silently sending garbage to a nonexistent index.
- **Telemetry capture replaces the bespoke CSV (Phase C):** `log_writer`
  (10 Hz hand-picked dict of fields written to a wide CSV) is gone. Instead,
  `server`'s own `-udp_json_stream_addr/-port` is pointed at localhost, and a
  new `telemetry_capture_relay()` thread tees every (sanitized, via the
  already-existing `pj_json_relay.sanitize()`) datagram to a `.jsonl` capture
  file *and* forwards it on to the Mac's PlotJuggler — same schema PlotJuggler
  already consumes, no bespoke columns to keep in sync with the firmware.
- **`--render` slimmed (Phase E):** dropped the deep STAB_MFC/STAB_ATTITUDE/
  WLS_V per-axis tables and their Ivy binds (on_stab_mfc, on_wls_v_stab,
  on_stab_attitude, on_sensors, on_gyro_bias, on_wind) — that level of detail
  lives in PlotJuggler/the scope now. Kept: JSBSim truth (attitude/rate/pos/
  alt), active law (DUAL_CTRL), motor commands (ROTORCRAFT_CMD), last command
  echo, short debug tail.
- `pprz_ctrl.py`: updated docstring + the `setting` subcommand to pass the
  name through as a string instead of `int()`-casting a flat index.

## What did NOT change (and why)

- **`pprzsim-launch` was evaluated, not adopted.** It only `execv`s simsitl
  with a handful of translated flags (fg/rc_script/norc/ivy_bus) — no support
  for the in-process scope emitter (`--scope_host/port/decim`) or a gdbserver
  wrap, both defaults for this workflow. Since it uses `optparse` with no
  passthrough for unknown long options, you can't sneak `--scope_host` past it
  either. Direct simsitl invocation (simsitl's own native CLI) stays.
- `_lookup_ac_id` (XML parse of `conf_enac.xml`) stays — it was never used for
  *launching* (that already just uses AC_NAME), only for the numeric `ac_id`
  Ivy messages need. Removing it wasn't actually on the table once you look at
  what it's really for.
- Server + link + simsitl are still launched as three separate subprocesses —
  this already mirrors the native `control_panel_mfc.xml` session structure
  (Data Link / Server / Simulator as separate programs), so there was nothing
  to "replace" here.
- The in-process NPS scope emitter (`--scope_host/port/decim` on simsitl
  itself) is untouched — it's a different, complementary data source (raw
  firmware-registered globals like `mfc/*`, `wls/*`) from the server's
  telemetry JSON stream, and both intentionally land on PlotJuggler's port
  9870 per the existing "NPS-scope ↔ ivy parity" design (see cerebrum).

## Verification

Ran the rewritten script for real via `./sim.sh ANTON_MFC --no-build
--no-scope` (ANTON_MFC already had a built `nps/simsitl`). Confirmed:
- server/link/simsitl all start with no Python traceback.
- Interactive commands (`block 4`, `switch mfc`, `setting active_law 1.0`)
  round-trip through the new Ivy-based senders with no exceptions; `switch
  mfc` on ANTON_MFC correctly declined with "not a dual-controller build"
  (ANTON_MFC has no `active_law`/`dual_ctrl_active` setting — confirmed via
  grep on its `settings.xml`).
- The `.jsonl` telemetry capture file stayed empty in this ~30s headless
  sandbox smoke test — consistent with the **pre-existing, already-documented
  sandbox limitation** (see cerebrum Do-Not-Repeat, 2026-06-25: "the headless
  CSV/debug logs from sim_anton.py read all-zero/frozen-t in this sandbox").
  Not a regression from this rewrite; full behavioral verification (does
  telemetry actually flow, does the takeoff sequence fly) needs a real run
  from the user's own machine, same as before.
- Caught and cleaned up two leftover `paparazzi-build` containers from
  `timeout -s KILL` runs — `timeout` only kills the `docker run` client, not
  the detached container (host networking mode), which caused a real
  `OSError: Address already in use` on the capture socket in one test. Worth
  remembering for future headless smoke tests in this repo.

## Next / open questions (unchanged from the plan)

- Whether `Hoops_111_MFC` should auto-start from the flight plan or stay
  script-driven (Phase D) — the takeoff sequence still hardcodes blocks 2/3;
  didn't change this, just re-encoded the same block IDs over Ivy.
- Joystick vs `--rc_script` preference, still open.
