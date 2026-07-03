# 2026-07-03 — Yaw wrap fix committed + PlotJuggler JSON relay tool

## What happened

Two pieces of prior work were sitting uncommitted across both repos and got
split into semantically separate commits, plus a new tool was added:

1. **Yaw wrapping fix (paparazzi submodule, carried over from 07-02 SITL work)**
   `mfc_core` is axis-agnostic and does no angle wrapping, so the yaw axis was
   hitting the ±pi discontinuity of `stateGetNedToBodyEulers_f()->psi`
   directly — this is the "yaw has wrapping issue" flagged in `7eb437a70`.
   Fixed in `stabilization_mfc.c` by tracking a continuous unwrapped yaw
   (`mfc_psi_continuous`, accumulated per-cycle wrapped increment, rebased in
   `stabilization_mfc_enter()`), with the setpoint expressed in the same
   continuous frame. Retuned `YAW_ALPHA` (0.2→1) and `YAW_DERIVATIVE_GAIN`
   (1.5→2) on `hoops_111_mfc.xml` now that the measurement is continuous.

2. **PlotJuggler UDP JSON relay/sanitizer (new tool: `pj_json_relay.py`)**
   `server.ml`'s UDP JSON emitter (vendored pprzlink) has two bugs that
   produce invalid JSON: unescaped strings, and OCaml's `%f` printing bare
   `nan`/`inf`/`-inf` instead of valid JSON literals. Both crash PlotJuggler's
   UDP Server plugin, and are worse on real hardware (lossy RF corrupts raw
   bytes far more than sim loopback). Rather than patch vendored pprzlink,
   the relay sits between `server.ml` and PlotJuggler, repairs or drops
   malformed datagrams, and re-emits clean JSON — also replacing the `socat`
   hop previously needed to get packets from the VM interface to localhost.
   Wired into `control_panel_mfc.xml` as a new "Command line (CMD)" session
   program; `server.ml`'s own JSON stream now targets `127.0.0.1` and the
   relay forwards on to the real PlotJuggler address.

3. **Flight-test config cleanup** — deduped the `mfc` telemetry mode in
   `mfc_flight_test.xml` (was defined twice, second copy had stray
   0.01s-period duplicates), and switched `conf_mfc.xml`'s flight plan to
   `anton_mfc_attitude` for MFC attitude-only testing.

## Commits

**paparazzi submodule** (`flight-testing-hoops`):
- `e0b9c3962` fix: unwrap yaw measurement in MFC stabilization
- `f95c69286` feat: route MFC telemetry through pj_json_relay, update flight-test config

**paparazzi_dev** (`feat/shadow-handoff`):
- `2e4093b` fix: yaw wrapping fix in MFC stabilization (paparazzi submodule) — bumps submodule pointer, backfills daily notes 06-30 → 07-02
- `468e225` feat: add pj_json_relay.py UDP JSON sanitizer for PlotJuggler

## Non-obvious things worth remembering

- The commit split is deliberately along "wrapping fix" vs. "new tool" lines
  per explicit request — a couple of files (telemetry mode dedupe, flight
  plan swap) don't map cleanly to either bucket and were folded into the
  tool commit as general flight-test config housekeeping rather than forcing
  a third commit.
- `sw/ext/chibios` shows as a dirty submodule with untracked content in the
  `paparazzi` repo — pre-existing, unrelated to this session, left untouched.
- `.wolf/anatomy.md` / `.wolf/buglog.json` are OpenWolf auto-maintained index
  files; they got swept into the tool commit since that's what they were
  freshest for (new file + new memory entries), not because they're
  conceptually "tool" content.

## Next steps

- Tune XYZ nav control in SITL (still open on today's list).
- Flight-test the yaw-wrap fix on real hardware (only verified in SITL/desk
  review so far — daily note flags it as "stable-ish" territory).
