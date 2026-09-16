# Plan — `sim_anton.py` Paparazzi-Native Rework

**Goal:** `sim_anton.py` works and already supports any airframe (the name is a leftover), but it is
a remnant of a hacky period: it hand-rolls things Paparazzi already provides. For **each feature**,
check whether a native Paparazzi mechanism covers it, and lean on the intended features —
flight plans, launching, `rc_script`, console/settings — instead of bespoke code. The name can stay.

Pairs with `Knowledge/Plans/MFC Flight-Test Enablement.md` — both share the NPS_SCOPE JSON schema
defined there (Phase 3). This document is item 2 of that request.

Date drafted: 2026-06-25. Source: `sim_anton.py` (current), `sim.sh`, `pprz_ctrl.py`.

> **STATUS 2026-07-22 — this plan is COMPLETE (Phases A–E), with two corrections.**
> Finished by the launcher cleanup (`Knowledge/Sessions/2026-07-22-launcher-cleanup.md`).
>
> **RETRACTED — row 1 (`pprzsim-launch`).** The recommendation to replace the manual
> `server`/`link`/`simsitl` `Popen` chain with `sw/simulator/pprzsim-launch` is **wrong**
> and was never actioned. That script is a thin `execv` wrapper exposing only
> fg/rc_script/norc/js_dev/spektrum/ivy_bus/time_factor/nodisplay — it cannot express the
> in-process scope emitter (`--scope_*`), cannot wrap the process in `gdbserver`, and has
> no passthrough for extra arguments. Invoking `simsitl` directly IS its native CLI, so
> the chain stays. `_lookup_ac_id` also stays: Ivy ground messages address the aircraft by
> numeric `ac_id`, which no launcher choice removes.
>
> **REVERSED — Phase E / the `--render` open question.** Previously "keep it, slimmed".
> `--render` has now been **deleted**: its whole JSBSim-truth panel was fed by
> `NPS_RATE_ATTITUDE` / `NPS_POS_LLH`, which arrive exactly **once per run** (measured: 1
> occurrence each across 22 150 packets), so every attitude/position/AGL figure it drew
> was a frozen startup value. PlotJuggler and the GCS do this properly. Plain stdout is
> also what the agent use-case wants.
>
> **Superseded by native GUI usage.** Interactive driving now belongs to the Paparazzi
> control panel (`conf/userconf/ENAC/control_panel_mfc.xml`). `pprz_ctrl.py` and the stdin
> `cmd_loop` are **deleted**, replaced by two flags: `--nav "Start Engine,Takeoff,+15,Nav"`
> (flight-plan blocks BY NAME, resolved from `var/aircrafts/<AC>/flight_plan.xml`, `+N`
> waits N seconds) and `--set NAME=VALUE` (repeatable `DL_SETTING` by shortname).
>
> **Phase C (logging) landed differently and better.** Rather than capturing ivy telemetry
> to `.jsonl`, the run capture is now the **same stream PlotJuggler is fed** (the scope by
> default), written as the canonical `/uav` wide **CSV** via `tools/scope2csv.py`. The old
> code forwarded the scope but logged ivy, so the log never contained the ground truth you
> were watching.

---

## Feature audit — `sim_anton.py` vs. native Paparazzi

| #   | `sim_anton.py` feature (current)                                                                                                            | Native Paparazzi equivalent                                                                                                                | Recommendation                                                                                                                                                                                                                                                                                                                          |     |
| --- | ------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --- |
| 1   | **Launch server + link + simsitl** by hand (`SERVER -b … -n`, `LINK -udp`, raw `simsitl`)                                                   | `sw/simulator/pprzsim-launch` (the canonical headless NPS launcher) + `sw/ground_segment/tmtc/server`; or `./paparazzi` session            | **Replace** manual `Popen` chain with `pprzsim-launch`. It already wires fdm/sensors/datalink correctly per-airframe.                                                                                                                                                                                                                   |     |
| 2   | **Send takeoff** via hand-encoded pprz `BLOCK` frames to UDP **4243** (`_pprz_frame`, checksum, `pprz_block_frame`)                         | Flight plan blocks + Ivy `BLOCK`/`JUMP_TO_BLOCK` messages routed through `server` (let `link` encode)                                      | **Replace** hand-encoded UDP frames with Ivy `BLOCK_TO`-style messages, or drive entirely from the **flight plan** auto-start. Stop bypassing the server.                                                                                                                                                                               |     |
| 3   | **Switch controller** by hand-encoded `SETTING` frame to a **hardcoded flat index** (`DUAL_CTRL_IDX=47`, with a "verify with grep" comment) | Ivy `DL_SETTING` by **name** via `settings` tooling / `sw/ground_segment/python/settings_tool`; `sdlogger_get_setting_id.py` for id lookup | **Replace** the brittle hardcoded index with name-based settings. (Dual-ctrl is `ANTON_DUAL` only — not needed for standalone `Hoops_111_MFC`.)                                                                                                                                                                                         |     |
| 4   | **RC stick scripts** `--rc_script N` (hover/step/ff)                                                                                        | Already a **native NPS feature** (`nps_radio_control.c` compiled scripts) — `sim_anton` just forwards the flag                             | **Keep as-is.** This is the one part already using the intended mechanism. Optionally also expose native **joystick** input (`sw/ground_segment/joystick`).                                                                                                                                                                             |     |
| 5   | **Live state TUI** (`--render` dashboard) + Ivy binds for `NPS_*`, `STAB_*`, `WLS_*`, `DUAL_CTRL`                                           | GCS, `messages.py` (real-time message viewer), PlotJuggler via NPS scope                                                                   | **Keep `--render`, slim it down.** It's a handy headless at-a-glance check. Keep the basic state block (attitude/rate, pos/alt, active law, command status, the few-line debug tail); drop the deep MFC/INDI per-axis tables (PlotJuggler + the scope already do that better). Trim the now-dead Ivy binds that fed the dropped panels. |     |
| 6   | **CSV logger** (`log_writer`, wide bespoke columns at 10 Hz)                                                                                | server `.log`/`.data`; **NPS_SCOPE JSON** (already streams, sim-time stamped)                                                              | **Replace** with the NPS_SCOPE JSON capture (shared schema with the SD log). Removes the bespoke CSV that `analyze_mfc.py` depends on (retargeted in the other plan).                                                                                                                                                                   |     |
| 7   | **Console / interactive commands** (`cmd_loop`: `block`/`switch`/`setting` from stdin; `pprz_ctrl.py` second-terminal helper)               | GCS strip buttons; `messages`/`settings` Ivy tools; flight-plan `exception`/auto-sequencing                                                | **Replace** with thin Ivy senders (name-based), or just use the GCS. Keep a tiny stdin shell for headless convenience, but built on Ivy not raw UDP.                                                                                                                                                                                    |     |
| 8   | **AC_ID lookup from conf XML** (`_lookup_ac_id`)                                                                                            | `pprzsim-launch -a AIRCRAFT` resolves this from the conf                                                                                   | **Drop** — let the launcher resolve the aircraft.                                                                                                                                                                                                                                                                                       |     |
| 9   | **FlightGear stream** (`--fg`)                                                                                                              | `pprzsim-launch --fg_host …` / native NPS FG flags                                                                                         | **Forward to launcher** instead of re-implementing flag passing.                                                                                                                                                                                                                                                                        |     |
| 10  | **PlotJuggler scope** (`--scope_*`)                                                                                                         | NPS scope is compiled into `simsitl`; flags are native                                                                                     | **Keep**, forward via launcher. Add MFC signals (other plan, Phase 3a).                                                                                                                                                                                                                                                                 |     |
| 11  | **gdbserver wrap** (`--gdb`)                                                                                                                | `pprzsim-launch` / direct; or VSCode sim debug task                                                                                        | **Keep** as a thin wrapper option.                                                                                                                                                                                                                                                                                                      |     |

**Summary (as drafted; see the 2026-07-22 status block for what actually happened):** the only
genuinely-native pieces today are `--rc_script`, the scope, and FG (all just flag-forwarding). The
launching, command delivery, settings-by-index, and CSV logging are bespoke and should move to
~~`pprzsim-launch`~~ (retracted — direct `simsitl` is native) + Ivy-by-name + the NPS scope. The
`--render` TUI ~~stays but gets slimmed~~ **was deleted**.

---

## Target architecture

Thin orchestrator, native underneath:

```
sim_anton.py AIRCRAFT [flags]
  ├─ launch:   pprzsim-launch -a AIRCRAFT [--fg_*] [--scope_*] [--rc_script N] [--gdb]
  │            (which itself brings up fdm/sensors/datalink + server/link as needed)
  ├─ commands: Ivy senders by NAME — BLOCK <name|id>, DL_SETTING <setting-name> <val>
  │            (no hand-rolled pprz frames, no UDP 4243, no hardcoded flat indices)
  ├─ autostart: prefer the flight plan to auto-start (Wait GPS→Holding→Start Engine→Takeoff);
  │            scripted timing only as an override
  ├─ logging:  capture the NPS_SCOPE JSON stream to a file (shared schema) — no bespoke CSV
  └─ view:     PlotJuggler (live) / messages tool; optional minimal stdin shell over Ivy
```

Keep the positional `AIRCRAFT` + existing flags (`--fg`, `--gdb`, `--rc_script`, `--scope*`) so the
CLI and `sim.sh`/VSCode tasks don't break; internally route them to the native tools.

---

## Migration phases (incremental, keep it runnable each step)

- **Phase A — launch via `pprzsim-launch`.** Replace the manual `server`/`link`/`simsitl` `Popen`
  chain. Verify the sim still flies and the scope still streams. Drop `_lookup_ac_id`.
- **Phase B — Ivy command delivery.** Replace `_pprz_frame`/`pprz_block_frame`/`pprz_setting_frame`
  and the UDP-4243 sends with Ivy `BLOCK`/`DL_SETTING` by name (reuse the `ivy` import already
  present). Delete the `DUAL_CTRL_IDX=47` hardcode. Fold `pprz_ctrl.py` into the same senders.
- **Phase C — logging.** Remove `log_writer`/CSV; add a small NPS_SCOPE JSON capture sink (or
  document PlotJuggler recording). Coordinate with `analyze_mfc.py` retarget (other plan, Phase 4).
- **Phase D — flight-plan-first start.** Prefer flight-plan auto-start over the scripted
  `takeoff_sequence`; keep `--switch-after`/scripted timing as an explicit override only.
- **Phase E — view.** Keep `--render` but slim it to the basic state block (attitude/rate, pos/alt,
  active law, last command, short debug tail); remove the deep MFC/INDI per-axis tables and the now-
  unused Ivy binds that fed them. Keep an optional minimal stdin Ivy shell. Update the module
  docstring (it still describes UDP 4243 / hardcoded indices / `/tmp` CSV).

Each phase leaves `sim.sh ANTON_MFC` / `Hoops_111_MFC` working.

---

## Touch points  *(all closed 2026-07-22)*
- [x] `sim_anton.py` — rewritten around Ivy-by-name (~~`pprzsim-launch`~~ retracted); frame
      encoder, bespoke CSV logger and hardcoded setting index deleted; `ac_id` lookup KEPT
      (Ivy ground messages need it); `--render` **deleted**, not slimmed; `--nav`/`--set` added
- [x] ~~`pprz_ctrl.py`~~ — **deleted**, replaced by `--nav` / `--set`
- [x] `sim.sh` — unchanged structurally (the `IS_SANDBOX` network fork is load-bearing);
      usage header updated for the new flags
- [x] Docstring/README — rewritten to the native model, and points at the GUI control panel
      for interactive work
- [x] Logging — coordinated with `analyze_mfc.py` via the canonical `/uav` wide CSV;
      ~~`tune_mfc.sh`~~ deleted

## Risks
- `pprzsim-launch` networking inside the ephemeral container (sandbox DinD vs Mac Docker) — `sim.sh`
  already forks on `IS_SANDBOX`; verify the launcher honours the same `--network`/Ivy bus settings.
- Ivy bus address must match (`127.255.255.255:2010` today) across launcher, senders, PlotJuggler.
- Don't regress the working `--rc_script` / scope / FG paths during the rewrite.

## Open questions
~~`--render` …~~ **Closed 2026-07-22: deleted** (see status block).
Start-up is now explicit and data-driven via `--nav` (default `"Start Engine,Takeoff"`), so the
"flight-plan auto-start vs script-driven" question is moot — the sequence is a CLI argument.

**Still open / found during the cleanup:** headless SITL in the sandbox does not actually take
off — `Hoops_111_MFC` sits at `TRUTH/agl` 0.098 m with `WLS_U` outputs near zero even with
`--rc_script 0`. Pre-existing (not caused by this rework) and unrelated to telemetry; needs its
own look.
