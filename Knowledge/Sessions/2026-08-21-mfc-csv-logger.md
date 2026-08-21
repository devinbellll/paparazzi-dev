# 2026-08-21 — On-board CSV logging + manual→NAV handover (Hoops_111_MFC)

## 1. ANTON_HEOL gains committed

`a6051d54` (submodule) — the working-tree retune, recorded as behaving similarly
to the Simulink reference:

| axis | int_window | alpha | kd |
|------|-----------|-------|-----|
| roll  | 20 → 50 | 1/Ixx → 2/Ixx | 2.8 → 12 |
| pitch | 20 → 50 | 1/Iyy → 2/Iyy | 2.8 → 12 |
| yaw   | 20 → 50 | 1/Izz → 3/Izz | 2.8 → 6 |

`kp` (4.0) and `ki` (0) unchanged. The diff was gains only — nothing diagnostic
was left in, so this is a restored-configuration record, not a diagnostic build.

## 2. New module: `logger_mfc_csv`

**The problem it removes.** `flight_recorder` writes pprzlog binary, so every
flight needed `.tlm → sd2log → .log/.data → sdlog2scope.py` before anything
could be plotted. This module writes the canonical wide CSV *on board*, so the
card comes out of the aircraft ready for PlotJuggler's CSV importer and pandas.

**The column contract is the existing one, not a new one.** All 141 column names
are the canonical `/uav/<BRANCH>/<field>` keys that `pj_json_relay.py` already
produces from the live ivy / NPS-scope feeds, so `plotjuggler_mfc.xml` (127
curves) and `tools/mfcdata` work on the file unchanged.

| branch | cols | contents |
|--------|-----:|----------|
| `MFC_STAB` | 38 | sp / sp_traj / me / err / fk / cmd per axis, u0..u3, **plus presat, dterm, extder, interr, alpha, thrust_filt_z — none of which are on the wire** |
| `MFC_GUIDANCE` | 30 | same shape for gx/gy/gz |
| `MFC_ACC2ATT` | 10 | acc→attitude internals |
| `EST` | 15 | pos/vel/acc NED, eulers, body rates |
| `SP/stab`,`SP/guidance`,`SP/nav` | 24 | what each layer was asked for |
| `WLS_V`,`WLS_U`,`ACT` | 13 | pseudo-control, allocated u, actuator counts |
| `MODE` | 7 | ap/h/v/stab/arming/motors_on/in_flight |
| `LOG` | 3 | rows / drops / nan — the file describes its own health |

**Design points worth remembering**

- **Header and row come from one X-macro list**, so a column name can never
  drift off the value under it.
- **Own float formatter, not printf.** The ChibiOS sdlog `%f` has no NaN/Inf
  case and would write unparseable text mid-row, breaking every later column.
  Non-finite → `0`, counted in `/uav/LOG/nan`. Trailing zeros trimmed.
- **Chunked writer.** `SDLOG_MAX_MESSAGE_LEN` is 300 on H7 and a row is ~900 B,
  so a row is written as ~5 ordered chunks of ≤224 B.
- **Drop resync.** If the log queue refuses a chunk, the row is short — so a
  lone newline is re-emitted, otherwise one dropped chunk would splice into the
  next row and misalign every column after it. `/uav/LOG/drops` must read 0 for
  a clean file.
- **Third SD file.** `SDLOG_NUM_FILES` was hard-coded to 2 in
  `arch/chibios/mcuconf_h7.h`; the four SDLOG defines there are now `#ifndef`
  guarded so an airframe/module `-D` can raise it. The module sets 3 on `ap`
  (pprzlog + flight recorder + this), costing 16 KB of buffer.
- **sdLog always names files `.LOG`** — the file lands as `MFC/mfc_0001.LOG` and
  must be renamed to `.csv` for PlotJuggler's importer to offer it.

## 3. Verified

- `./pprz.sh rebuild Hoops_111_MFC ap` and `nps` — both produce an ELF.
  Symbols present in `ap.elf`; `ap.CFLAGS += -DSDLOG_NUM_FILES=3` generated;
  the four GCS settings generated into `settings.xml`.
- **Formatter host-tested** over 18 float cases incl. NaN/±Inf, and a 130-column
  row through the chunked writer (3 rows → 15 chunks, 130 cols each).
- **SITL run, 45 s** (`--nav "Start Engine,Takeoff,+15,Standby"`):
  **4481 rows, 141 columns, dt exactly 10.00 ms (100 Hz), `drops` 0, `nan` 0.**
  Only the final row is ragged — truncated mid-write by the SIGKILL that ends
  every sim. Attitude, estimator `fk`, guidance, WLS and actuator branches all
  live; the 38 constant columns are explained by a stationary hover (no nav
  target motion, no rate setpoints, `alpha` is a tuned constant not scheduled).

> The SITL run says the **logger** works. It says nothing about the controller —
> Hoops uses `ins ext_pose` and stock NPS still has no mocap feed (the known
> blocker from 2026-06-25), so flight behaviour in SITL is not meaningful here.

## 4. Bugs found and fixed

- **bug-283** — clamp `MFC_CSV_ABS_MAX` was 1e6 but the ×10000 fixed-point
  conversion overflows int32 above 2.147e5, so `1e7` printed as `214748.3647`.
  Bound is now 2e5, derived from the scale. *Found by the host test, not by
  reading the code.*
- **bug-284** — `STAB_ATTITUDE` was listed **twice** in the FlightRecorder
  process of `mfc_flight_test.xml` (periods `.1` and `0.01`). Slower duplicate
  removed. That file is shared by ANTON_MFC/HEOL/FINDI, so nothing else in it
  was touched — see "not done" below.
- **bug-285** — SITL CSV looked missing: the sim container's cwd is the repo
  **root** (not `paparazzi/`), and `sim_anton.py` never exits so the SIGKILL
  skips `fclose`. Added a periodic `fflush()` every 100 rows on the non-ChibiOS
  path. ChibiOS unaffected (sdLog flushes on its own 10 s period).

## 5. User correction — SITL run length

I ran the sim with `timeout 420` / `timeout 360` for a 15 s nav sequence. **~30 s
is enough.** `./sim.sh` **never exits on its own** (`while True: time.sleep(1)`,
no `--duration` flag), so every run is killed and a **non-zero exit (124/137) is
the expected outcome**. Rule now in `CLAUDE.md`:
`timeout ≈ (sum of --nav "+N" waits) + ~20 s startup margin`.

## 6. Not done / open for the flight

- **The write budget was not measured.** The CSV adds ~90–120 KB/s at 100 Hz on
  top of the binary FlightRecorder log, which still carries STAB_MFC /
  GUIDANCE_MFC / WLS / ACC2ATT at 100 Hz. I deliberately did **not** thin the FR
  process: `mfc_flight_test.xml` is shared with three ANTON airframes that have
  no CSV logger and would silently lose their MFC log. **Watch `/uav/LOG/drops`
  (GCS: "MFC CSV log → drops") on the first flight;** if it climbs, raise
  `decim` live rather than editing the shared telemetry file.
- Untested on real hardware — the SD path, the third file descriptor and the
  card's sustained write rate are hardware-only facts.
- `tools/mfcdata` ingest of this file has not been run (the `mfcdata` package
  itself is still unwritten; Phases B/C/D of the data-management plan).

---

## 7. Manual → NAV handover ("Hold Here" block)

**Question:** how do you hand over from a manual attitude hand-flight to NAV
mid-flight, holding wherever the aircraft happens to be? Switching to NAV was
making it fall out of the air.

**Diagnosis — it is not a missing setpoint, it is a zero one.** `AP_MODE_NAV`
does not "hold position"; it *executes the current flight-plan block*. After a
hand takeoff, `Wait GPS` falls through to `Holding point` once the fix is valid,
and that block's stage is `<attitude ... throttle="0" vmode="throttle"/>`.
Switching to NAV there commands **zero throttle**. (`NavKillThrottle()` in the
same block is a no-op outside NAV — see navigation.h:229 — so the motors are
still on; they are just commanded to nothing.)

The controller side was never the problem: `guidance_h_run_enter()` /
`guidance_v_run_enter()` in `guidance_mfc.c` already re-seed the position and
thrust filters, the gz trajectory and the heading from the current state on mode
entry, precisely to avoid entry transients.

**RC switching already works** — `hoops_111_mfc.xml` §AUTOPILOT sets
`MODE_AUTO2 = AP_MODE_NAV`, so the 3-way switch's top position *is* NAV. And
with an RC link connected the **RC switch is the authority**:
`autopilot_static_on_rc_frame()` re-derives the mode from the switch every RC
frame and stamps back anything the GCS set. So a GCS button cannot perform the
handover while RC is connected — it can only prepare a safe target.

**The fix — new `Hold Here` block** in `flat_traj_demo.xml`, purely additive:

```xml
<block name="Hold Here" strip_button="Hold Here" strip_icon="home.png"
       pre_call="if (autopilot_get_mode() != AP_MODE_NAV) { NavSetWaypointHere(WP_STDBY); nav_hold_alt = stateGetPositionEnu_f()->z; nav_set_heading_current(); }">
  <stay wp="STDBY" alt="nav_hold_alt"/>
</block>
```

A block-level `pre_call` is emitted at the top of the block and runs **every nav
tick** before the stage switch (gen_flight_plan.ml:715). So while the autopilot
is *not* in NAV, the hold target is dragged along with the aircraft; the instant
NAV engages the condition goes false and the target **freezes where the aircraft
was at that moment**. That is what makes it "hold right here" rather than "fly
back to where you pressed the button".

Details that matter:

- `NavSetWaypointHere` is `waypoint_set_here_2d` — **horizontal only**. The
  altitude must be latched separately, which is why `nav_hold_alt` exists and is
  passed as an explicit `alt=`. That also dodges the `USE_ALT_LLA_WAYPOINTS`
  frame trap already documented on the Standby block.
- `nav_hold_alt` is declared in the flight plan `<header>`, which the generator
  emits **before** the `WP_*` defines — so the tracking logic itself cannot live
  there and must be inline in `pre_call`.
- `call_once`/`pre_call` bodies are emitted **verbatim** (`lprintf "%s;"`), so
  `->` works in them but the `@DEREF` escape does **not** (that is for
  expression-parsed attributes like `cond=` and `alt=`).
- The block deliberately does **not** call `autopilot_set_mode()`, so it never
  fights the RC switch.

**Procedure:** click `Hold Here` on the GCS strip first (it only tracks while it
is the active block), then flip the RC switch to AUTO2.

## 8. SITL could not validate the handover — arming is broken in the sim

`arming_status` stays at **0 = `AP_ARMING_STATUS_NO_RC`** for the whole run, so
`NavResurrect()` cannot turn the motors on and the aircraft never leaves the
ground. Nav itself is fine (`SP/guidance/v_z` reaches the -2.0 m Standby target);
only arming is stuck.

**This is not caused by the flight-plan change** — verified by stashing it,
rebuilding and re-running the identical known-good nav sequence: same
`arming = 0`, same no-takeoff.

It is also **intermittent**: the first run of the session (the one that validated
the CSV logger — 4481 rows, climbed to 2.34 m AGL, motors on) armed fine. Every
run after it has not. Nothing in the firmware changed between them except the
flight plan, which is now ruled out. **Unexplained; worth its own look before
relying on SITL for mode-transition work.** On the real aircraft the nps target
is irrelevant — the ap target has a real SBUS receiver.

**Consequence: the `Hold Here` block is verified only as far as "generates the
intended C and builds".** Its flight behaviour is untested.
