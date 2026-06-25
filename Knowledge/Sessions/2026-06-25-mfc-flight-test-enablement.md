# 2026-06-25 — MFC Flight-Test Enablement (Hoops_111_MFC)

Implemented `Knowledge/Plans/MFC Flight-Test Enablement.md`: telemetry + SD
fast-logging + unified analysis so the standalone MFC stack can be flight-tested
on `Hoops_111_MFC` (ac_id 111, board `tawaki_2.0`).

## What changed

**Phase 0 — airframe/config (`hoops_111_indoor.xml`, `conf_enac.xml`)**
- Switched the firmware to the **full MFC stack**: `stabilization type="mfc"`
  (WLS_N_U_MAX/V_MAX=4) + `guidance type="mfc"`, mirroring the proven
  `anton_mfc.xml`. Documented the **paired 2-line swap to attitude-only**
  (guidance→`indi`) right above the modules.
- Added `STABILIZATION_MFC` (this airframe's own G1/G2/ACT_FREQ — fixed the
  pre-existing G1 thrust-row comma typo `{-0.7,-0.7 -0.7,-0.7}`; anton MFC SISO
  gains as a retune-me starting point) and `GUIDANCE_MFC` sections. The three
  `*_PPRZ_SCALE`s are derived from Hoops's G1 (THRUST=1000/(4·-0.7),
  TILT=1000/(2·14), TWIST=1000/(2·0.9)) — they set command clamps in BOTH
  guidance_mfc.c (gz) and stabilization_mfc.c (roll/pitch/yaw).
- Added `<module name="flight_recorder"/>` (pulls in `logger_sd_chibios`+`pprzlog`
  by dependency) → SD fast-log chain. Added `logger_sd_chibios.xml` to the Hoops
  `settings_modules` (GCS panel). Added `nps_scope_state` to the nps target.
- **No `STABILIZATION_MFC_COMMANDS` mapping**: Hoops drives motors via
  `actuators_pprz[]` command_laws (same path INDI used) and stabilization_mfc.c
  writes that global unconditionally — COMMAND_FR/BR/BL/FL don't exist here.

**Phase 1 — debug telemetry**
- New `GUIDANCE_MFC` pprzlink message **id 57** (free gaps were 7/13/51/57),
  mirroring STAB_MFC for the guidance layer (per-axis sp/meas/err/fk + cmd).
  Added `send_guidance_mfc()` + `register_periodic_telemetry` in `guidance_mfc.c`.
- `mfc_flight_test.xml` Main/default: STAB_MFC@20Hz, STAB_ATTITUDE/WLS_V/WLS_U/
  GUIDANCE_MFC@10Hz, ROTORCRAFT_CMD, LOGGER_STATUS (XBee budget).

**Phase 2 — SD fast-log**: `mfc_flight_test.xml` FlightRecorder: STAB_MFC/WLS_V
@200Hz (0.005; →0.002 for 500Hz if logger keeps up), WLS_U/STAB_ATTITUDE/
GUIDANCE_MFC@100Hz, IMU@200Hz, ESC@100Hz.

**Phase 3+4 — unified analysis tooling**
- NEW `tools/sdlog2scope.py`: decoded `.data` (sd2log output) → NPS_SCOPE JSON
  (one ndjson object per STAB_MFC sample, forward-filling GUIDANCE_MFC/WLS;
  truth/* from MFC measured attitude+position). The message+field→scope-key map
  is the single cross-compatibility contract.
- `analyze_mfc.py`: now auto-detects CSV **or** scope JSON; one analyser grades
  sim CSV, recorded scope JSON, and converted SD flight logs identically.
- `tune_mfc.sh`: added `--scope FILE` and `--flight LOG.data` paths (+`--aircraft`).

## Verified
- **Build gate PASSED**: `./pprz.sh rebuild Hoops_111_MFC {nps,ap}` both produce
  ELF. `send_guidance_mfc` in ap.elf; `flight_recorder.o`/`sdlog_chibios.o`/
  `sdLog.o` compiled into ap.
- Regenerated pprzlink headers first (`make ... pymessages` — `pprz.sh build`
  does NOT do this; needed for `PPRZ_MSG_ID_GUIDANCE_MFC`).
- Python pipeline end-to-end on a synthetic `.data`: sdlog2scope → analyze_mfc
  (AGL/errors/thrust all correct); legacy CSV path still works.

## Decisions / assumptions
- Defaulted to **full MFC stack** (not attitude-only) because it's the proven,
  guaranteed-to-build config and gives "everything in place"; attitude-only is a
  documented 1-line swap. The cross pair guidance=indi+stab=mfc may hit the MFC
  thrust-unit union bug (bug-122) — prefer full MFC or the documented swap.
- MFC gains are **starting points needing flight tuning**; only the structural
  PPRZ scales were derived from Hoops's effectiveness.
- Left `SDLOG_PREFLIGHT_ERROR` at default FALSE (won't block arming on a first
  flight); documented how to set TRUE for campaign discipline.

## SITL test results (live NPS run, this session)
Validated the two SITL-testable items by tapping the Ivy bus during a real
`./sim.sh Hoops_111_MFC` run (sibling `--network host` container, Ivy
`127.255.255.255:2010`, a throwaway listener writing a paparazzi `.data` capture):
- **Telemetry decode ✅** — `STAB_MFC` 401 msgs @20 Hz (19 fields), **`GUIDANCE_MFC`
  id 57** 200 msgs @10 Hz (15 fields), both ac_id 111, fields populated.
- **Offline chain ✅** — fed the captured `.data` through `tools/sdlog2scope.py`
  (401 rows) → `analyze_mfc.py`: full report printed. (The `.data` came from the
  Ivy bus, not a real card — same text format `sd2log` emits, so it's a faithful
  stand-in; the actual on-card write + `sd2log` decode remain hardware-only.)

### ⚠️ Blocker for SITL flight-behavior testing: `ins ext_pose` has no NPS feed
That run DIVERGED (attitude 60–70°, AGL −2→65 m, `me_z` started at −44 m) — NOT an
MFC fault. Hoops uses `ins ext_pose` (OptiTrack) and **stock NPS provides no
mocap/ext_pose feed** (no NPS sender; `ins_ext_pose.c` has no SITL path; only
hoops_111_indoor + hexa_tilted_motors use ext_pose, all other ENAC quads use
`ekf2` for sim). So the AP flies on a garbage state estimate. To exercise MFC
flight behavior in SITL, override the **nps target** to `ins ekf2` (what anton_mfc
uses, flies in NPS) while keeping `ext_pose` on `ap`; or wire an NPS mocap feed.
Not yet applied — flagged to user.

## Next (manual / hardware)
1. NPS: run the sim, confirm `mfc/*`+`mfc_g/*` in PlotJuggler, grade scope JSON.
2. Bench: flash `ap` (`.hex`/`.bin`, not `.elf` — CubeProgrammer zero-size RAM
   segs), confirm GCS MFC panel + STAB_MFC/GUIDANCE_MFC downlink, SD log created,
   `sd2log`→`sdlog2scope.py` round-trips, LOGGER_STATUS keeps up at chosen rate.
3. Flight: attitude-only (swap guidance→indi) → download SD → analyze_mfc; then
   full stack.
