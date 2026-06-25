# Plan — MFC Flight-Test Enablement (telemetry + SD fast-logging + analysis)

**Goal:** get everything in place to flight-test the standalone MFC stack
(`stabilization_mfc.c` + `guidance_mfc.c`) on the real aircraft **`Hoops_111_MFC`**
(ac_id 111, board `tawaki_2.0`). Attitude-only first, then the full guidance+stab stack.

Scope, in the user's words:
1. Make sure the **right telemetry** is downlinked to debug the system, and get the
   **SD-logging module** working with everything needed for **fast logging of MFC detail**.
3. Fold **`tune_mfc.sh`** and **`analyze_mfc.py`** into that new telemetry/fast-log system,
   with compatibility between the Ivy messages and the custom logging types — and make the
   **SD-card log format match the NPS_SCOPE JSON schema** so it loads into PlotJuggler / is
   analysed the same way as NPS.

(Item 2 — the `sim_anton.py` rework — is its own document: see
`Knowledge/Plans/sim_anton Paparazzi-Native Rework.md`. It shares the JSON schema defined here.)

Date drafted: 2026-06-25. "Assume basically no changes to the system" — i.e. reuse the
existing MFC C code and message set; this plan is about wiring, config, and tooling.

---

## Current-state audit (what exists, what's broken)

**Aircraft registration** — `conf/airframes/ENAC/conf_enac.xml:134` (`Hoops_111_MFC`, ac_id 111):
- ❌ **No SD logger:** `settings_modules` has no `modules/logger_sd_chibios.xml` (every other
  ENAC aircraft does). Without it there is no onboard fast log.
- ⚠️ `telemetry="telemetry/mfc_flight_test.xml"` — file **exists** but is a near-verbatim copy of
  `default_rotorcraft.xml`: **none of the MFC messages are enabled** in either the `Main` or the
  `FlightRecorder` process (`conf/telemetry/mfc_flight_test.xml`).
- `flight_plan` reuses `flight_plans/ENAC/anton_mfc_attitude.xml` (blocks: Wait GPS / Holding
  point / Start Engine(2) / Takeoff(3) / Pitch Step / Hover). Fine to start.

**Airframe** — `conf/airframes/ENAC/quadrotor/hoops_111_indoor.xml`:
- ❌ Still wired to INDI: `<module name="stabilization" type="indi"/>` and
  `<module name="guidance" type="indi"/>` (lines 47–48). To flight-test MFC these must become
  `type="mfc"`, and the airframe needs `STABILIZATION_MFC` + `GUIDANCE_MFC` sections (none present).
- Uses `gps optitrack` + `ins ext_pose`, `INS_EKF2_OPTITRACK=TRUE`, indoor OptiTrack setup,
  `PERIODIC_FREQUENCY=500`, telemetry `xbee_api`, actuators `dshot` (bidir). DShot bidir gives RPM
  feedback — relevant: MFC has `RPM_FEEDBACK`/`ACT_FEEDBACK_ID` settings.

**MFC telemetry that already exists** (registered by `stabilization_mfc.c:581–586`):
- `STAB_MFC` (id 212) — sp/meas/err per axis, `F_k` disturbance estimate, pre-WLS virtual cmd,
  motor outputs u0–u3. This is the core MFC debug message.
- `WLS_V` (187), `WLS_U` (188), `EFF_MAT_STAB` (185), `STAB_ATTITUDE` — also registered.
- ❌ **`guidance_mfc.c` sends no telemetry at all** and there is **no `GUIDANCE_MFC` message**
  in `messages.xml`. Full-stack (guidance) debugging has no dedicated downlink today — a gap to
  close before the "full stack" flight test.

**SD logging chain:** `logger_sd_chibios` logs the **`FlightRecorder`** telemetry process as a
binary `.TLM` on the SD card (FatFS). Offline it is decoded by `sw/logalizer/sd2log` →
`.log`+`.data` (the same pair the GCS server produces). So **whatever messages we list under the
`FlightRecorder` process in `mfc_flight_test.xml` are exactly what gets fast-logged**.

**NPS_SCOPE JSON schema** (`sw/simulator/nps/nps_scope.c:161`): one UDP datagram per (decimated)
sim step: `{"t":<simtime>, "truth":{x,y,z,phi,theta,psi,p,q,r,...}, "<folder>/<name>":val, ...}`.
Firmware vars are registered with `NPS_SCOPE_VAR("folder/name",&addr,type)` **inside the module
that owns them** (`nps_scope_state.c` is generic — leave it alone). The **full MFC schema already
exists**:
- `stabilization_mfc.c:370–391` — `mfc/{roll,pitch,yaw}/{sp,sp_traj,meas,err,fk,cmd}`, plus
  `mfc/act` (actuator array, `NPS_SCOPE_VARN`), `wls/v`, `wls/u`.
- `guidance_mfc.c:168–192` — `mfc_g/{x,y,z}/{sp,sp_traj,meas,fk,cmd}`, plus `mfc_g/acc2att/*`
  (the accel→attitude conversion internals).

So everything `analyze_mfc.py` needs is **already streamed in sim**. The scope key names are the
de-facto schema contract; they can be renamed *in those module files* if alignment with the SD/
pprzlink field names is wanted, but nothing new needs registering.

---

## Phase 0 — Fix `Hoops_111_MFC` so it builds the MFC stack (blocking)

1. `conf_enac.xml:142` — add `modules/logger_sd_chibios.xml` to `settings_modules`.
2. `hoops_111_indoor.xml` — switch the firmware modules:
   - `<module name="stabilization" type="mfc"/>` (carry `MFC_OUTPUTS=4`, `MFC_NUM_ACT=4`).
   - `<module name="guidance" type="mfc"/>` for the full-stack phase. **For the first
     attitude-only test, keep `guidance` as `indi`/`rotorcraft`** (NAV/GUIDED drives vertical +
     position, MFC owns attitude) — decide per the open question on flight mode below.
   - Add `<section name="STABILIZATION_MFC" prefix="STABILIZATION_MFC_">` (copy the tuned values
     from `anton_mfc.xml` as the starting point — the modules' defaults are generic) and, for full
     stack, `<section name="GUIDANCE_MFC" prefix="GUIDANCE_MFC_">`.
   - Add the `logger_sd_chibios` module to the airframe `ap` target if not pulled in by the conf
     line (confirm Tawaki v2 SDIO device / `SDLOG_SDIO`).
3. Build gate (compile == only automated check):
   `./pprz.sh build Hoops_111_MFC ap` and `./pprz.sh build Hoops_111_MFC nps`.

---

## Phase 1 — Debug telemetry (downlink, `Main` process)

Edit `conf/telemetry/mfc_flight_test.xml`, `Main` process `default` mode, to add the MFC signals.
Keep the existing housekeeping (`ROTORCRAFT_STATUS`, `ROTORCRAFT_FP`, `DL_VALUE`, `ALIVE`).

Attitude-test set (downlink is XBee — bandwidth-limited, so pick rates carefully):
```xml
<message name="STAB_MFC"      period="0.05"/>   <!-- 20 Hz: core MFC debug -->
<message name="STAB_ATTITUDE" period="0.1"/>
<message name="WLS_V"         period="0.1"/>
<message name="WLS_U"         period="0.1"/>
<message name="ROTORCRAFT_CMD" period="0.1"/>
<message name="ROTORCRAFT_FP" period="0.25"/>
```
XBee throughput is the constraint (not 500 Hz onboard). 20 Hz `STAB_MFC` is a debug compromise;
the **full-rate** picture comes from the SD log (Phase 2), not the radio. Tune periods against the
link budget on the bench.

**Full-stack addition — new `GUIDANCE_MFC` message (closes the gap):**
- Add `<message name="GUIDANCE_MFC" id="NNN">` to
  `sw/ext/pprzlink/message_definitions/v1.0/messages.xml` (pick a free id near the MFC block, e.g.
  in the 210s; verify no collision). Fields per axis (gx/gy/gz): pos sp, pos meas, err, `F_k`
  estimate, accel/thrust command — mirroring `STAB_MFC`'s structure for the guidance layer.
- Add a `send_guidance_mfc()` + `register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GUIDANCE_MFC, …)`
  in `guidance_mfc.c` (model on `stabilization_mfc.c:446/583`).
- Add it to both telemetry processes.

---

## Phase 2 — SD fast-logging (`FlightRecorder` process)

The SD log is the **high-rate** record (onboard, not radio-limited). Populate the
`FlightRecorder` process in `mfc_flight_test.xml` with the full MFC detail at high rate:
```xml
<process name="FlightRecorder">
  <mode name="default">
    <message name="STAB_MFC"      period="0.002"/>  <!-- up to 500 Hz (PERIODIC_FREQUENCY) -->
    <message name="WLS_V"         period="0.002"/>
    <message name="WLS_U"         period="0.01"/>
    <message name="STAB_ATTITUDE" period="0.01"/>
    <message name="GUIDANCE_MFC"  period="0.01"/>   <!-- full stack -->
    <message name="IMU_GYRO_SCALED"  period="0.005"/>
    <message name="IMU_ACCEL_SCALED" period="0.005"/>
    <message name="ESC"           period="0.01"/>    <!-- DShot bidir RPM -->
    <message name="ROTORCRAFT_FP" period="0.1"/>
    <message name="INS"           period="0.05"/>
    <!-- housekeeping: AUTOPILOT_VERSION, ROTORCRAFT_STATUS, GPS_INT, ENERGY -->
  </mode>
</process>
```
Notes / gotchas:
- The achievable rate is capped by `PERIODIC_FREQUENCY` (500 Hz here) and the SD write throughput
  / `SDLOG_AUTO_FLUSH_PERIOD`. Start at 100–200 Hz for `STAB_MFC`, push toward 500 Hz only if the
  logger keeps up (watch `LOGGER_STATUS`).
- `logger_sd_chibios` config worth setting on `Hoops_111_MFC`: `SDLOG_PREFLIGHT_ERROR=TRUE`
  (refuse to arm without a running logger — good discipline for a test campaign),
  `SDLOG_START_DELAY`, `SDLOG_SDIO` (confirm Tawaki v2 device).
- Confirm any **new** message (`GUIDANCE_MFC`) is also added to the `FlightRecorder` mode, or it
  won't make it onto the card.

---

## Phase 3 — Unified JSON schema (SD log ↔ NPS_SCOPE)

**Target:** one schema, two producers, one analyser. The NPS_SCOPE JSON
(`{"t":…, "truth":{…}, "<folder>/<name>":val,…}`) is the canonical format because PlotJuggler
already ingests it live. Make the SD log convertible into the *same* shape.

Two halves:

**(a) Scope side — already done; just adopt it as the contract.** The MFC internals are already
registered in the scope by their owning modules (`mfc/*`, `mfc_g/*`, `wls/*`, `mfc/act` — see the
audit). **No edit to `nps_scope_state.c` (it is generic).** The only optional work here is
*renaming* the existing scope-var strings *in `stabilization_mfc.c` / `guidance_mfc.c`* if we want
them to read identically to the SD/pprzlink field names — purely cosmetic for schema alignment. The
canonical key set is therefore the current scope output:
`mfc/{roll,pitch,yaw}/{sp,sp_traj,meas,err,fk,cmd}`, `mfc/act`, `wls/{v,u}`,
`mfc_g/{x,y,z}/{sp,sp_traj,meas,fk,cmd}`, `mfc_g/acc2att/*`, and `truth/*`.

**(b) SD-log → JSON converter** (new tool, e.g. `tools/sdlog2scope.py`):
`*.TLM (SD) → sd2log → .log/.data → JSON stream`, emitting one object per timestamp with keys that
**match the existing scope key set above**. The mapping is pprzlink message+field → scope key, e.g.:
- `STAB_MFC.err_phi → mfc/roll/err`, `.fk_theta → mfc/pitch/fk`, `.cmd_psi → mfc/yaw/cmd`,
  `.u0..u3 → mfc/act` (array), `.me_phi → mfc/roll/meas`, `.sp_phi → mfc/roll/sp`.
- `WLS_V → wls/v`, `WLS_U → wls/u`.
- `GUIDANCE_MFC.* → mfc_g/{x,y,z}/*` (the new full-stack message, Phase 1).
- `truth/*` ← estimated state (no JSBSim truth in a real flight: use INS/OptiTrack estimate).

That mapping table is the single source of cross-compatibility between "Ivy/pprzlink messages" and
"the custom JSON logging type". Output loads into PlotJuggler identically to a sim capture, and into
the analyser below. Firmware writes to the card are unchanged (binary pprzlog); only the converter
is new.

This is exactly the "SD format should match the json format of NPS_SCOPE" requirement: we don't
change what the firmware writes to the card (binary pprzlog, unchanged), we add a converter whose
output schema == the scope schema.

---

## Phase 4 — Fold `tune_mfc.sh` + `analyze_mfc.py` into the unified system

Today `analyze_mfc.py` reads the bespoke wide CSV that `sim_anton.py` writes (keys like
`mfc_err_phi`, `mfc_fk_phi`, `agl`, `rc_thrust`). Retarget it onto the unified JSON schema so the
**same analyser works on sim captures and real flights**:

- **Reader:** accept the NPS_SCOPE JSON stream / capture file (and the Phase-3 SD→JSON output)
  instead of (or in addition to) the CSV. The CSV keys map onto the **existing** scope keys:
  `mfc_err_phi`→`mfc/roll/err`, `mfc_fk_*`→`mfc/{roll,pitch,yaw}/fk`, `mfc_cmd_*`→`mfc/<axis>/cmd`,
  `mfc_u0..3`→`mfc/act`, `agl`→`truth/agl`, attitude→`truth/phi…` (sim) or estimated state (flight).
  Keep the existing metric math (per-axis RMS/max error, F_k range, saturation, AGL/thrust,
  divergence/crash flags).
- **`tune_mfc.sh`:** keep the build→run→analyse loop but make it schema-driven:
  - Capture the NPS scope stream to a file during the timed sim (a small UDP→file sink, or reuse
    PlotJuggler's recording), then run `analyze_mfc.py` on that JSON — dropping the dependence on
    `sim_anton.py`'s CSV entirely.
  - Add a `--flight LOG.TLM` path that runs Phase-3 conversion then the same analyser, so the one
    script grades both a sim run and a downloaded SD log.
- **Live Ivy compatibility:** the analyser's field map is derived from the pprzlink message
  definitions, so an Ivy listener (e.g. from the `sim_anton` rework, or `messages.py`) can feed the
  *same* keys for real-time grading. One field dictionary, three feeds: NPS scope JSON, SD→JSON,
  live Ivy.

---

## Touch points (checklist)

- [ ] `conf_enac.xml:142` — fix `stabilization_mfc.xm` typo; add `logger_sd_chibios.xml`
- [ ] `hoops_111_indoor.xml` — `stabilization type=mfc` (+ guidance for full stack); add
      `STABILIZATION_MFC` / `GUIDANCE_MFC` sections; confirm SD logger config for Tawaki v2
- [ ] `conf/telemetry/mfc_flight_test.xml` — `Main`: add `STAB_MFC`/`WLS_*`/`STAB_ATTITUDE`;
      `FlightRecorder`: high-rate MFC set
- [ ] `messages.xml` — add `GUIDANCE_MFC` message; `guidance_mfc.c` — `send_guidance_mfc()` +
      register (downlink/SD only; scope already has `mfc_g/*`)
- [ ] ~~register scope signals~~ — **already done** in `stabilization_mfc.c`/`guidance_mfc.c`; do
      NOT touch `nps_scope_state.c`. (Optional: rename scope-var strings in those modules for
      schema alignment.)
- [ ] NEW `tools/sdlog2scope.py` — `.TLM → sd2log → JSON keyed to the existing scope schema`
- [ ] `analyze_mfc.py` — read unified JSON; keep metrics
- [ ] `tune_mfc.sh` — capture scope JSON; add `--flight LOG.TLM` path
- [ ] Build gate: `./pprz.sh build Hoops_111_MFC {ap,nps}`

## Validation sequence
1. **Sim:** `./pprz.sh build Hoops_111_MFC nps`; run sim; confirm `mfc/*` + `mfc_g/*` already show in
   PlotJuggler (scope); `analyze_mfc.py` grades the scope JSON.
2. **Bench (real AP):** flash `ap`; confirm GCS shows MFC settings panel + `STAB_MFC` downlink;
   confirm SD log is created and `sd2log`+`sdlog2scope.py` round-trips to PlotJuggler-loadable JSON;
   confirm `LOGGER_STATUS`/flush keep up at the chosen `FlightRecorder` rate.
3. **Flight — attitude only:** hover under MFC attitude (guidance non-MFC); download SD log; grade
   with the same `analyze_mfc.py`.
4. **Flight — full stack:** enable `guidance type=mfc` + `GUIDANCE_MFC` telemetry; repeat.

## Risks
- XBee bandwidth: downlink `STAB_MFC` rate must respect the link budget — the SD log is the real
  high-rate record. Don't over-subscribe the radio.
- SD write throughput vs. 500 Hz logging — ramp the rate; watch for dropped records.
- New `GUIDANCE_MFC` message id collision — verify against `messages.xml` and regen.
- Compile ≠ correct. NPS, then bench, then flight.

## Open questions
See the consolidated list at the end of the session (flight mode for attitude-only test, message
id choice, downlink rate budget, OptiTrack vs onboard state for the SD `truth/*` mapping).
