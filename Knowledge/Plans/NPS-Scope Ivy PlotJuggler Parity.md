# NPS-scope ↔ ivy-server PlotJuggler parity (MFC)

## Context

Two independent PlotJuggler feeds exist for the MFC aircraft (Hoops_111_MFC, ac_id 177):

1. **ivy server** (`sw/ground_segment/tmtc/server.ml`) — streams every received telemetry
   message as UDP/JSON on port 9870, tree = `"<name> (<id>)" / MSG / field`. Rate-limited by
   the radio link (STAB_MFC 20 Hz, GUIDANCE_MFC/WLS 10 Hz). Rich flight-mode/status context.
2. **nps_scope** (`sw/simulator/nps/nps_scope.c` + `nps_scope_var.h` + `modules/nps_scope/nps_scope_state.c`)
   — sim-only in-process UDP/JSON at full sim rate; tree = flat semantic keys
   (`mfc/roll/sp`, `truth/x`, `wls/v0`), root `{"t":…}`.

Goals (user-confirmed):
- **Mirror the message schema** in nps_scope so a single PlotJuggler template lines up
  leaf-for-leaf across both feeds. Rename `t`→`timestamp`, wrap everything under an
  aircraft-name root so both feeds can stream simultaneously as **distinct tree roots**.
- **Close the telemetry gaps** the template needs: add `sp_traj` to `STAB_MFC` and
  `GUIDANCE_MFC`; add the `acc2att` block as a **debug** message (flagged for later removal).
- **Maximal PlotJuggler template**: reproduce the setpoint-vs-measurement plots (the `sp/nav`
  SPvMEAS tab) from data already available over ivy.
- **Fix the `EFF_MAT_STAB` "too many bytes" error** — diagnosed as a generated-definition skew
  (the running server loaded a stale `var/messages.xml` with the pre-`G2` 3-field shape while
  firmware sends 4 arrays). Same tree → fix is regenerate + restart, plus resolve a latent
  `GUIDANCE_MFC`/`TARGET_POS` **id-57 collision**.

Time-base caveat (documented, not blocking): nps_scope `timestamp` = sim time (`fdm.time`),
ivy `timestamp` = wall clock. Simultaneous feeds appear as separate roots but will **not** share
an absolute x-axis. Normal usage is one feed at a time; both-at-once is a convenience.

## Key facts established during exploration

- Authoritative message file: **`sw/ext/pprzlink/message_definitions/v1.0/messages.xml`** (a git
  submodule, currently `567a941` on `feat/shadow-handoff`). `var/messages.xml` is generated from
  it and is what the server loads — never hand-edit `var/`.
- `EFF_MAT_STAB` (id 185): 4 `float[]` (`G1_roll/pitch/yaw/G2`). Firmware senders already pass 4
  arrays (`stabilization_mfc.c:417`, `stabilization_indi.c:381`, etc.) — source is self-consistent.
- `STAB_MFC` (id 212): 19 floats `sp_/me_/err_/fk_/cmd_{phi,theta,psi}` + `u0..u3`.
  `GUIDANCE_MFC` (id 57): 15 floats `sp_/me_/err_/fk_/cmd_{x,y,z}`. Both **lack** sp_traj.
- nps_scope already streams `sp_traj` internally (`mfc_*.setpoint_trajec[0]`,
  `mfc_g*.setpoint_trajec[0]`); the `acc2att` block lives in `guidance_mfc.c` as `nps_*` globals.
- `AIRFRAME_NAME` / `AC_ID` are available to `nps_scope.c` via `#include "generated/airframe.h"`.
- PlotJuggler JSON parser: nested objects AND `/`-bearing keys both build tree paths with `/`,
  and array fields render as `field/field_N`. So a flat key `"STAB_MFC/sp_phi"` nested one level
  under the root yields the **same** full series name as ivy's nested `STAB_MFC→sp_phi`.

## Work items

### 1. nps_scope.c — restructure emitter (mirror schema)  — `sw/simulator/nps/nps_scope.c`
- `#include "generated/airframe.h"`; define root key `AIRFRAME_NAME " (sim)"` (distinct from ivy's
  `"… (177)"`).
- Change datagram framing from `{"t":…, "truth":{…}, "<flatkey>":v, …}` to:
  ```
  { "<AIRFRAME_NAME> (sim)": { "TRUTH":{…}, <registered "MSG/field":v …> }, "timestamp": <fdm.time> }
  ```
  i.e. open root object, emit the hard-coded ground-truth block as `"TRUTH":{…}` (rename from
  `truth`), append the registered vars (their `append_var` `,"name":v` fragments are unchanged and
  now sit inside the root), close root, then append `,"timestamp":<fdm.time>}`.
- Keep buffer/decimation logic; bump `char buf[8192]` to e.g. 16384 (more fields now wrapped).

### 2. Rename NPS_SCOPE_VAR names to the message schema (pattern across ~8 files)
Pattern: replace semantic prefixes with `MSG/field` names matching telemetry. Representative files
and mapping (axis roll/pitch/yaw → phi/theta/psi; x/y/z kept):

- `stabilization/stabilization_mfc.c` & `oneloop/oneloop_mfc.c` (STAB_MFC):
  `mfc/<axis>/{sp,sp_traj,meas,err,fk,cmd}` → `STAB_MFC/{sp_,sp_traj_,me_,err_,fk_,cmd_}<phi|theta|psi>`;
  register `mfc_u[0..3]` as `VARN("STAB_MFC/u", …)` → `STAB_MFC/u0..u3` (match the message's u0..u3).
- `guidance/guidance_mfc.c` & `oneloop/oneloop_mfc.c` (GUIDANCE_MFC):
  `mfc_g/<ax>/{sp,sp_traj,meas,fk,cmd}` → `GUIDANCE_MFC/{sp_,sp_traj_,me_,fk_,cmd_}<x|y|z>`; **add**
  `GUIDANCE_MFC/err_<ax>` ← `&mfc_g<ax>.error[0]` (message has err_*, nps currently omits it).
  Move the `acc2att` block to its own debug branch `ACC2ATT/<vhatx,vhaty,phi_d,theta_d,cosphid,vhaty_cd,att_sp_phi,att_sp_theta,att_sp_psi,u1>`.
- `stabilization/stabilization_indi.c` & `oneloop/oneloop_mfc.c` (WLS): change array prefixes so
  the indices match ivy's array rendering: `VARN("WLS_V/v/v_", …)`→`WLS_V/v/v_0…`,
  `VARN("WLS_U/u/u_", …)`→`WLS_U/u/u_0…` (and the `mfc_wls/` variants likewise).
- `modules/nps_scope/nps_scope_state.c`: these are sim-only extras (no exact ivy message); keep as
  `TRUTH`-sibling branches but uppercase/group to read cleanly: `EST/*`, `SENSORS/*`, `SP/nav/*`,
  `SP/guidance/*`, `SP/stab/*`, `MODE/*`. (No telemetry counterpart required.)

### 3. Telemetry message changes  — `sw/ext/pprzlink/message_definitions/v1.0/messages.xml`
- `STAB_MFC` (id 212): add `sp_traj_phi`, `sp_traj_theta`, `sp_traj_psi` (float).
- `GUIDANCE_MFC`: add `sp_traj_x`, `sp_traj_y`, `sp_traj_z` (float). **Reassign its id** off 57 to a
  verified-free telemetry id (57 collides with `TARGET_POS`). Confirm both are in the `telemetry`
  class before reassigning.
- New **debug** message `GUIDANCE_MFC_ACC2ATT` (free id; comment-flag as debug/temporary) with the
  acc2att fields. Bandwidth-cheap, low rate.
- Submodule discipline: commit inside `sw/ext/pprzlink`, bump the parent gitlink.

### 4. Firmware send-site updates
- `guidance/guidance_mfc.c:210` `pprz_msg_send_GUIDANCE_MFC` → add the three `&mfc_g{x,y,z}.setpoint_trajec[0]`
  args; add a `pprz_msg_send_GUIDANCE_MFC_ACC2ATT` call + registration for the acc2att globals.
- `stabilization/stabilization_mfc.c:446` and `oneloop/oneloop_mfc.c:533` `pprz_msg_send_STAB_MFC`
  → add the three `&mfc_{roll,pitch,yaw}.setpoint_trajec[0]` args.
- `conf/telemetry/mfc_flight_test.xml`: add `GUIDANCE_MFC_ACC2ATT` at a low period (e.g. 0.2 s);
  STAB_MFC/GUIDANCE_MFC unchanged (sp_traj rides existing messages).

### 5. EFF_MAT_STAB / sync fix (same tree — operational)
- Regenerate `var/messages.xml` from the submodule (happens via the message-gen step of a normal
  build) and **restart the server** so the loaded definition matches firmware (resolves
  "too many bytes"). The message edits in (3) require this regen+restart anyway.
- Rebuild + reflash firmware from this tree so the live md5 matches conf (or run the server with
  `-no_md5_check` during dev). Investigate the stray `AC_ID 112` on the bus (unregistered in
  `conf_enac.xml`) — likely an old flash; confirm what is actually transmitting.

### 6. PlotJuggler template rewrite (maximal)  — `plotjuggler_mfc.xml`
- Re-point every curve to the mirrored schema. For shared controller tabs (MFC_RPY, MFC_XYZ, WLS,
  STAB_MFC/GUIDANCE_MFC), include **both** roots' paths — `"Hoops_111_MFC (177)/…"` (ivy) and
  `"Hoops_111_MFC (sim)/…"` (sim) — so the template works for either or both feeds; absent series
  simply don't draw.
- Sim-only tabs (Truth, Sensors, Estim) → sim root (`…/TRUTH/*`, `…/SENSORS/*`, `…/EST/*`).
- **SPvMEAS tab (maximal)**: plot setpoint-vs-measurement using ivy-available data the way the old
  `sp/nav` tab did — ivy `ROTORCRAFT_FP` (target/carrot/pos) + `GUIDANCE_MFC` (sp_*/sp_traj_*) +
  sim `TRUTH`/`EST` overlays.
- Keep `<Plugins>`/UDP-Server block; document two UDP-Server instances (9870 ivy, separate port for
  the relayed sim feed) in a header comment.

## Critical files
- `sw/simulator/nps/nps_scope.c` (emitter restructure)
- `sw/airborne/firmwares/rotorcraft/{stabilization/stabilization_mfc.c, oneloop/oneloop_mfc.c, guidance/guidance_mfc.c, guidance/guidance_indi.c, stabilization/stabilization_indi.c}` and `modules/nps_scope/nps_scope_state.c` (rename registrations)
- `sw/ext/pprzlink/message_definitions/v1.0/messages.xml` (submodule: sp_traj, id-57 fix, acc2att debug msg)
- `conf/telemetry/mfc_flight_test.xml` (acc2att debug entry)
- `plotjuggler_mfc.xml` (template rewrite)

## Verification
1. **Build nps**: `./pprz.sh build Hoops_111_MFC nps` — compile success confirms all renamed
   registrations resolve. (Also build the `ap` target to validate the message-gen + send sites.)
2. **Inspect sim JSON**: run NPS with `--scope_host <ip>`; on the receiver
   `socat -u UDP4-RECV:<port> -` (or tcpdump) and confirm the new shape
   `{ "Hoops_111_MFC (sim)": { "STAB_MFC": {…,"sp_traj_phi":…}, "GUIDANCE_MFC": {…}, "WLS_U": {"u":{"u_0":…}}, "TRUTH": {…}, "ACC2ATT": {…} }, "timestamp": … }`.
3. **Regen + restart server**; confirm `EFF_MAT_STAB` decodes with no "too many bytes", and that
   `STAB_MFC`/`GUIDANCE_MFC` now expose `sp_traj_*` in the ivy JSON. Confirm no id-57 misdecode.
4. **Template**: open `plotjuggler_mfc.xml` against (a) the ivy feed and (b) the sim feed; verify
   the shared controller leaves populate from both roots and the SPvMEAS tab draws from ivy data.
5. Optional both-at-once: two UDP-Server instances, confirm two coexisting roots (note the
   sim-time vs wall-clock x-axis caveat).
