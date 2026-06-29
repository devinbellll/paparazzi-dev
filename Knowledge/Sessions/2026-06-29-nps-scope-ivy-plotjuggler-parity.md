# Session — 2026-06-29: NPS-scope ↔ ivy-server PlotJuggler parity (MFC)

Executed `Knowledge/Plans/NPS-Scope Ivy PlotJuggler Parity.md`. Goal: make the
NPS scope emit the same tree shape as the ivy server so one PlotJuggler template
lines up leaf-for-leaf across both feeds; close the `sp_traj` telemetry gaps; add
an acc2att debug message.

## What changed

- **`sw/simulator/nps/nps_scope.c`** — datagram restructured to
  `{ "<AIRFRAME_NAME> (sim)": { "TRUTH":{…}, "<MSG>/<field>":v, … }, "timestamp": fdm.time }`.
  Includes `generated/airframe.h` for the root key; `truth`→`TRUTH`; `t`→`timestamp`
  (moved outside the root); buffer 8192→16384.
- **Registration renames to message schema** (axis roll/pitch/yaw → phi/theta/psi):
  - `stabilization_mfc.c`, `oneloop_mfc.c` → `STAB_MFC/{sp,sp_traj,me,err,fk,cmd}_<phi|theta|psi>`, `STAB_MFC/u0..u3` (= `mfc_u`).
  - `guidance_mfc.c`, `oneloop_mfc.c` → `GUIDANCE_MFC/{sp,sp_traj,me,err,fk,cmd}_<x|y|z>` (added `err_*`), acc2att → `ACC2ATT/*`.
  - `stabilization_indi.c`, `oneloop_mfc.c`, `stabilization_mfc.c` WLS → `WLS_V/v/v_0…`, `WLS_U/u/u_0…`.
  - `nps_scope_state.c` extras uppercased: `EST/ SENSORS/ SP/ MODE/`.
- **`messages.xml`** (pprzlink submodule): `STAB_MFC` +`sp_traj_phi/theta/psi`;
  `GUIDANCE_MFC` +`sp_traj_x/y/z`; new debug `GUIDANCE_MFC_ACC2ATT` (id **51**).
- **Send sites**: `STAB_MFC` (stabilization_mfc.c + oneloop_mfc.c) and `GUIDANCE_MFC`
  (guidance_mfc.c) carry the 3 `sp_traj` args; added `send_guidance_mfc_acc2att` +
  registration. acc2att `nps_*` globals + their assignment made unconditional (were
  `#ifdef SITL`) so the ap flight-test message can read them.
- **`conf/telemetry/mfc_flight_test.xml`**: `GUIDANCE_MFC_ACC2ATT` in Main (0.2 s) and FlightRecorder (0.02 s).
- **`pprz_docker.sh`**: export `CAML_LD_LIBRARY_PATH=…/var/lib/ocaml/pprzlink:…/sw/lib/ocaml` (build fix, below).
- **`plotjuggler_mfc.xml`**: every curve re-pointed to the mirrored schema; shared
  controller branches (STAB_MFC, GUIDANCE_MFC, WLS_V/U, GUIDANCE_MFC_ACC2ATT) carry
  BOTH roots (`(sim)` + `(111)`); SPvMEAS augmented with GUIDANCE_MFC/STAB_MFC sp/me
  overlays; header documents the dual UDP-Server setup + sim-time vs wall-clock caveat.

## Verification

- `./pprz.sh build Hoops_111_MFC nps` → `nps.elf` ✓ and `… ap` → `ap.elf` ✓.
- Regenerated C message headers: `make -C sw/ext/pprzlink pymessages … VALIDATE_XML=FALSE`.
- **Sniffed live scope JSON** (ran simsitl --norc, UDP listener on 9871): valid JSON,
  top-level `["Hoops_111_MFC (sim)","timestamp"]`, branches ACC2ATT/EST/GUIDANCE_MFC/
  MODE/SENSORS/SP/STAB_MFC/TRUTH/WLS_U/WLS_V, with `STAB_MFC/sp_traj_phi`,
  `STAB_MFC/u0..u3`, `GUIDANCE_MFC/sp_traj_x`, `GUIDANCE_MFC/err_x`, `WLS_U/u/u_0..`,
  `ACC2ATT/phi_d` all present. ✓

## Gotchas hit

- **OCaml `dllpprzlink_stub` not found** at `nps.ac_h` codegen — fixed by exporting
  `CAML_LD_LIBRARY_PATH` in pprz_docker.sh (bug-160). Pre-existing env gap, surfaced
  after bootstrap reinstalled the pprzlink ocaml lib.
- **Duplicate message id 191** — free-id scan regex missed `name= "…"` (space). Real
  free telemetry ids: 7, 13, 51. Used 51 (bug-161).
- `bootstrap` runs `pprzlink.update`; my submodule edits survived (tree was dirty) but
  watch for reverts.

## Decisions / open items

- **Kept GUIDANCE_MFC at id 57** (no real collision: TARGET_POS 57 is *datalink* class,
  different from telemetry). Deviates from the plan's id-57 reassignment, which was
  based on a cross-class false alarm; reassigning would burn a scarce id. Flagged to user.
- **Submodule NOT committed** — `sw/ext/pprzlink/message_definitions/v1.0/messages.xml`
  is modified; needs a commit inside the submodule + parent gitlink bump (deferred per
  "commit only when asked"). Builds work off the working tree regardless.
- `guidance_v/z_sp|z_ref` left lowercase (out of plan scope; sim-only, no ivy message).

## Next

- Commit pprzlink submodule + bump gitlink when ready to share.
- Validate template against a live ivy feed + the relayed sim feed (couldn't run
  PlotJuggler/server here). Optionally add ROTORCRAFT_FP overlays to SPvMEAS.
- Remove GUIDANCE_MFC_ACC2ATT once the acc2att path is validated (flagged debug/temp).
