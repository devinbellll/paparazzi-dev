# Session: 2026-06-11 — MFC thrust-unit link between guidance and stabilizers

## Goal
Make the thrust setpoint hand-off from `guidance_mfc.c` to the downstream
stabilizer unit-correct, without hacking the shared stabilizers. Surgical change.

## Background
`mfc_gz` (vertical MFC guidance) emits a thrust command in **physical
"thrust-objective" / indi_v[3]-space units** (~-17 at hover, raw `mfc_siso_run`
output — not normalized throttle, not a paparazzi command). Previously this was
made to fly by **hacking** `stabilization_indi.c` to read
`v_thrust.z = thrust->sp.thrust_f[Z]` directly (bypassing the `MAX_PPRZ` scaling
and the `Bwls` multiply, letting WLS/pseudo-inverse map it straight to motors).
That corrupts the generic INDI stabilizer for every non-MFC user (see bug-035/037).

## What changed
1. **`stabilization_indi.c`** — reverted to **stock**: restored
   `v_thrust.z += th_cmd.z * Bwls[3][i] * (int32_t) act_thrust_mat[2][i];` in the
   `INDI_OUTPUTS == 4` branch; removed the hack + its debug comments.
2. **`stabilization_mfc.c`** — `MFC_OUTPUTS == 4` branch now reads
   `v_thrust.z = (float)thrust->sp.thrust_f[THRUST_AXIS_Z];` directly (consumes
   the raw physical value guidance produces, fed to the pseudo-inverse). This is
   the form the INDI hack used; the MFC stabilizer is where it belongs.
3. **`guidance_mfc.c`** — added one selector define + a scale:
   - `GUIDANCE_MFC_THRUST_TO_PPRZ` (default `FALSE`): emit raw physical float via
     `th_sp_from_thrust_f` → for `stabilization_mfc`'s pseudo-inverse.
   - `TRUE`: `thrust_pprz = (int32_t)(thrust_cmd * GUIDANCE_MFC_THRUST_PPRZ_SCALE)`,
     `Bound(0, MAX_PPRZ)`, emit via `th_sp_from_thrust_i` → for **stock** INDI's
     Bwls path.
   - `GUIDANCE_MFC_THRUST_PPRZ_SCALE` (default `1.f`) = `1 / sum(Bwls[3][i])`.
4. **`conf/modules/guidance_mfc.xml`** — documented both defines.
5. **`anton_mfc.xml`** — MFC->INDI stack: `THRUST_TO_PPRZ = TRUE`,
   `THRUST_PPRZ_SCALE = (1000./(4.*-1.5))` = **-166.67**.

## The scale, derived
INDI G1 thrust row = `{-1.5,-1.5,-1.5,-1.5}`, `INDI_G_SCALING = 1000`
(`stabilization_indi.c:1114` `g1g2[i][j] = g1[i][j]/INDI_G_SCALING`).
So `Bwls[3][i] = -0.0015`, `sum over 4 thrusters = -0.006`,
`scale = 1/-0.006 = -166.67`.
Round-trip is exact: guidance `thrust_pprz = thrust_cmd * -166.67`; stock INDI
`indi_v[3] = thrust_pprz * sum(Bwls[3][i]) = thrust_cmd`. The negative scale also
flips the negative physical command to a positive paparazzi command (~2833 at
hover, matching 0.30*9600).

## Verification
- `./build_fw.sh -c ANTON_MFC conf/airframes/ENAC/conf_enac.xml nps` → **clean
  `nps.elf`** (guidance=mfc + stab=indi, THRUST_TO_PPRZ=TRUE).
- `stabilization_mfc.c` edit confirmed to **parse/compile** (compiler reached the
  function body, only a benign "th_cmd set but not used" warning in OUTPUTS==4).
- NOT yet flight/sim-validated for hover equivalence — next step: run ANTON_MFC
  in NPS, command altitude hold, confirm `indi/thrust_cmd` and motor commands
  match the pre-revert behavior; tune `THRUST_PPRZ_SCALE` if needed.

## Gotchas / out of scope
- **bug-039 (pre-existing):** the `stabilization type="mfc"` stack does NOT
  compile — `wls_alloc.h`/`stabilization_mfc.c` include `stabilization_indi.h`
  whose `extern float g1g2[INDI_OUTPUTS][INDI_NUM_ACT]` needs INDI macros absent
  in an MFC-only build. Left unfixed (surgical scope). The MFC stabilizer change
  was validated by direct object compile, not a full stack build.
- **PPRZ-mode horizontal caveat:** in `THRUST_TO_PPRZ` mode `mfc_thrust_sp` is
  INT-format, so `accel_to_att_sp()`'s read of `sp.thrust_f[Z]` is invalid. Inert
  while gx/gy are disabled (TODO noted in code).
- `build_fw.sh` CONF_XML is relative to `paparazzi/`, not `/workspace`.

## Next
- NPS hover validation + scale tuning for ANTON_MFC.
- If a stab=mfc / MFC->MFC stack is ever needed, fix bug-039 first (decouple the
  `g1g2` extern in `stabilization_indi.h` from the MFC build).
