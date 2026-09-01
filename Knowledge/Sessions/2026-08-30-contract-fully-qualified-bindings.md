# Fully-qualified contract bindings, and the SITL 6-DOF branch

2026-08-30. Contract v3 -> v4.

## The blocker

`qsim.metrics.tracking_6dof` needs `pos`, `sp`, `pos_meas`, `euler` and `u`
from one SITL capture. Those live under four unrelated first path segments in
the NPS scope stream: `TRUTH` (simulator fdm), `SP` (guidance references),
`EST` (state estimate) and `WLS_U` (allocator).

Both resolvers build the column prefix from the BRANCH:

- `tools/mfcdata/contract.py`, `column()` — `"/{}/{}/{}".format(COLUMN_ROOT, branch, field)`
- `qsim/+qsim/contract.m`, `i_columns()` — `"/uav/" + branch + "/" + names`

So every role in a branch inherits ONE first segment. No branch name can
express four, and a new SOURCE does not help because the prefix comes from the
branch, not the source.

`column_format` in `signals.json` looks like it configures this. It does not —
**grep finds zero readers**. It is documentation, and it was inaccurate
documentation, which cost a previous session a false lead.

## The fix

A binding that already starts with `/` is FULLY QUALIFIED and used verbatim.
Added to both runtimes in the same change; patching one alone is exactly the
drift the vendored contract exists to prevent.

Behaviour preservation was checked before applying, not assumed: all 22
bindings in v3 were enumerated (list entries counted individually) and none
starts with `/`. The escape hatch is a strict addition. Re-checked after
patching: all 74 resolved columns across `sitl`/`flight` x `MFC_STAB`/
`MFC_GUIDANCE` still come out as `/uav/<branch>/<field>`.

## The branch

`SITL_6DOF_ANTON_MFC`, under `sources.sitl.branches`. Carries **no `message`
key** on purpose — it is a composite of scope state, not a telemetry message,
and `contract.firmware_branches()` only walks branches that have one, so
`mfcdata check` correctly skips it. It binds no `err`, so `mfcdata verify`
reports "nothing to verify against" rather than failing.

All 19 resolved columns verified present in
`sim_logs/mfc_sim_20260830_182223.csv`.

Frames: NED throughout, z DOWN, no negation. `TRUTH/*` is `fdm.ltpprz_pos`
(`struct NedCoor_d`); `EST/*` and `SP/guidance/*` are NED. The ENU->NED
negation in `pj_json_relay`'s `FIELD_ALIAS` is on `ROTORCRAFT_FP`, and the
only ENU scope source is `SP/nav/*`, which this branch does not bind. On this
capture a climb drives `sp` and `TRUTH` z both to -3.

## `u` — airframe-dependent, and in pprz units

`/uav/WLS_U/u/u_` is registered by `oneloop_mfc.c` (ANTON_MFC);
`/uav/FLAT/alloc/u/u_` by `oneloop_findi.c` and `oneloop_fmfc.c`. It cannot be
one shared binding — hence one branch per airframe. Only an ANTON_MFC capture
exists today, so only that branch is written.

`u` IS bound. The role is semantically exact (both Simulink `alloc_u` and
firmware `WLS_U/u/u_` are the pre-clamp allocator output) and
`recipes/tracking_6dof.m` asks for `u` before falling back to `u_total`.

But the values are PPRZ actuator units, 0..4082 observed, not the normalised
[0,1] of Simulink. So `u_rms`, `max_abs_u` and `du_rms` are a within-firmware
magnitude only, and **`u_sat_frac` is invalid and must not be reported** —
`mean(u < 0 | u > 1)` returns about 1.0 for any flying run. The contract note
`u_is_airframe_dependent_and_in_pprz_units` says so explicitly.

## Left alone deliberately

`mfcdata verify --source sitl` still FAILS on MFC_GUIDANCE x/y/z (0.188,
0.188, 0.0249). The error is formed against `sp_traj_<axis>`, not `sp_<axis>`,
contradicting the dated "VERIFIED 2026-08-16" note
`branches_use_different_references`. That is a real and separate problem, and
changing `ref_cmd` would alter what every existing MFC_GUIDANCE run means.
**Author's call. Not touched.**
