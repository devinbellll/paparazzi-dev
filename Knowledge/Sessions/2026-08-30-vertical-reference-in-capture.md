# 2026-08-30 — `SP/guidance/v_ref_z`: the vertical reference in the capture

Follow-up to [[2026-08-30-flat-traj-multi-table-registry]]. Binding the SITL capture to
qsim's `tracking_6dof` roles needs the `sp` (commanded position) role to take all three
axes from the SAME stage of the guidance chain. It could not: horizontal had both the
nav-rate setpoint and the loop-rate reference model, vertical had only the setpoint.

`tracking_6dof` reports PER-AXIS metrics, so binding x/y to `h_ref_*` and z to `v_z` would
have given the vertical axis a nav-rate staircase the horizontal axes do not have, and the
artefact would have read as "the vertical channel tracks worse".

## What was changed — four lines, two files

`sw/airborne/modules/loggers/logger_mfc_csv.c` (on-board logger), after `v_zd`:

```c
  X("/uav/SP/guidance/v_ref_z",      POS_FLOAT_OF_BFP(guidance_v.z_ref))       \
```

`sw/airborne/modules/nps_scope/nps_scope_state.c` (the SITL scope mirror) — mirror field,
assignment, registration, each placed beside its `gv_z_sp` sibling:

```c
  float gv_z_ref;                /* vertical pos reference (NED down m) */
  m.gv_z_ref = POS_FLOAT_OF_BFP(guidance_v.z_ref);
NPS_SCOPE_VAR("SP/guidance/v_ref_z", &m.gv_z_ref, NPS_SCOPE_FLOAT);
```

Nothing in the guidance reference path was touched. `z_sp` is unchanged. No XML touched.

## The scope side DID need wiring — and the trap in the question

The `SP/guidance/` branch had `h_ref_n` and `h_ref_e` but **no vertical reference at all**.

There is a separate, pre-existing `guidance_v/z_ref` scope var
(`guidance_v.c:66` definition, `:74` registration, `:250` assignment inside
`guidance_v_run()`, all `#ifdef SITL`) carrying exactly the same quantity under a
different branch name. So the value was reaching the capture — just not as the sibling of
the horizontal pair, and only in sim. Verified bit-identical: max |v_ref_z − guidance_v/z_ref|
= 0.000e+00 across a whole capture.

**The trap worth recording: the SITL CSV comes from `nps_scope_state.c`, not from
`logger_mfc_csv.c`.** `sim_anton.py` defaults to the in-process NPS scope emitter, and
`logger_mfc_csv` is the on-board logger, loaded by `hoops_111_mfc.xml` ONLY. Adding the
row to the logger alone would have compiled, passed review, and never appeared in
`sim_logs/*.csv`. Both sides need the row for the role binding to hold across sources —
and compile-checking the logger means building **Hoops_111_MFC**, not ANTON_MFC.

Both built clean: `ANTON_MFC nps` (scope) and `Hoops_111_MFC nps` (logger).

## Measured, on a fresh `Flat minsnap` capture

`sim_logs/mfc_sim_20260830_182223.csv`,
`--nav "Start Engine,Takeoff,+15,Flat minsnap"`. Manoeuvre window taken as first-to-last
change of `h_ref_n`: 23.053 to 25.503 s (2.450 s), 1226 scope samples = **500.4 Hz**.

| column | span [m] | distinct values | changes/s |
|---|---|---|---|
| `SP/guidance/v_z` (nav-rate setpoint) | 0.9961 | 39 | **15.5** |
| `SP/guidance/v_ref_z` (new) | 0.9961 | 256 | **104.5** |
| `SP/guidance/h_n` (nav-rate setpoint) | 0.9961 | 39 | 15.5 |
| `SP/guidance/h_ref_n` (existing) | 0.9961 | 256 | 104.9 |
| `guidance_v/z_ref` (pre-existing) | 0.9961 | 256 | 104.5 |

Same range, 6.7x the update rate — and the new vertical pair is now numerically
indistinguishable from the horizontal pair. That is the asymmetry closed.

## Why 104 Hz and not 500 Hz — do not read this as a defect

`guidance_v.z_ref` is **Q23.8**, LSB = 1/256 = 3.906 mm. Over a 1 m climb in 2.45 s a
Q23.8 value can take at most 256 distinct values, i.e.

    256 LSB / 2.450 s = 104.1 values/s

The measurement is 104.5. **The rate metric is saturated on the fixed-point LSB, not on
the loop rate**, and `h_ref_n` saturates at the identical ceiling for the identical reason
(it is Q23.8 too). The scope samples at 500 Hz and the reference model runs at 500 Hz;
the signal simply has no finer level to move to.

The consequence for anyone measuring this again: compare the distinct-level COUNT
(39 vs 256) rather than treating changes-per-second as the update rate. A slower or
shorter manoeuvre will show a proportionally lower changes/s for both references without
anything being wrong.

That LSB is the same one [[17 - The Flat Reference Fixed-Point Ratchet]] is about — the
ratchet lived in exactly this value. It is live and correct here: `v_ref_z` runs from
−2.008 to −3.000 m monotonically with no sawtooth, which is what the fixed version should
look like.

## Files touched

- `sw/airborne/modules/loggers/logger_mfc_csv.c` (+1)
- `sw/airborne/modules/nps_scope/nps_scope_state.c` (+3)

Not touched: `guidance_v.c`/`.h`, `z_sp`, any module or airframe XML.
