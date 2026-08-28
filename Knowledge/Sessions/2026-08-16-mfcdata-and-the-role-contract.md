# 2026-08-16 — mfcdata, and two binding errors the schema check could not see

Built the firmware half of a shared run-record contract: `tools/mfcdata/`, a
vendored `contract/signals.json`, and two checks. The checks found two real
binding errors, both of which would have produced plausible and wrong plots.

## What was built

`tools/mfcdata/` — stdlib only, matching the rest of this repo's tooling.

```
python3 -m tools.mfcdata check                 # contract vs messages.xml
python3 -m tools.mfcdata verify RUN.csv        # bindings vs the logged err_*
python3 -m tools.mfcdata sd  FLIGHT.data -o <effort>/data/<stem>.csv
python3 -m tools.mfcdata sim CAPTURE.csv -o <effort>/data/<stem>.csv
```

Each converter writes `<stem>.csv` plus `<stem>.meta.json`. The stem is the join
key shared with the MATLAB side; nothing downstream re-reads raw bytes.

`contract/signals.json` is the role vocabulary — `ref_cmd`, `ref`, `y`, `err`,
`fk`, `u`, `u_total` — and its per-source bindings. JSON rather than YAML because
it is machine-read from two runtimes and both parse JSON with nothing installed;
this repo has no pyyaml and MATLAB has no dependable YAML reader.

## The two findings

**1. Stabilization and guidance score against different setpoints.**
`MFC_STAB` forms its error against `sp_traj_<axis>`. `MFC_GUIDANCE` forms its
against `sp_<axis>`. The branches look structurally identical — same six field
prefixes, same layout — and are not.

Evidence, from the logged `err_*` which the firmware emits alongside the signals
it is computed from:

| branch | source | `me - sp_traj` | `me - sp` |
|---|---|---|---|
| MFC_STAB | flight fr_0003 | **0.000000** | out by up to 194% of range |
| MFC_GUIDANCE | SITL | out by 10.97 | **1e-5** |

**2. A manual flight cannot settle a guidance binding.** On fr_0003 guidance was
not driving, so `sp` equals `sp_traj` exactly and *both* candidate bindings
verify clean. Only the SITL run disambiguates. Verify against every source, not
the convenient one.

## Why the schema check was not enough

`check` compares the contract against `messages.xml` and `BRANCH_MAP`. It catches
renamed messages, dropped fields, and fields nobody bound. It **cannot** catch a
binding that points at a real-but-wrong field — every name resolves, every plot
renders, and the numbers are wrong. Tested explicitly: reverting the `sp_traj`
fix produces zero errors from `check`.

`verify` closes that. The logged `err_*` is redundant with `y - ref_cmd`, and
that redundancy is the only independent evidence available offline. It found both
errors on its first run.

Two checks, two different jobs. Neither substitutes for the other.

## Also fixed

- **`apply_field_aliases` now runs on the SD path.** `sdlog2scope` imports
  `BRANCH_MAP` but never called the alias step, so offline flight files carried
  raw `ROTORCRAFT_FP/*` while SITL carried `EST/*`. The alias also applies
  fixed-point scales and an ENU→NED negation, so skipping it left position in
  int32 counts and altitude inverted. `mfcdata sd` applies it (9 columns).
- **Full-rate conversion.** fr_0003 from `.data` directly gives 88548 samples at
  440 Hz. The committed `_pj.csv` is 794 at 4 Hz — a ground-station export
  wearing the filename a full-rate conversion produces.
- **Provenance capture.** The sidecar records aircraft, ac_id, wall-clock start,
  measured rate, maneuver, and `--set` gain overrides. Note the `.log` names the
  aircraft as `<airframe NAME="...">` — uppercase attribute, no `<aircraft>`
  element — and `ac_id` is not in the header at all, only in the `.data` records.

## Not done

- `firmware_commit` is never guessed from the working tree. The card was written
  by whatever was flashed at the time, which today's checkout does not know.
  Pass `--commit` or the run honestly records none.
- Time alignment across sources. Manual `--t0` only, recorded in the sidecar; no
  auto-detection, because no heuristic survives a crash, an aborted run and a nav
  flight alike, and a wrong anchor is invisible in the output.
- The remaining flight logs and the ~90 legacy `.jsonl` captures are unconverted.
