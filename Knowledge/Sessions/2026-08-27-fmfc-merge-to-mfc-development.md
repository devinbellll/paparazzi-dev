# 2026-08-27 — Merging both FMFC branches into `mfc-development`

Scope: a MERGE, not a change. No tuning, no gain edits, no structural change to
either controller, to the FINDI spines, to `mfc_core_mimo` or to `heol_mimo`.
The only content edits are three conflict resolutions and nine comment
punctuation fixes.

Worked in a dedicated worktree at `.wt-fmfc-merge/` (added to
`.git/info/exclude`). The main tree was left on `fmfc-darko-20260822`,
untouched.

## Result

| | outer `paparazzi_dev` | submodule `paparazzi` |
|---|---|---|
| base = `mfc-development` | `b81adf8` | `cd0ba036f` |
| quad (`fmfc-quad-integration-20260822`) | `caab384` | `f8c1dab32` |
| darko (`fmfc-darko-20260822`) | `120cdd7` | `ebe639fb0` |
| **merge commit** | **`26a7f9a`** | **`03b32ddbc`** |
| XML-comment fix | `68d6636` | `71c995b0e` |

`mfc-development` tips: outer `68d6636`, submodule `71c995b0e`. The outer
gitlink points at the submodule's own `mfc-development` tip. Neither source
branch was deleted; nothing was pushed.

## The merge

Both branches were exactly one commit off a shared base, and the quad branch
fast-forwarded in both repos. Only the darko merge conflicted.

**Submodule — `conf/userconf/ENAC/conf_mfc.xml`.** Both sides append an 11-line
`<aircraft>` element immediately before `</conf>` and change nothing else. Kept
both, ANTON_FMFC (223) then DARKO_FMFC (29). Nothing collides: different names,
ac_ids and `gui_color`. Diffed the result against each source branch: exactly
the other branch's block added, nothing else.

**Outer — `.wolf/anatomy.md`.** Took the quad side wholesale. An older task note
claimed the quad branch carried a duplicate Darko section; it does not. Against
the merge base the darko file adds a 12-line Darko FMFC section and the quad
file adds that same section PLUS a 10-line ANTON FMFC section — the quad file is
a byte-exact superset of the darko file and contains the Darko section once.

**Outer — `.wolf/buglog.json`.** Two pure appends over an identical 292-entry
base, both `version: 1`. Concatenated the added blocks textually: quad's
`bug-293..295`, then darko's `bug-296..297`. 297 entries in id order, no id
collision. The 9 duplicate ids that predate both branches were left alone.

**Outer — the `paparazzi` gitlink.** Set to the submodule merge.

The two changesets are disjoint over every controller source, module XML and
airframe XML. Confirmed post-merge that the six shared
`mfc_core*` / `heol_mimo*` files are byte-identical to `cd0ba036f`, so
`run_oneloop_fmfc.sh`'s pre-compile guard survives the merge untouched — and it
passed.

## XML comments

A `--` inside `<!-- ... -->` is not well-formed XML. Nine occurrences, all
prose dashes, in the two files the merge brought in:
`conf/airframes/ENAC/quadrotor/anton_fmfc.xml` (7, in the comments beginning at
lines 53/181/290) and `conf/modules/oneloop_fmfc_darko.xml` (2, at lines 51/83).
Replaced with a colon or comma. With all comments stripped both files are
byte-identical to their previous revision, and both now parse (they did not
before). Committed separately from the merge.

**Not fixed, and not in scope:** a sweep of every XML under `paparazzi/conf`
finds 11 further comments carrying `--` in files neither branch touched —
`airframes/ENAC/hybrid/darko.xml`, `airframes/ENAC/quadrotor/anton_findi.xml`
(3), `anton_heol.xml` (2), `flight_plans/ENAC/flat_traj_demo.xml`,
`flight_plans/OPENUAS/openuas_versatile_unified.xml`, `joystick/nes_gamepad.xml`,
`modules/guidance_rotorcraft.xml`, `modules/oneloop_findi_darko.xml` (2),
`radios/UCM/T16SZ_SBUS_rover.xml` (3). All predate both branches. Fixing them
here would have turned a merge into a nine-file sweep; left for the author.

## Verification

Host property checks, on the merged tree:

| suite | result |
|---|---|
| `tests/run_oneloop_fmfc.sh` | shared-core guard PASS, **78 checks, all pass** |
| `tests/run_oneloop_fmfc_darko.sh` | **109 checks, all pass** |
| `tests/run_flatness_quad.sh` | all pass (26 PASS lines) |
| `tests/run_flatness_darko.sh` | all pass (53 PASS lines) |

Builds, all exit 0, artifacts produced:

| aircraft | target | command | result |
|---|---|---|---|
| ANTON_FMFC | nps | `rebuild` | `nps/simsitl` |
| ANTON_FMFC | ap | `build` (fresh tree = clean) | `ap/obj/ap.elf` |
| DARKO_FMFC | nps | `rebuild` | `nps/simsitl` |
| DARKO_FMFC | ap | `build` (fresh tree = clean) | `ap/obj/ap.elf` |

No warning cites any FMFC, `mfc_core`, `heol_mimo` or `flatness_` source. The
only compiler warning is `'nav_hold_alt' defined but not used` from ANTON's
generated `flight_plan.h` (a property of `flat_traj_demo.xml`); the only make
warnings are `Clock skew detected` from sub-100 ms mtime granularity on the
bind mount during the OCaml bootstrap.

Fleet parse: `./pprz.sh db ANTON_FMFC` (226 entries) and `./pprz.sh db
DARKO_FMFC` (220 entries), both exit 0 — `conf_mfc.xml` parses for both.

SITL, `--nav "Start Engine,Takeoff,+15,Standby"`, `timeout 60` (exit 124 both,
as expected). **A smoke test, not an evaluation — no figure below is a
performance result.**

- **ANTON_FMFC** (`sim_logs/mfc_sim_20260827_184208.csv`, 28434 rows,
  t 4.45–61.32): motors on t = 6.50 s, in flight t = 7.05 s, climbed to
  z = −2.34 m, held z = −1.995 m with sd 0.001 m over the last 5 s.
  `alloc_ok = 1` throughout, no NaN.
- **DARKO_FMFC** (`sim_logs/mfc_sim_20260827_184336.csv`, 29079 rows,
  t 4.44–61.29): motors on t = 6.47 s, in flight t = 7.00 s, climbed to
  z = −7.50 m, held z = −7.02 m (sd 0.23 m) over the last 5 s. `alloc_ok`
  drops below 1 only for 116 samples at t ≈ 4.44–4.67, all pre-flight; never
  while `in_flight`. Horizontal position wanders ±11 m at `Standby`.

That horizontal wander is **not a merge regression**: the 2026-08-22 darko
session recorded the same picture from the same build (motors on t = 6.5 s,
climb to 7 m, altitude held 6.3–7.5 m, horizontal residual 11.5–15 m rms, tilt
on the 30° limiter 100 % of samples) and attributes it to untuned placeholder
gains and a horizontal loop saturated open. The merged tree reproduces it.

## Not verified

- No flight test. The merge is a source-level claim plus SITL smoke.
- The aerodynamic half of the Darko flatness inversion is still untestable in
  SITL (`TRANSFORM_V_SCALE = 0`, JSBSim `cyclone`) — unchanged by this merge.
- `tests/run_mimo_golden.sh` and `tests/run_oneloop_findi_darko.sh` were not
  run; only the four suites in the table above.
- Nothing was pushed and no branch was deleted.

## Snags

- `tests/run_oneloop_fmfc_darko.sh` does not pin `WORKSPACE_DIR` the way
  `run_oneloop_fmfc.sh` does, so it fails with `cc1: fatal error: ... No such
  file or directory` on every source whenever the session's `WORKSPACE_DIR`
  points anywhere but the repo root (bug-298, same cause as bug-293). Worked
  around with `WORKSPACE_DIR="$PWD"`; the runner was **not** edited, as that is
  outside this merge's scope. The two-line fix is in bug-298.
- `./pprz.sh db` does a clean build and wiped both nps `simsitl` binaries
  (bug-299). Rebuilt them before the SITL runs.
- `paparazzi/` is an old-style embedded repo, so a `git worktree` of it leaves
  every nested `sw/ext/*/.git` relative pointer dangling (bug-300). Rewrote them
  to absolute paths into the main tree's `.git/modules`. Inside the build
  container those host paths do not exist, so git calls in the build print
  `fatal: not a git repository` — cosmetic; every build and codegen completed.

## Next

- Author's call: push, and whether to delete either source branch.
- The 11 pre-existing `--`-in-comment offenders under `conf/`.
- The `WORKSPACE_DIR` pin in `tests/run_oneloop_fmfc_darko.sh`.
- Tuning either FMFC controller. Both still ship dated placeholder gains and
  no number measured from either airframe is a performance result yet.
