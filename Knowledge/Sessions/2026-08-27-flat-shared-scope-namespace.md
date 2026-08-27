# 2026-08-27 — a shared `FLAT/` NPS scope namespace for FINDI and FMFC

Worked directly on `mfc-development` in the main working tree (no branch, no
worktree — the author asked for it there), starting from outer `ac95edb` /
submodule `71c995b0e`.

## The problem

`oneloop_findi.c` and `oneloop_fmfc.c` registered two parallel scope trees that
were suffix-for-suffix identical for everything the controllers share:
`FINDI/lin/a_c_x` and `FMFC/lin/a_c_x` are the same quantity under two names.
`plotjuggler_flatness.xml` therefore carried every shared curve twice — 116
curves, of which only one prefix could ever populate, since only one of the two
modules links into a given aircraft.

## What changed

**One namespace for the shared flatness spine, one for what only FMFC has.**

| Prefix | Count | Registered by | Content |
|---|---|---|---|
| `FLAT/` | 45 | BOTH modules, identical names | applied reactions through `G1`, the flatness force transform, the quaternion attitude error, the tilt limiter, the allocation, the mode/health flags |
| `FMFC/` | 26 | `oneloop_fmfc.c` only | `lin/fi_star_*`, `ang/m_star_*`, the whole `est/` tree |

45 + 26 = 71 = FMFC's previous total; FINDI's previous 45 mapped entirely onto
the shared set, with nothing left over. The partition was verified against the
source before applying it and matched the proposal exactly.

`plotjuggler_flatness.xml` keeps its 9 tabs / 30 plots and its one-unit-per-plot
discipline (including the two earlier deviations: the estimator needs two tabs
because the force and moment brackets disagree on units for all four
quantities, and `alloc/v` splits because `v = [Mx, My, Mz, Fz]` mixes N*m and
N), but now carries **71 curves instead of 116**, each quantity exactly once.
The header was rewritten, not restarted: everything the previous pass got right
(the SITL-only warning, the `flat_status` enum sense with 0 as the good value,
the partial angular instrumentation, `flat/f` negative in hover, the estimator
unit table, the `fi_c = fi_star + du_fi` bracket identity with the numbers from
the 2026-08-22 integration run) was carried over.

## The judgement call that was honoured, not fixed

**`ang/dw_c_y` is shared in name but not in composition.**

* FINDI: `dw_c = k_rate (w_c - w) + dw_ref` — the feedforward is inside `dw_c`.
* FMFC: `dw_c = k_rate (w_c - w)` — no `dw_ref`, because the feedforward enters
  through `m* = I dw_ref` instead. Adding it back would double count it.

Same quantity (commanded angular acceleration) reached by two different laws,
and only one controller runs per session, so sharing the name is right. Neither
law was touched. Documented at both registration sites and in the layout header.

## A second such pair, found while verifying: `lin/a_c_*`

Not in the brief; found by reading the two linear halves side by side.

* FINDI: `a_c = a_ref + (kp e_p + kv e_v + ka e_a)` with **reference minus
  measurement** errors — a commanded acceleration that carries the feedforward.
* FMFC: `a_c = kp e_p + kv e_v + ka e_a` with **measurement minus reference**
  errors and no `a_ref` — it is the linear bracket's `f_b` drive, with the
  nominal carried separately by `fi_star`.

So for the same position error the two `a_c` point **opposite ways**, and they
differ by the feedforward on top of that. Both senses are deliberate and each
module's source says so in as many words. Treated exactly like `dw_c`: shared
name, documented difference, no law changed.

A third, milder one: `mode/guidance` is 1 on both when the guidance latch is
fresh and the linear half is enabled, but FMFC additionally requires
`motors_on` before its linear bracket actually runs, so on FMFC a 1 with motors
off means the attitude-only branch ran and the bracket was held in reset.

`lin/fi_prev_*` / `ang/m_prev_*` were diffed line by line and are composed
identically in both modules (same `G1_SI * H(z) u` reconstruction, same
`R_be^T` rotation of the collective) — a genuine shared quantity with no caveat.

## Deliberately out of scope: the Darko modules

`oneloop_findi_darko.c` and `oneloop_fmfc_darko.c` keep `FINDID/` and `FMFCD/`
and were not touched. **This is a decision, not an oversight.** Their spine is
genuinely a different one: `darko_wrench` instead of a `G1` matrix multiply,
sequential allocation instead of a 4x4 inverse, the AERO frame, four actuators
of two kinds. Folding them into `FLAT/` would assert a commonality that does
not hold. Both still build (see below).

No control-law change of any kind was made. This was a rename plus a layout
rewrite: no gains, no structure, no new signals.

## Verification

Builds (all with `WORKSPACE_DIR` pinned to `paparazzi_dev/`, bug-293; a
module-set change needs the clean rebuild):

| Aircraft | Target | Result |
|---|---|---|
| ANTON_FINDI | nps (clean rebuild) | `nps/simsitl` linked |
| ANTON_FINDI | ap | `ap/obj/ap.elf` linked |
| ANTON_FMFC | nps (clean rebuild) | `nps/simsitl` linked |
| ANTON_FMFC | ap | `ap/obj/ap.elf` linked |
| DARKO_FINDI | nps (clean rebuild) | `nps/simsitl` linked — untouched modules still build |
| DARKO_FMFC | nps (clean rebuild) | `nps/simsitl` linked — untouched modules still build |

The only compiler warning on any of the six is the pre-existing
`'nav_hold_alt' defined but not used` out of ANTON's generated `flight_plan.h`;
the only make warnings are the bind-mount clock-skew ones. No changed file
produces a warning. Note `pprz.sh`'s `==> Done: .../nps/obj/nps.elf` line names
a path that never exists for the nps target (bug-301, renumbered this session);
the real artifact is `var/aircrafts/<AC>/nps/simsitl`.

SITL captures, 60 s each, `--nav "Start Engine,Takeoff,+15,Standby"`, exit 124
as expected:

* ANTON_FINDI → `sim_logs/mfc_sim_20260827_200330.csv`
* ANTON_FMFC → `sim_logs/mfc_sim_20260827_200439.csv`

**The check that matters — set comparison of the layout's `<curve name>` values
against each capture's CSV header, in both directions:**

| Capture | `FLAT/` cols | `FMFC/` cols | layout curves absent from capture | captured series unplotted |
|---|---|---|---|---|
| ANTON_FINDI | 45 | 0 | none | none |
| ANTON_FMFC | 45 | 26 | none | none |

On the FINDI capture the layout's 26 `FMFC/` curves have no column, which is
the by-design emptiness the header describes, not a mismatch.

`grep -rn 'FINDI/' --include=*.c --include=*.xml .` now returns exactly one
hit: the words "NOT FINDI/" inside the new explanatory comment. No `FINDI/`
series name survives anywhere; the Darko files' `FINDID/` is untouched.

## Bookkeeping

`.wolf/buglog.json` had a duplicate `bug-300` — the FMFC merge session used it
for the embedded-repo/worktree gitdir issue, and the plotjuggler session reused
it for the `pprz.sh` done-line issue. The **later** one (the `pprz.sh`
`nps/obj/nps.elf` entry) is now `bug-301`. The 9 older duplicates (051, 052,
123, 124, 204, 205, 272, 273, 274) predate all of this and were left alone.

## Not verified

* **PlotJuggler still cannot run in this sandbox** (no apt candidate, no pip
  package), so no claim here about the file's LOAD behaviour is an observation.
  What is verified is the name set, against the two captures. Whether
  PlotJuggler ignores or complains about the 26 absent `FMFC/` curves on a
  FINDI run is the same open question the previous pass left, unchanged — but
  the number of possibly-absent curves dropped from 71 of 116 to 26 of 71.
* Nothing about control performance. No number in the captures was read for
  tracking quality; the runs exist to produce column headers.
