# 2026-08-28 — Recovering mfcdata (it was never actually lost)

## What this was supposed to be

`tools/mfcdata/` — the converter that ingests Paparazzi SD flight logs and SITL
CSVs into the qsim effort-store format with provenance sidecars — was down to
eight `__pycache__/*.cpython-314.pyc` files and no `.py`. `git log --
tools/mfcdata` returned nothing, so the package looked never-tracked and
unrecoverable. The plan was to decompile all eight modules by hand from
disassembly and verify behaviour against the surviving bytecode.

**None of that was necessary.** The source was intact in git the whole time.

## The actual finding

`git log -- <path>` searches **only the current branch**. The package was
committed on 2026-08-16 as `cf38361`, on a local branch `mfcdata-ingest` that
was never merged into `mfc-development` and never pushed to origin. The task
note in `ThAIsis/Tasks/archive/mfcdata-ingest-package.md` even named the commit
and the branch — "done and committed (`cf38361`, branch `mfcdata-ingest`)". The
record was correct; nobody had a reason to distrust the `git log` that appeared
to contradict it.

Two things kept the loss invisible for twelve days:

1. **`git log` without `--all` is branch-local.** A branch is not history you
   can see from where you are standing. `git log --all -- <path>`,
   `git log --all --oneline | grep`, and `git reflog --all | grep` each found it
   in seconds.
2. **`.gitignore` carries `**/__pycache__/`.** Once the `.py` files were gone,
   the directory's only remaining content was ignored — so `git status` never
   reported `tools/mfcdata` as untracked *at all*. The directory was invisible
   to the tool that exists to make untracked work visible.

The second is the more interesting one. The usual safety net for "you have
untracked work" is `git status`, and it stayed silent precisely because the
thing that survived was the thing we ignore.

## Proving the recovery rather than trusting it

A decompilation would have needed behavioural testing to argue it was
*equivalent*. A git recovery can make a much stronger claim, so it was worth
spending the time to establish it: compiling each of the eight recovered modules
under this interpreter (3.14.4, magic `2b0e0d0a`) reproduces the corresponding
`.pyc` **exactly** — every opcode, constant, name, varname, freevar, cellvar,
flag and argument count, recursively through all 43 nested code objects.

Not "behaves the same". The same bytecode. The recovered source *is* the source
that produced the artefacts on disk.

The end-to-end check was run anyway, because a proof about code objects is not a
proof about the pipeline around them. Converting `fr_0003`'s 25 MB `.data` twice
— once with the recovered source, once with the surviving bytecode loaded as
sourceless modules from `x.pyc` — gives:

- byte-identical 165 MB CSVs, md5 `618e43e6f8a86cd433baeccb938d2a47`
- `meta.json` identical except `converted_at`, which is wall-clock

and both reproduce the metadata committed on 2026-08-16 exactly: 88548 samples,
440.116 Hz, 9 aliases applied, `HOOPS_111_MFC`, ac_id 177.

## The second loss, which mattered more

`contract/signals.json` — the vendored role contract — landed in the *same*
unmerged commit, and was therefore also missing. This is easy to miss because
the symptom is different: `check` and `verify` could not run **on this branch at
all**, whoever's source was present. `contract.contract_path()` walks up looking
for it and raises `FileNotFoundError`. The package was broken here in a second,
independent way that no amount of decompilation would have fixed.

With it restored, `verify` passes all six axes on both sources:

| source | branch | max \|derived − logged\| |
|---|---|---|
| flight (fr_0003) | MFC_STAB φ/θ/ψ | 1e-06 |
| flight (fr_0003) | MFC_GUIDANCE x/y/z | 0 |
| sitl | MFC_STAB φ/θ/ψ | 5e-06 |
| sitl | MFC_GUIDANCE x/y/z | 1e-03 / 7.8e-04 / 1e-05 |

against `TOL = 0.01` of the logged error's own range.

## Checked against the design intent

`ThAIsis/Tasks/archive/mfcdata-ingest-package.md` flagged the SD-path field
aliasing as **a correctness bug rather than a naming issue** — the alias carries
fixed-point scale factors and an ENU→NED vertical negation, so skipping it
leaves position in raw int32 counts and altitude inverted, "a plot that looks
entirely plausible while being upside down."

The recovered source implements this correctly. `sd.convert` calls
`apply_aliases`; `alias_columns()` reads the scales and the negation **off
`FIELD_ALIAS`** rather than restating them — with a comment saying that
duplicating them to save an import is how a sign error gets introduced. Aliases
only ever add columns and never overwrite an existing one, matching the live
relay's rule. 9 alias columns applied on fr_0003.

Also as specified: manual time anchoring only (`--t0`, no auto-detect heuristic);
sidecar provenance with nothing defaulted (an unestablished fact is an *absent
key*, and `firmware_commit` is never guessed from the working tree); stdlib only;
two checks rather than one.

**One spec item is still unmet.** The doc asked for "one time-column name and one
lead-column convention — the two converters disagree today, which blocks any
join." They still disagree: `sd.TIME_COL = "Time"`, `sim.TIME_COL = "time"`.
`sim.times_of` works around it by accepting `time`/`Time`/`t`, and each sidecar
records its own `time_column`, so it is survivable, but the join key is not
unified. The doc's own Updates section spun remaining converter work out to the
time-alignment task, so this is probably deliberate deferral rather than an
oversight — worth confirming before the Data/ consolidation work leans on it.

## Latent bug found while verifying

`python3 -m mfcdata check` printed **nothing** and exited 0. `check.run()`'s
no-`messages.xml` branch returns before the `if verbose:` print block at the end
of the function, and `__main__` discards the report. Since `messages.xml` is
build output, a fresh clone always takes that path — so the common case was a
silent exit 0, indistinguishable from a check that ran and found nothing wrong.

Fixed in a **separate commit**, deliberately, so the byte-identity evidence for
the restore stands on an untouched tree. The fix is scoped so that only
`check.run`'s code object differs from the 2026-08-16 bytecode; the other seven
modules and check's five other code objects still match exactly.

## Exposure audit

- `tools/mfcdata` was the **only** sourceless package in the repo and the **only**
  untracked tool. Every other script at the repo root and in `tools/` is tracked.
- One other file exists solely on an unmerged local branch:
  `Knowledge/Daily Notes/2026-06-17.md` on `daily-notes`.
- Five branches carry unmerged commits (`arm64-build-system`, `daily-notes`,
  `mfc-riachy-trick`, `mfcdata-ingest`, `outer-worktree-nav-sweep`), one commit
  each.
- **origin carries only `main`.** All seventeen local branches are unpushed. The
  condition that made this possible has not gone away; it has only lost its
  sharpest instance.

## What is next

- Push the branches, or at least stop treating a local topic branch as a durable
  record. A task note saying "done and committed" is not a backup.
- Decide whether the `Time`/`time` split gets unified before the Data/
  consolidation work, or stays deferred to time-alignment.
## Postscript: merged, and `check` finally ran

Fast-forwarded onto `mfc-development` (exactly the two commits; nothing on
mfc-development that wasn't already on the branch). The main checkout has
`paparazzi/var/messages.xml` — build output, which is why the worktree did not —
so `mfcdata check` ran against real upstream truth for the first time in this
recovery, and passes clean:

```
contract  v3 (2026-08-16)
messages  paparazzi/var/messages.xml
OK      MFC_STAB     <- STAB_MFC      (19 fields bound)
OK      MFC_GUIDANCE <- GUIDANCE_MFC  (18 fields bound)
0 error(s), 0 warning(s)
```

So both halves of the two-check design are now exercised on this branch: `check`
against messages.xml + BRANCH_MAP, `verify` against the logged `err_*` on both a
flight and a SITL source. The contract has not drifted from the firmware in the
twelve days it spent stranded.

Worth noting the stale `tools/mfcdata/__pycache__` from the incident is still
present and is *not* shadowing anything — Python's source-based invalidation
compares the recorded mtime/size against the restored `.py`, recompiles, and
`__file__` confirms the `.py` files are what load. It is gitignored and
regenerable; left in place rather than deleted, since it is the only physical
trace of the incident.
