# 2026-08-30 — Flat-trajectory multi-table registry and selection (Phase B)

Phase B of `ThAIsis/Plans/sitl-flat-traj-comparison-dispatch.md`: make
`nav_flat_traj` carry several generated tables and select among them, from the
`sim_anton.py` command line and as GCS buttons, with one mechanism.

Scope was registry + selection only. The playback path
(`guidance_h_set_flat()` / `guidance_v_set_flat()` / `guidance_flat_nominal_set()`)
was NOT touched, so nothing entered the blast radius of
[[17 - The Flat Reference Fixed-Point Ratchet]] or
[[14 - Flat Nominal Inputs Plumbing Decision]].

## What was built

`sw/airborne/modules/nav/nav_flat_traj.{c,h}`

- `struct FlatTrajEntry { name, samples, nb_samples, dt_ms, duration_ms, order }`,
  every field taken from the per-table macros the generator emits.
- `flat_traj_registry[FLAT_TRAJ_NB]`, one row per compiled-in table, built through a
  `FLAT_TRAJ_ENTRY(PREFIX, ARRAY, NAME)` macro so adding a table is copy header,
  `#include`, add row.
- `enum FlatTrajId { FLAT_TRAJ_MINSNAP, FLAT_TRAJ_CIRCLE4, FLAT_TRAJ_LOOP_UPRIGHT,
  FLAT_TRAJ_LOOP_ROLL, FLAT_TRAJ_NB }` in the header, so the flight plan names
  trajectories rather than integers.
- `nav_flat_traj_select(id)` sets a PENDING index (`nav_flat_traj_sel`);
  `nav_flat_traj_start()` latches it into `flat_traj_active`. Selection therefore takes
  effect at the next block entry, never mid-playback.
- `nav_flat_traj_run()` indexes `idx = t * 1000 / tr->dt_ms` against the active entry
  and passes `tr->order` through. Everything downstream of that is unchanged.

## Why the selection is latched, not live

Each table has its own `dt_ms` and `nb_samples`. Reading the selection per tick would let
a GCS write swap the table under a running elapsed-time index — a step on the position
setpoint, which is precisely the class of thing note 17 says a controller that
differentiates its own setpoint cannot survive. Latching costs nothing and removes the
case.

## Where the tables live, and why

Tracked in-repo, `sw/airborne/modules/nav/flat_traj_<name>_data.h`. Four reasons:

1. `flat_traj_demo_data.h` is already git-tracked in that directory — this is the repo's
   established handling of these generated files, not a new policy.
2. The generator stamps `@file "modules/nav/flat_traj_<name>_data.h"` into each banner and
   takes a `ModulePath` argument for exactly this destination.
3. The generator is in a different repo (`Generic_Quad`) and needs MATLAB. Generating into
   the build tree would make `paparazzi_dev` un-buildable on its own.
4. A firmware commit then reproduces a specific flown reference, which is the point of the
   cross-source comparison.

Cost: ~5.6 MB of C text in git. That is the honest price of (3) and (4).

`flat_traj_demo_data.h` is superseded by `flat_traj_minsnap_data.h` (same manoeuvre,
regenerated 2026-08-30 with `HeadingCoeffs`). It is left on disk and no longer `#include`d
— NOT deleted. Note it is the older generator vintage and has no `FLATTRAJSAMPLE_DEFINED`
guard, so it cannot be added back alongside the others without being included first.

## Selection surface — one mechanism, both halves

A flight-plan `<block>` IS the GCS strip button AND the string `sim_anton.py --nav` takes.
So one block per trajectory covers both requirements, and no CLI flag was added.

`conf/flight_plans/ENAC/flat_traj_demo.xml`, exact `--nav` strings:

| block name | strip button | table |
|---|---|---|
| `Flat minsnap` | Flat minsnap | minsnap |
| `Flat circle4` | Flat circle4 | circle4 |
| `Flat loop_upright` | Flat loop_up | loop_upright |
| `Flat loop_roll` | Flat loop_roll | loop_roll |
| `Flat_Traj_Demo` | Run Flat Traj | whatever `flat_traj_sel` holds |

Each new block is `<exception>` geofence, then `nav_flat_traj_select()`, then
`nav_flat_traj_start()`, then `nav_flat_traj_run()`. **Call order is load bearing** —
select sets the pending index, start latches it; reversed, the block plays the previous
table.

`Flat_Traj_Demo` is kept under its historical name and does not call the selector, so it
plays whatever `flat_traj_sel` holds. That is the settings-driven fallback route.

Block names carry the qsim run stem because the stem is the cross-source join key.

## The settings fallback and what it needed

`conf/modules/nav_flat_traj.xml` gained a `flat_traj_sel` `dl_setting`
(`values="minsnap|circle4|loop_upright|loop_roll"`). Two things this taught:

- `<settings>` must come BEFORE `<dep>`. `module.dtd` fixes the order
  `(doc, settings_file*, settings*, dep?, header?, init*, ...)`; after `<dep>` gives
  `DTD prove error: Unexpected tag : 'SETTINGS'`.
- The setting is invisible until the module XML is listed in the aircraft's
  `settings_modules=`. Added to the five aircraft on this flight plan only (ANTON_MFC,
  ANTON_HEOL, ANTON_FINDI, ANTON_FMFC, Hoops_111_MFC) — adding it to an aircraft that does
  not compile the module would be an undefined symbol.

It now generates as `case 48: nav_flat_traj_sel = _value;` in ANTON_MFC's `settings.h`.
The end-to-end `--set` route was NOT exercised: `sim_anton.py` applies `--set` only after
the whole `--nav` sequence, so a setting cannot be sequenced before a block entry from the
CLI. From the GCS a human can (click the setting, then the button).

## Size — measured, not estimated

`size -A var/aircrafts/ANTON_MFC/nps/simsitl`:

| | `.rodata` |
|---|---|
| before (one table compiled in) | 218,148 B |
| after (four tables) | 1,128,804 B |

Per table, from `nm --print-size`:

| table | samples | dt | bytes |
|---|---|---|---|
| circle4 | 3858 | 1 ms | 493,824 |
| loop_roll | 1798 | 1 ms | 230,144 |
| minsnap | 1501 | 2 ms | 192,128 |
| loop_upright | 1458 | 2 ms | 186,624 |
| **total** | 8615 | | **1,102,720** |

128 B per sample (32 floats). **The 5.6 MB C-text figure overestimates the binary cost by
about 5x** — this is why the plan said measure rather than estimate.

For context and NOT as a measurement: Tawaki 1.0 is STM32F767 with 2 MB of flash
(`STM32F76xxI.ld`, `flash0 len = 2M`), so the four tables would be ~53% of it. The `ap`
target was not built or measured this session, so whether the whole `ap` image fits is
still open.

Conclusion: no build-flag subset is needed for SITL, and one was deliberately not designed.

## SITL verification

`timeout 55 ./sim.sh ANTON_MFC --no-build --nav "Start Engine,Takeoff,+15,<block>"`,
reading `/uav/SP/guidance/{h_n,h_e,h_heading,v_z}` out of `sim_logs/*.csv`.

| block | setpoint observed | matches the table |
|---|---|---|
| `Flat minsnap` | (0,0,-2) to (1,1,-3) NED, heading 0 to 44.9 deg | yes, banner says (1,1,-1) offset, 45 deg |
| `Flat circle4` | circle, fitted radius 1.498 m, heading swept 359.7 deg, z flat | yes, 4 m/s R 1.5 m |
| `Flat loop_upright` | vertical loop in the E-D plane, E span 1.99 m, 2 m of altitude, heading constant, 2.60 s of motion | yes, R 1 m at 2.5 m/s |
| `Flat loop_roll` | same 1 m loop geometry, 1.55 s of motion | yes, R 1 m at 4.5 m/s |

The two loops have identical position references by construction; they differ in the
attitude columns, and the tables confirm it: `phi_ref` peaks at 0.860 rad (49.3 deg) for
upright and 3.1376 rad (179.8 deg) for roll.

Each block jumped to a distinct block id and produced a distinct, spec-matching reference.
Selection works from the command line, and the same blocks are the GCS strip buttons.

## Observed, not investigated (all out of Phase B's scope)

- **ANTON_MFC does not TRACK either loop.** Setpoints played correctly; truth roll peaked
  at 24 deg on `loop_roll` and 18 deg on `loop_upright`, and the aircraft returned to its
  hover altitude. This is a controller/guidance question, deliberately untouched. It does
  say the roll loop's open flyability question is not answered by this work.
- **`heading` wraps at +/- pi mid-circle**, exactly where the plan predicted
  (`guidance_h_set_flat()` applies `FLOAT_ANGLE_NORMALIZE()`, the generator does not wrap).
  Whether anything downstream reconstructs a heading rate by differencing `sp.heading` was
  not checked.
- **`server` prints "live md5 signature for 218 does not match current configuration" on
  every ALIVE.** Present in every run this session, including one taken before the settings
  change, so the new setting is not the cause. Mechanism not established. It did not stop
  block jumps, settings generation, or the runs.
- **`conf/flight_plans/ENAC/flat_traj_demo.xml` already contained `--` inside an XML
  comment** before this session (it fails `xml.etree` parsing at HEAD; paparazzi's own
  xml-light tolerates it). Fixed while editing the file; the file now parses strictly.
- **`WORKSPACE_DIR` must be the paparazzi_dev root** for `pprz.sh` / `sim.sh` — a session
  launched at the vault root inherits the vault path and the container cannot find
  `./pprz.sh`.

## Files touched

- `sw/airborne/modules/nav/nav_flat_traj.c` / `.h` — registry, selector, enum.
- `sw/airborne/modules/nav/flat_traj_{minsnap,circle4,loop_upright,loop_roll}_data.h` — new,
  generated, copied verbatim (md5-checked) from
  `Generic_Quad/scripts/trajectories/build/`.
- `conf/modules/nav_flat_traj.xml` — doc + `flat_traj_sel` setting.
- `conf/flight_plans/ENAC/flat_traj_demo.xml` — four new blocks.
- `conf/userconf/ENAC/conf_mfc.xml` — `settings_modules` for the five affected aircraft.

Not touched: `flat_traj_demo_data.h`, the generator, guidance, any controller.
