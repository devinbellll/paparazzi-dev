# 2026-08-22 — `oneloop_fmfc`: the MIMO HEOL/MFC brackets on the FINDI spine

**Scope: INTEGRATION ONLY.** Nothing here is tuned and no number below is a
performance result. The Simulink reference `fmfc_quad` is explicitly not
fine-tuned, so any tracking figure measured from this build would be measuring
an untuned controller. The gains ship as labelled placeholders, swappable as an
airframe edit.

Branch `fmfc-quad-integration-20260822` (outer and submodule), based on
`mfc-development` at outer `b81adf8` / submodule `cd0ba036f`.

## What was built

`oneloop_fmfc` — the whole FINDI spine (`oneloop_findi.c`, flown as ANTON_FINDI)
with **only the two increments** replaced by 3-channel HEOL/MFC brackets:

| | FINDI | FMFC |
|---|---|---|
| linear | `f_i^c = f_i^prev + m (a_c - a_tilde)` | `f_i^c = f_i* + m(-F_fi - a_c)` |
| angular | `m_c = m^prev + I (dw_c - dw_lpf)` | `m_c = m* + I(-F_m + dw_c)` |

Everything else — applied reactions through the same `G1`, the flatness force
transform, the quaternion attitude error, the tilt limiter, the allocation
pseudo-inverse, the guidance-latch dispatch — is carried over verbatim.

New files:

- `sw/airborne/firmwares/rotorcraft/oneloop/oneloop_fmfc_law.h` — the pure
  bracket, host-includable.
- `sw/airborne/firmwares/rotorcraft/oneloop/oneloop_fmfc.{c,h}` — the module.
- `conf/modules/oneloop_fmfc.xml`
- `conf/airframes/ENAC/quadrotor/anton_fmfc.xml`
- `tests/oneloop_fmfc_test.c`, `tests/run_oneloop_fmfc.sh`,
  `tests/stubs_clock/mcu_periph/sys_time.h`
- `conf/userconf/ENAC/conf_mfc.xml` — ANTON_FMFC, ac_id **223** (purely
  additive; a parallel worker was adding DARKO entries to the same file).

`mfc_core_mimo.{c,h}`, `mfc_core.{c,h}` and `heol_mimo.{c,h}` are **untouched**.

## The decision that shaped the port: where `f_b` enters the core

`mfc_core_mimo`'s feedback has exactly three lanes — `kp*e`, `kd*e_dot`,
`ki*int(e)` — all formed from its own error. **Neither FMFC bracket's `f_b`
fits.**

- The linear bracket's `a_c = kx^2 e_p + 2 zeta_x kx e_v + ka e_a` has three
  terms of three different derivative orders. The core has no acceleration lane.
- The angular bracket's `f_b = -dw_c` is built from the **raw** attitude error
  and the **raw** body rate, while the *same bracket's estimator drive* is the
  **filtered** attitude error. Feedback and estimator drive are different
  signals there, by specification.

Two options, and the one not taken matters:

1. **Add a feedforward input to `mfc_core_mimo`.** Rejected — it edits the
   shared core that the flying 2-vector horizontal channel links, for the
   benefit of a new module.
2. **Chosen:** the caller forms `f_b` in full and hands it to the core through
   its single derivative lane with `kd = 1`, `kp = ki = 0`,
   `use_external_derivative = true`, `deriv_filter = 0`. The core's `fb` term is
   then *exactly* `f_b`, with no gain of the wrapper's own in the path, and the
   two roles (estimator drive `eps`, feedback `f_b`) stay independent — which is
   what the specification actually describes.

`kd = 1` is a **structural pass-through, not a damping gain**. `kx`, `zeta_x`,
`ka`, `k_att`, `k_rate` remain independent module scalars, so a GA can still
search them.

## Why `oneloop_fmfc_law.h` and not `heol_mimo`

`heol_mimo` is the same structure and the new bracket deliberately mirrors it.
It was not reused because `HEOL_MIMO_N` is a compile-time **2** baked into its
struct *and* into the signatures of `heol_mimo_set_alpha()` / `heol_mimo_run()`,
and its only caller is flying. Widening it to a runtime width would edit a
flying 2-vector path to serve a new 3-vector one. The width difference lives
entirely in the new wrapper, which the flying stack never links.

## Two conventions preserved, deliberately unharmonised

1. **The linear loop's errors are `measurement - reference`** — the MFC
   convention, the *opposite* of the FINDI cascade sitting in the next file.
2. **No `dw_ref` term inside `dw_c`.** The feedforward enters through
   `m* = I dw_ref` and nowhere else.

Both are checked by observable consequence rather than by reading the code back
(checks D2/D3 and D4).

## alpha: constant, diagonal, plant algebra

`alpha_fi = (1/m) I`, `alpha_m = J^-1`, both set once and re-scheduled each tick
only to honour `mfc_mimo_set_alpha()`'s once-per-tick contract literally. The
flatness map is downstream of the loop rather than inverted inside the
estimator, so there is no Jacobian, no conditioning question, and
`heol_input_sensitivity` has nothing to do here. No `alpha` define exists in the
airframe and none is wanted.

## Estimator parameters — taken, not derived

From `flat_mfc_quad_params.m`, via the airframe:

```
FI_INTEGRATION_WINDOW = 500     FI_EST_HOLD_TIME = 0.5
M_INTEGRATION_WINDOW  = 20      M_EST_HOLD_TIME  = 0.5
```

500 on the force bracket against 20 on the moment bracket is the same
position-loop-vs-attitude-loop split the HEOL channels use. These are **not**
inherited from the HEOL horizontal channel; they only happen to agree on the
force one.

`est_use_presat_command` is set **explicitly to `false`** on both brackets at
the call site in `oneloop_fmfc_init()`. There is deliberately **no airframe
define** for it, and `oneloop_fmfc.c` carries an `#error` that fires if one
appears — a module-XML define shadowing the core default is what hid the HEOL
divergence defect for three days (`cfede3078`).

## `u_prev`: the estimator sees the *measured applied* correction

Spec part 3.2 writes the estimator's previous applied correction as
`f_i_prev - f_i*` and `m_prev - m*` — the measured applied wrench minus the
nominal, not the core's own last output. It also matches open question **Q3**,
which records that the `Memory` on `u_prev` inside both reference HEOL blocks is
commented *through*, i.e. no unit delay.

Both point the same way, so `use_applied_u_prev` defaults **true** and the
applied correction is written into `command_est` **before** `mfc_mimo_run()`
(the core reads it forming `alpha @ d2u` and rewrites it in its history shift,
so a write before the call is consumed that same tick, with no added delay). The
measured wrench is already one actuator-lag behind the command, so there is no
algebraic loop.

**Q3 is not author-confirmed.** The switch exists so the choice is named and
defaulted rather than silent; setting it OFF falls back to the core's own
delayed command, which is what every other MFC channel in this tree does.

## Verification

### Property checks — 78, all pass

`./tests/run_oneloop_fmfc.sh`. There are no golden traces and none are coming
(no MATLAB here); the checks were decided **up front**, before the port. Six
groups:

- **A (16)** — bracket algebra. `f_f` grounded at zero; `f_b` arriving with no
  wrapper gain; the **inverse** multiplying (`du = -m f_b`, not `-f_b/m`); the
  nominal added strictly outside; the HEOL invariant that the delayed-command
  lane carries `du` alone and never `u* + du`; the rails clamping the correction
  and the presat/postsat taps differing when a rail bites.
- **B (22)** — core configuration. Width 3, decoupled, `kd = 1 / kp = ki = 0`,
  channel 2 actually live with no cross-talk, estimator inert until
  `est_hold_time` and live after, **per-element numerator over a scalar shared
  denominator**, and `mfc_mimo_reset()` clearing `setpoint_trajec[]`.
- **C (5)** — the flying 2-vector caller. A 300-step `heol_mimo` trajectory with
  the horizontal channel's anti-diagonal `alpha` is **bit-identical** whether or
  not a 3-vector bracket runs interleaved against the same core. The runner also
  asserts `git diff --quiet cd0ba036f` over the six shared core/heol files
  before it compiles anything, so the claim cannot rot.
- **D (15)** — conventions. Hover identity `f_i^c == [0,0,-m g]` exactly; a
  north position error commanding a **southward** force (the error-sense check);
  `d(m_c)/d(dw_ref) == I` exactly *once* (the missing-`dw_ref` check — a second
  occurrence would show as a clean factor of two); the trimmed angular bracket
  commanding zero; and the `-H(z) zeta_e` negation on the estimator drive.
- **E (13)** — `alpha` constant, diagonal, and inverted the right way round.
  `alpha_m^-1 == J`, not `J^-1`: a swap there is a factor of ~2.2e4 on this
  airframe and would read as a wild gain rather than as a bug.
- **F (5)** — `float_mat_inv_4d`'s 0-is-success convention and its
  undivided-adjugate failure path, asserted rather than remembered.

### Build

Clean for **both** targets, no warnings from any new file. The only warning in
either build is the pre-existing generated
`flight_plan.h: 'nav_hold_alt' defined but not used`, which every aircraft on
this flight plan emits.

```
nps: var/aircrafts/ANTON_FMFC/nps/obj/nps.elf
ap : var/aircrafts/ANTON_FMFC/ap/obj/ap.elf     587896 text / 11796 data / 508560 bss
```

### SITL smoke test — qualitative only

`timeout 80 ./sim.sh ANTON_FMFC --no-build --nav "Start Engine,Takeoff,+20,Standby,+30"`,
capture `sim_logs/mfc_sim_20260822_134153.csv`, 38434 rows over ~77 s.

It **arms, takes off, and holds.** Judged strictly as a divergence check:

- altitude settles on the -2 m reference and stays inside [-2.009, -1.993] m for
  the last 40 s;
- horizontal position wanders inside ±0.28 m with no growth;
- `alloc_ok` and `mode/guidance` are 1 throughout; attitude error is ~2e-3 rad;
- motors sit at ~1880 of 9600 — nowhere near a rail;
- **both estimators settle and stay bounded.** `F_fi_2` converges to 1.486 and
  drifts 0.002 over 40 s; `F_m_*` stay inside ±1 rad/s^2; **no correction rail
  is touched in any of the 38434 samples** on any channel.

**No tracking figure is quoted as a result, and none should be.** The gains are
untuned placeholders, so a good number would be luck and a bad one would be
uninformative.

### The finding worth keeping: the bracket identity closes *in flight*

The settled hover numbers reproduce the bracket's own algebra to four decimals,
which is an in-flight confirmation of the "nominal outside the correction"
structure that no offline check can give:

```
f_i*            = -m g          = -7.8480 N
F_fi_2 (settled)                =  1.4865 m/s^2
-m * F_fi_2                     = -1.1892 N   ==  du_fi_2 = -1.1899 N
f_i* + du_fi_2                  = -9.0379 N   ==  f       = -9.0381 N
```

And the *reason* the correction is non-zero is itself informative: the
airframe's `MASS = 0.8 kg` is a `TODO: measure` placeholder taken from the
JSBSim model, so the plant is ~15 % heavier than the nominal the loop inverts.
The estimator absorbs the whole mismatch as a steady 1.49 m/s^2 and hover holds
regardless. That is exactly what the ultra-local model is for, and it is a
useful sanity signal: had the estimator been mis-signed, this is the term that
would have run away instead of settling.

## Carried-forward facts, re-checked rather than trusted

- `float_mat_inv_4d()` returns **0 for success, 1 for failure**, and on failure
  leaves the **undivided adjugate** behind. Both asserted in group F. The row
  normalisation in `fmfc_calc_pinv()` is what keeps the guard from firing at
  all, and it cancels exactly (`pinv(G) = pinv(Gn) D`).
- The `mfc_siso_reset()` hole — `setpoint_trajec[]` never cleared, so a stale
  setpoint leaked into the first `ddot_sp` on re-entry (fixed 2026-08-20) —
  **does not exist in the MIMO reset path.** `mfc_mimo_reset()` clears the whole
  capacity of `setpoint_trajec`, `error`, `z`, `estimator_num` and
  `estimator_den`. Verified directly (checks B6/B7), not inferred.

## Harness gotcha worth remembering

`pprz_docker.sh` bind-mounts `$WORKSPACE_DIR` at `/workspace`. When a session is
launched from the **vault root**, that variable points at the vault, not at this
repo, and every in-container path resolves one level too high — the symptom is
`cc1: fatal error: <every source file>: No such file or directory`.
`tests/run_oneloop_fmfc.sh` pins `WORKSPACE_DIR` to the repo root itself; other
callers need `WORKSPACE_DIR="$PWD" ./pprz.sh ...`.

Also: `tests/stubs/` cannot be used for anything needing the real pprz algebra —
it shadows `math/pprz_algebra_float.h` as well as the clock. `tests/stubs_clock/`
is new and replaces the clock only.

## Not done, deliberately

- **No tuning of any kind**, and no `Flat_Traj_Demo` run. Both belong to a
  tuning session against this structure, not to an integration one.
- No noise-axis pair (noise-off / noise-on). That is a rung-7 question and the
  ladder has not been climbed past hover here.
- The allocation pseudo-inverse and the flatness transform are not re-checked in
  `tests/oneloop_fmfc_test.c` — they are carried over verbatim from the flown
  FINDI spine and are covered by its own harness. This file is scoped to the
  delta.

## Next

1. Ask the author about **Q3** (the missing unit delay on `u_prev`) and about
   whether `use_applied_u_prev` should stay defaulted ON.
2. Bench-measure `MASS` and the inertias. They are load-bearing twice over here
   — the nominal plant *and* both `alpha` matrices — and the current values are
   JSBSim placeholders. The 1.49 m/s^2 steady estimate above is the size of the
   error they currently paper over.
3. Climb the ladder: vertical alone, then horizontal, then `Flat_Traj_Demo`,
   then the noise sources one at a time. Only then tune.
