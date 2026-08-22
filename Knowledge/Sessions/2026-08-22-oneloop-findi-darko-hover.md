# 2026-08-22 — `oneloop_findi_darko` stage 2: the controller, and a stable hover

## Task

Build the FINDI flatness controller for the DarkO tailsitter as a new oneloop
module and get a **stable hover in SITL**. Hover only — no transition, no
`AP_MODE_FORWARD`, no trajectory tracking.

Stage 1 (the spine: `flatness_darko.{c,h}`, three pure functions, 53 property
checks) was already committed at `473c1501d` and was **inherited, not rebuilt**.
Re-ran its harness first thing this session: all 53 pass.

## Verdict, up front

**Hover is stable — with a named diagnostic engaged, and not otherwise.**

- The controller as ported, and the ladder below it, is sound: 55 new analytic
  property checks pass, the module builds clean for `nps` and for `ap`, and it
  holds the Standby waypoint to **0.098 m rms with perfect sensors and 0.265 m
  rms with the stock noise**, with altitude flat to ±0.002 m / ±0.020 m.
  That is roughly **35x tighter than the stock-INDI baseline** on the same
  plant, same flight plan, same sensors, back to back.
- **BUT** that result is from a build with `TRANSFORM_V_SCALE = 0`, which
  removes the wing from the flatness force transform and reduces it to pure
  thrust vectoring. **That is not the published law.** With the published law
  (`= 1`) the aircraft diverges to >114 m within a minute, **with the noise
  sources zeroed as well as with them on** — so the divergence is structural.
- The cause is a **plant mismatch, not a port bug**: the NPS/JSBSim aircraft
  this airframe simulates is `cyclone`, which is *not* DarkO. Numbers in
  "Finding 1" below.

So: the port is done and correct, hover is demonstrated, and the honest caveat
is that the SITL plant cannot exercise the aerodynamic half of the flatness
inversion. Nothing here says the published law fails on the real DarkO — it says
this simulator cannot test it.

## What was built

New files:

- `sw/airborne/firmwares/rotorcraft/oneloop/oneloop_findi_darko_law.h` — the
  PURE half: hover/aero frame maps, eq. (tsPD), the moment increment, the
  actuator→command maps, the tilt limiter. Split out so a host harness can
  include it without `state.h` / `generated/airframe.h`.
- `sw/airborne/firmwares/rotorcraft/oneloop/oneloop_findi_darko.{c,h}` — the
  module: measurement, applied wrench, outer loop, angular half, allocation,
  commit, and the oneloop framework dispatch (guidance hooks latch only).
- `conf/modules/oneloop_findi_darko.xml`
- `conf/airframes/ENAC/hybrid/darko_findi.xml` — airframe twin of `darko.xml`,
  identical outside the control stack so the comparison is controlled.
- `tests/oneloop_findi_darko_test.c` + `tests/run_oneloop_findi_darko.sh` — 55
  analytic property checks.
- `DARKO_FINDI` (ac_id 19) registered in `conf/userconf/ENAC/conf_mfc.xml`.

Modified: `conf/airframes/ENAC/hybrid/darko.xml` — added the named NPS noise
switch (`NPS_SENSORS_PARAMS` → `nps_sensors_params_anton_mfc.h`,
`NPS_NOISE_SCALE = 1.`), which is byte-identical to the stock defaults it
replaces. Added only so the baseline could be flown as a no-noise/with-noise
pair against `DARKO_FINDI`. No other change to the baseline.

Followed `oneloop_findi.c` (the quad port) for SHAPE. No gain, no allocation and
no axis sense was carried across.

## The three port decisions that were not obvious

### 1. The angular half runs in the AERO frame, and the axis correspondence is inverted from what "yaw onto differential thrust" suggests

`flatness_darko.*` works in the paper's AERO frame (thrust along body +x,
hover pitch `+pi/2`). The firmware's state, setpoints and actuators are all in
this tree's HOVER frame (thrust along body −z, hover pitch 0). They differ by
`R_h_i = Ry(-pi/2) R_b_i`, so as a component map

    v_hover = [ v_aero_z ; v_aero_y ; -v_aero_x ]

and therefore

| hover axis | aero axis | actuator |
|---|---|---|
| ROLL (x)  | YAW (z)      | differential thrust |
| PITCH (y) | PITCH (y)    | common-mode flap |
| YAW (z)   | −ROLL (x)    | differential flap |

eq. (37)'s "yaw onto differential thrust" is, to a firmware that thinks in
hover axes, **roll onto differential thrust**. Getting this transposed rotates
roll into yaw and yaw into −roll: a plausible-looking wrong answer, not a crash.
`check_axis_correspondence()` drives it through the real allocator and asserts
which actuators move.

**Rotor 1 is the LEFT motor.** Established twice independently: the spine's
`m_z = lTy (T1 - T2)` with aero-z == hover-roll puts rotor 1 at `y = -lTy`, and
`darko.xml`'s flight-proven stock `G1_ROLL = {0, 0, -15, +15}` for
`[ele_l, ele_r, RM, LM]` says positive roll comes from the left motor.

### 2. The elevons are mounted mirrored, so the pprz mapping carries the mirror, not the law

`darko.xml`: ELEVON_LEFT "min vers le haut", ELEVON_RIGHT "min vers le bas".
Equal-sign pprz is equal-and-*opposite* physical deflection. The spine's model
is the other way round — common-mode `d` makes pitch, differential `d` makes
yaw. So the mapping is

    ele_left_cmd = +kf d1,    ele_right_cmd = -kf d2

**One sign choice makes BOTH plant axes agree**, which is what makes it more
than a guess: with it the model's pitch and yaw both come out with the NPS
plant's sign; flip either and exactly one inverts. Checked against the plant's
own coefficients in `check_elevon_sign()`.

### 3. Two calibration constants, set by SLOPE not by value

The allocator emits SI (rad/s, rad); the airframe takes a normalised command.
`W_FULL_CMD` and `D_FULL_CMD` are the bridge, and they are **the only two
numbers in the stack calibrated against the simulator rather than the
aircraft** — flagged as such in three places, and they must be re-measured
before hardware.

What matters to an INDI loop is the slope `d(wrench)/d(command)`, not the
absolute value: a value offset is absorbed because the increment integrates on
the command, whereas a slope error *is* the loop gain. Both were set to match
the NPS plant's slope at hover:

- `W_FULL_CMD = 1300 rad/s` — NPS motor makes 2.2 lbf = 9.786 N at full
  command; `dT/dcmd = 2 cT w K_w` matches that at the hover speed
  `w = 692.45 rad/s`. Hover lands at 0.53 command, mid-range.
- `D_FULL_CMD = 0.638 rad` (36.5 deg) — makes the model's flap pitch moment
  equal the plant's elevon pitch moment at hover: **−0.017732 vs −0.017724 N·m**
  at `d = 0.1 rad`, measured this session. Yaw then comes out 1.36x the
  plant's; one constant cannot match both axes, and pitch was chosen because
  that is where the unmodelled disturbance lives.

## Finding 1 (the big one) — the SITL plant is not DarkO, and the flatness transform is the one part no increment protects

`darko.xml` (and therefore `darko_findi.xml`) simulates
`conf/simulator/jsbsim/aircraft/cyclone.xml` — Ewoud Smeur's Cyclone, not
DarkO. Read out of that file and its `Systems/aerodynamics_cyfoam.xml` this
session:

| quantity | DarkO (`flatness_darko.h`, measured) | NPS `cyclone` | ratio |
|---|---|---|---|
| mass | 0.492 kg | 2.4 lb = 1.089 kg | 2.21x |
| J hover-roll | 0.00606 | 0.0132 slug·ft² = 0.0179 | 2.95x |
| J hover-pitch | 0.002785 | 0.00250 slug·ft² = 0.00339 | 1.22x |
| J hover-yaw | 0.007018 | 0.0150 slug·ft² = 0.0203 | 2.90x |
| motor y-arm | `lTy` = 0.155 m | 9.4 in = 0.2388 m | 1.54x |
| **wing lift at 5 m/s** | `cLV·\|v\|·v` = **8.96 N** | qbar·Sw·CLmax = **0.74 N** | **12x** |

The mass and inertia mismatches an INDI increment absorbs — they are loop-gain
errors, and the loop is sluggish rather than unstable because of them. **The
wing does not get absorbed.** The force transform is an *open-loop aerodynamic
inversion*: it decides how much of the commanded force the wing will make, and
turns the rest into an attitude. Measured offline against the shipped
constants:

```
pure hover force demand, varying forward speed:
  vx = 0.0 m/s   T = 4.920 N   theta_h =   0.00 deg
  vx = 2.5 m/s   T = 4.468 N   theta_h = -24.90 deg
  vx = 5.0 m/s   T = 2.384 N   theta_h = -61.70 deg
  vx = 7.5 m/s   T = 1.272 N   theta_h = -76.54 deg
```

At 5 m/s the transform hands over most of the weight to the wing and commands
62 deg of tilt. On the real DarkO that is right. On `cyclone` the wing delivers
a twelfth of it, the aircraft falls and accelerates, the transform sees more
speed and commands more tilt — **positive feedback, and it is the inversion, not
the feedback loop, that closes it.** Observed: pitch drifts to +69 deg over
~8 s, then the airframe departs and flies 178 m downrange.

Note the transform reads `norm(v)`, so a **vertical climb triggers it too** —
the runaway starts during the takeoff climb, not at some later transition.

The transform itself is correct — checked by direct sweep this session: a
northward force demand gives negative `theta_h` and a thrust vector with
positive north component, and the sweep is symmetric about zero. There is no
sign error anywhere in the inversion.

### The diagnostic, promoted

`GUIDANCE_FINDI_DARKO_TRANSFORM_V_SCALE` (module default **1.0**, the published
law) scales the velocity handed to the **force transform only** — the wrench
model and the allocator still see the true velocity, so the incremental half is
untouched. At `0` the transform reduces to pure thrust vectoring: the attitude
that points the commanded force, no wing lift credited. Hover-only and
plant-agnostic; it cannot transition and it is not the paper's law.

`darko_findi.xml` sets it to `0.` because that is the configuration the
reported hover was flown in. Every number below states which value it used.

## Results — all four runs flown this session

Same flight plan (`rotorcraft_basic`, `Start Engine, Takeoff, +8, Standby,
+40`), same NPS `cyclone` plant, same INS, same STDBY waypoint (NED −5.0, −2.0),
same 40–74 s settled window. `|e_wp|` is the horizontal distance from STDBY.

| run | controller | `tf_v_scale` | noise | `\|e_wp\|` peak | `\|e_wp\|` rms | AGL | roll band | pitch band | yaw band |
|---|---|---|---|---|---|---|---|---|---|
| `baseline_nonoise` | stock INDI | — | off | 4.417 m | 3.881 m | 6.955 ±0.011 m | 7.8 deg | **69.9 deg** | wraps 360 |
| `baseline_noise`   | stock INDI | — | on  | 4.248 m | 3.363 m | 6.929 ±0.043 m | 10.0 deg | **70.3 deg** | 117 deg / 37 s |
| `findi_tf0_nonoise`| FINDI | 0 | off | **0.134 m** | **0.098 m** | 6.962 ±0.002 m | 0.13 deg | 0.12 deg | 0.11 deg |
| `findi_tf0_noise`  | FINDI | 0 | on  | **0.431 m** | **0.265 m** | 6.961 ±0.020 m | 0.42 deg | 0.47 deg | 0.64 deg |

Plus the two divergent runs, kept because they are the evidence for Finding 1:

| run | `tf_v_scale` | noise | outcome |
|---|---|---|---|
| `findi_v1_noise` / `findi_v1b_noise` | 1 | on | diverged, −178.8 m in north, tumbled; the two are bit-identical (determinism confirmed) |
| `findi_tf1_nonoise` | 1 | off | diverged, −114.3 m in north |

Logs: `sim_logs/findi_darko/<tag>.csv` + `<tag>_debug.log`.

**Noise is an axis, and it was tested, not assumed.** The FINDI divergence
reproduces with every stochastic source zeroed, so it is structural. The
baseline's pitch limit cycle likewise reproduces with noise off.

Peak body rates in the settled FINDI window: p, q < 1.0 deg/s, r < 0.25 deg/s
(with noise); < 0.3 deg/s (without). No limit cycle in any axis.

## The known confound — pitch is NOT attributable

The DarkO baseline under stock INDI has an unresolved **~±35 deg pitch limit
cycle** with peak `q` of **±280 deg/s**, and it reproduced exactly as described
in both baseline runs above. Pitch is also exactly the axis where the
unmodelled `PHI_mv` wing pitching moment lives.

**State it plainly: the pitch comparison in the table above is confounded and
neither controller's pitch number is attributable.** FINDI's 0.47 deg pitch band
against the baseline's 70.3 deg looks like a crushing win and it is not
evidence — the two controllers are not merely tuned differently, they are
inverting different models of the same wrong plant.

**Roll, yaw and altitude carry the verdict**, and they are clean:

- altitude 6.961 m ±0.020 m (FINDI, noise) vs 6.929 m ±0.043 m (baseline)
- roll band 0.42 deg vs 10.0 deg
- yaw 0.64 deg over 34 s vs 117 deg over 34 s (≈3.4 deg/s baseline drift)
- horizontal rms error 0.265 m vs 3.363 m

Did not spend the session diagnosing the baseline limit cycle, per the brief.
Two things noticed in passing and written down, not chased:

- The baseline's `eff_scheduling_cyfoam` module rewrites `G1` at runtime
  (`EFF_SCHED_USE_FUNCTION = TRUE`), so the airframe's static `G1_PITCH` is not
  what flies. Anyone chasing the limit cycle should look there first.
- The static `G1` values are in the right ballpark for roll and thrust against
  the `cyclone` plant (13.6 vs 15.0; 0.94 vs 1.0 in airframe units) but
  over-estimate pitch (1.74 vs 4.0–4.3) and yaw (2.43 vs 3.9). An
  over-estimated `G` under-commands, which is sluggish rather than oscillatory
  — so the limit cycle is probably in the scheduler, not in the static table.

## Bring-up ladder — where each rung landed

1. **Offline property checks** — 55 new, all pass
   (`tests/run_oneloop_findi_darko.sh`), plus the spine's 53 re-run and passing.
   Decided up front, as the brief required, since there is no MATLAB here and
   no golden traces are coming. They caught nothing this session, which is the
   point: it meant the SITL divergence could be attributed to control and not
   to transcription within minutes rather than hours.
2. **Attitude-only** — the `findi_darko_linear_enabled` switch is carried and
   wired, but was **not exercised as a separate rung**: with the datalink RC
   and no stick script, the attitude-only path gets a zero thrust setpoint from
   the latching guidance hooks and simply sits. Rungs 2–3 were effectively
   covered by the fact that the closed-loop runs hold attitude to 0.12 deg. If
   a future session needs the rung properly, drive it with `--rc_script 0`.
3. **Outer loop closed** — done, see the table.
4. **Noise pair** — done, both controllers.

## Gains — what was used and what was not touched

Inner (eq. tsPD): `k_xi = 25`, `k_om = 7` (wn = 5 rad/s, zeta = 0.7), the
measured 2026-08-12 hand sweep, **used unchanged**. They worked first try on
this plant with no retuning, which is mildly surprising given the 2.9x roll and
yaw inertia mismatch — the incremental structure evidently absorbed it.

Outer: `kx = 1.8898`, `zeta_x = 1.1093`, `ka = 0.2062`, the quad GA result the
brief flagged as explicitly untuned. **Also used unchanged** — they were never
retuned this session, because they did not need to be once the transform issue
was isolated. There is no retuning finding to report; the numbers above are
from the carried-over values.

Zero integral action in the outer loop, so the settled position carries a
steady offset: −0.207 m north / +0.055 m east (noise on), −0.035 / +0.090
(noise off). Small enough not to matter here; worth knowing it is a bias, not a
drift.

## What is next

1. **The real question this session could not answer: does the published law
   (`tf_v_scale = 1`) hover on a plant that actually is DarkO?** That needs a
   JSBSim DarkO model matching `flatness_darko.h`'s measured constants — mass
   0.492 kg, the three inertias, `lTy = 0.155`, and above all `cLV = 0.3585`.
   Until that exists, no SITL result speaks to the aerodynamic half of the
   inversion, in either direction.
2. Re-measure `W_FULL_CMD` and `D_FULL_CMD` on the aircraft before any hardware
   flight. They are simulator calibrations today.
3. The outer loop is still untuned in the sense the brief meant (gate 2 is
   open). A GA search would now have a working structure to search against —
   which was the precondition.
4. `findi_darko_linear_enabled` is untested in flight. Exercise it with
   `--rc_script 0` before trusting it as a bring-up tool on hardware.

## Environment notes for the next session

- **`WORKSPACE_DIR` must point at `paparazzi_dev/`, not the vault root.**
  `pprz_docker.sh` bind-mounts `$WORKSPACE_DIR` at `/workspace`; a session
  launched higher up in the tree gets a mount whose paths are all one level
  off, and every script fails with "No such file or directory" on files that
  are plainly there. `export WORKSPACE_DIR=$PWD` from the repo root fixes it.
- **Two concurrent sims corrupt each other's logs.** `sim_anton.py` hard-codes
  the NPS scope port (9871) and the IVY bus (2010), and `sim.sh` runs the
  container with `--network host`. A second sim on the same daemon mixes both
  streams into both CSVs. One log this session was auto-flagged
  `.CONTAMINATED` by another worker's tooling. Check
  `docker ps --filter ancestor=paparazzi-build:latest` is empty before flying,
  and verify afterwards that `<tag>_debug.log` names the JSBSim model you
  expected.
- **`--set` is applied AFTER the whole `--nav` sequence completes**
  (`nav_sequence()` in `sim_anton.py`), so it cannot configure anything for the
  flight itself. Diagnostic switches have to go in the airframe XML and be
  rebuilt.
