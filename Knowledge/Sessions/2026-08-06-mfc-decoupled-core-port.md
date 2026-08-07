# 2026-08-06 — MFC decoupled core port (HEOL stage 1)

Executed [[MFC Decoupled Core Port (HEOL Stage 1)]] in full. Stage 1 of getting
ANTON flying HEOL control with a flatness feedforward trajectory.

## What changed

### `mfc_core` — the actual change

`struct MfcParameters` gained `decoupled` (uint8), `z[3]`, `int_err`, `ki`,
`est_hold_time`; `use_Kd` is deleted. `kp`/`kd` are retyped as the raw
coefficients of the closed-loop polynomial `s² + kd·s + kp` — **not** `wn`/`zeta`.

`mfc_siso_run()` now selects the estimator structure per axis:

| | **coupled** (`decoupled = FALSE`, default) | **decoupled** (`TRUE`) |
|---|---|---|
| estimator driven by | tracking error → `z[0] = error[0]` | measurement → `z[0] = measure` |
| folded coefficients | `a = −kd`, `b = −kp` | `a = b = 0` |
| what `F̂` means | plant dynamics **plus** the closed-loop polynomial | the plant's own lumped disturbance (physical, m/s²) |
| feedback `fb` | `ki·∫e` only | `kd·ė + kp·e + ki·∫e` |

Command law: `u = (−F_k + ẍ_ref − fb) / alpha`, with
`error = measure − setpoint_trajec`. The estimator numerator kernel and both
`(W² + 2W + 1)` IIR smoothers were left untouched — they already match the
upstream Simulink model character-for-character; only the estimator's *input*
changed. `est_hold_time` replaces the hardcoded `0.1f` blanking threshold.
Integral uses a trapezoidal rule with anti-windup: the candidate is rolled back
in any sample where the `u_min`/`u_max` clamp bites.

The `decoupled` flag gates **both** the drive signal and the folding together.
Splitting them would apply `kp` twice — once inside `F̂`, once explicitly — and
run the loop at double proportional gain with nothing erroring.

### Gain conversion (complete, all four consumers)

`kp_new = kp_old²`; `kd_new = 2·kd_old·kp_old` (`use_Kd=TRUE`) or `2·kp_old`
(FALSE/absent). Applied at every site in the plan's table:
`stabilization_mfc.c`, `guidance_mfc.c`, `oneloop_mfc.c`, the hardcoded
`guidance_indi.c` thrust literal, and `anton_mfc.xml`, `hoops_111_mfc.xml`,
`anton_oneloop.xml`, `anton_mfc_thrust.xml`. `grep -rn use_Kd paparazzi/`
returns zero.

### Module XML wiring

`stabilization_mfc.xml` and `guidance_mfc.xml` gained per-axis
`*_DECOUPLED` / `*_INTEGRAL_GAIN`(`*_KI`) / `*_EST_HOLD_TIME` defines and
matching persistent `dl_setting`s (`values="COUPLED|DECOUPLED"`, `type="uint8"`).
The `KP`/`KD` `description=` strings — which read *"natural frequency wn=kp"* and
*"Damping ratio (zeta), used only when …_USE_KD=TRUE"* — were rewritten to
*"closed-loop polynomial coefficient: s² + kd·s + kp"*, since they had become
actively misleading. Ranges widened to `kp,kd ∈ [0,200]`, `ki ∈ [0,50]`: under
the new convention the pitch axis alone needs `kd = 80`, well past the old
`[0,10]` bound. `oneloop_mfc.xml` got range widening and `use_Kd` removal only,
per the scope decision — no new settings.

`anton_mfc.xml` and `hoops_111_mfc.xml` carry the three new defines explicitly at
their defaults, so the structure choice is visible in the airframe.

## Verification

**Compile — all five targets link:**

```
ANTON_MFC ap · ANTON_MFC nps · Hoops_111_MFC ap · ANTON_ONELOOP ap · ANTON ap
```

**Behaviour preservation, proven rather than argued.** `mfc_core.c` has only two
external dependencies (`get_sys_time_float`, `float_vect_zero`), so the version
at `HEAD` and the new one were both compiled standalone with trivial stubs and
driven with an identical 4000-sample multi-tone signal — old at the old roll
tuning (`kp=6, kd=1.5, use_Kd=TRUE`), new at the converted one (`kp=36, kd=18`,
`decoupled=FALSE`, `ki=0`). Output is **byte-identical** on both `command[0]` and
`estimator` across all 4000 samples. The conversion table and the coupled path
are correct. Decoupled mode was separately run with `ki=0.5` and is bounded with
no NaN/Inf.

There is no `gcc` in the sbx sandbox — the harness runs inside
`paparazzi-build:latest` with its directory mounted. This is a cheap, repeatable
check worth reusing for any future `mfc_core` change.

## Not done — needs the user

- **Golden-trace comparison against the model** (plan §Verification 2). The
  traces live outside this mount, in the MFC_SISO model repo at `tests/golden/`
  (`2nd_decoupled_alg.csv`, `2nd_coupled_alg.csv`). Copy them to
  `tools/mfc_golden/` and the harness above extends to replay them directly.
- **SITL coupled regression and the decoupled A/B** (plan §Verification 3–4).
  Both need a live GCS session. The decoupled A/B is the one that matters: flip
  `mfc_gx.decoupled` / `mfc_gy.decoupled` to DECOUPLED, enter the model's tuning,
  and watch the x/y command oscillation that motivated this task. The real test
  is that `mfc_gx.estimator` — now a physical disturbance in m/s² — should
  **not move when `kp`/`kd` are retuned**. If it does, the estimator is still
  error-driven somewhere.

## Known bug, deliberately left

Logged as `bug-247` in `.wolf/buglog.json`: `oneloop_mfc.c:1420-1421` sets
`mfc_gz.u_max = 0.f` and `u_min = MAX_PPRZ / GUIDANCE_MFC_THRUST_PPRZ_SCALE` =
**+9600** at the default scale, so `u_min > u_max` and the clamp order pins the
gz command to +9600 every tick. Real, but oneloop-only (`guidance_mfc.c:334-335`
is correct) and ANTON_MFC does not use oneloop — fixing it changes
ANTON_ONELOOP flight behaviour, a separate decision. The new anti-windup makes it
marginally worse: that axis now also freezes its integral permanently, harmless
only while `ki = 0` (the default).

## Next

Stage 2 — HEOL as its own library (`control/heol.c/.h` + `conf/modules/heol.xml`
with `<depends>mfc_core</depends>`), **not** by widening `mfc_core`. HEOL's
estimator is fed `ε` and `u_fb` (the feedback component only, not total `u`),
which is exactly what `mfc_core`'s `d2u` term gives when `mfc_core` *is* the
`u_fb` block. Drive a decoupled `MfcParameters` with `setpoint = 0`,
`measure = ε`. Stage 1's core needs no further change to support it.

## See also

- [[MFC Decoupled Core Port (HEOL Stage 1)]] — the plan executed here
- [[Flatness Trajectory Setpoints (Pos-Vel-Accel-Jerk-Snap + Psi)]] — the setpoint path this builds on
- [[08 - NPS Simulation Telemetry]] — logging `mfc_gx.estimator` for the A/B
