# Session — 2026-07-16: MFC SI-unit refactor (Hoops_111_MFC)

> **Rev 2 (same day, after review):**
> 1. **G1/G2 stay in the identified acceleration convention in every airframe
>    XML** — people understand those numbers and tools exist to measure them.
>    The SI conversion moved into code: `sum_g1_g2()` multiplies each row by
>    `STABILIZATION_MFC_INERTIA_XX/YY/ZZ` / `MASS` (defaults 1.0 = legacy
>    accel behavior) when building the allocation matrix. The adaptive
>    estimator (`lms_estimation`) works in raw accel units against the raw G1,
>    exactly as stock.
> 2. The `THRUST_INCR_SP` branch was reverted to the stock
>    `stabilization_indi.c` shape (guidance_mfc emits `THRUST_SP`, so that
>    branch is not on this stack's path and should not diverge from stock).
>    Only the absolute branch changed — and it now matches stock INDI exactly.
> 3. gz `u_min`/`u_max` GCS sliders un-restricted from negative-only
>    ([−100, +100]); the physical defaults in code ([−T_max, 0] N) unchanged.
>    Documented that |−m·g| (the hover magnitude) is also the maximum net
>    force achievable in the gravity direction (at zero thrust).
> 4. `anton_mfc.xml` ported: MODEL section (0.8 kg / 0.0068 / 0.0136 from the
>    anton JSBSim model, TODO measure), G1/G2 untouched, alphas & WLS
>    priorities rescaled, `*_PPRZ_SCALE`/`THRUST_TO_PPRZ` removed,
>    `GZ_MAX_THRUST = 4·(1.5·m)/1000·9600 ≈ 46.1 N`, hover = −m·g ≈ −7.85 N.
> Verified: Hoops ap+nps and ANTON_MFC ap+nps all build; sim hover rerun is
> numerically identical to rev 1 (seed −3.7376 N, counts↔N exact).

The MFC guidance + stabilization stack now speaks SI at every module boundary:
measurements in rad / m in, virtual commands in **N·m (roll/pitch/yaw), N
(thrust)** out. `ATTITUDE_DIRECT`, `ATTITUDE_Z_HOLD` and `NAV` can be toggled
in flight — the stabilizer decodes the thrust setpoint from its own
type/format tags, so no global flag has to agree between modes anymore.

## What changed (code)

- **`stabilization_mfc.c`** — thrust setpoint decode is now tag-driven: both
  the absolute and increment branches go through the stock
  `th_sp_to_thrust_i()` / `th_sp_to_incr_i()` accessors (counts) and map to
  Newtons through the SI G1 thrust row. The `#if GUIDANCE_MFC_THRUST_TO_PPRZ`
  compile guard and the private raw read of `thrust->sp.thrust_f[]` are gone.
  RC-direct PPRZ-int thrust (ATTITUDE_DIRECT) and guidance's normalized float
  land in the same Newtons at the allocator.
- Roll/pitch/yaw `u_min`/`u_max` are computed at init from physical torque
  capability: `±0.5·MAX_PPRZ·Σ|g1g2_row|` [N·m]. The
  `GUIDANCE_MFC_TILT/TWIST_PPRZ_SCALE` fudge factors are removed.
- Pseudo-inverse conditioning is adaptive (normalize by the largest element of
  G·Gᵀ) instead of the fixed ×1000 that assumed accel-scale rows.
- `lms_estimation` (adaptive, off by default) scales the measured
  accelerations by `STABILIZATION_MFC_INERTIA_XX/YY/ZZ` / `MASS` so it
  estimates the SI G matrix.
- G2 stays **out of the allocation matrix** (`sum_g1_g2` builds G1/1000 rows
  unconditionally — the settled 2026-07-09 decision, see Knowledge/13). G2 is
  rescaled by I_zz in the XML for the adaptive estimator only.
- **`guidance_mfc.c`** — gz commands Newtons (hover ≈ −m·g); emitted as a
  properly normalized `THRUST_SP_FLOAT` = |T|/`GZ_MAX_THRUST` ∈ [0:1],
  honoring the public ThrustSetpoint contract. The
  `guidance_mfc_thrust_to_pprz` runtime toggle (which never reached its
  consumer — the stabilizer branched on the compile-time macro) and
  `GUIDANCE_MFC_THRUST_TO_PPRZ`/`THRUST_PPRZ_SCALE` are **removed entirely**;
  one packaging serves every downstream. gz clamps to
  [−`GZ_MAX_THRUST`, 0] N; gx/gy clamp to ±g·sin(`GUIDANCE_H_MAX_BANK`) m/s²;
  tilt scaling uses T/m via `GUIDANCE_MFC_MASS`.
- **`mfc_core.h`** — documented the unit contract: the SISO core is
  unit-agnostic, alpha carries the conversion (`command·alpha` must have the
  units of the measure's second derivative).

## What changed (hoops_111_mfc.xml)

- New `MODEL` section: `MASS` 0.381 kg, `INERTIA_XX/YY` 0.0068,
  `INERTIA_ZZ` 0.0136 kg·m² — **PLACEHOLDERS** taken from the NPS JSBSim
  model (`simple_x_quad_ccw`) so the sim is self-consistent.
  **TODO: bench-measure the real Hoops mass and inertia.**
- G1 rows rescaled to SI-per-count (×1000 `MFC_G_SCALING` kept for
  readability): torque rows = I_axis × identified accel row, thrust row =
  m × accel row. The INDI-era numbers (±14, ±14, ±0.9, −0.7) stay visible as
  factors in the XML expressions.
- `*_PPRZ_SCALE` defines removed; limits are now derived in code / in N.

## Retune ledger — every gain whose numeric scale changed

| Gain | Old (accel units) | New (SI) | Rule |
|---|---|---|---|
| `STABILIZATION_MFC_ROLL_ALPHA` | 2 | `2/I_xx` ≈ 294 | alpha_SI = alpha_old / I_xx |
| `STABILIZATION_MFC_PITCH_ALPHA` | 2 | `2/I_yy` ≈ 294 | / I_yy |
| `STABILIZATION_MFC_YAW_ALPHA` | 3 | `3/I_zz` ≈ 221 | / I_zz |
| `GUIDANCE_MFC_GZ_ALPHA` | 5 | `5/m` ≈ 13.1 | alpha_SI = alpha_old / m |
| `GUIDANCE_MFC_GX/GY_ALPHA` | 15 | 15 (unchanged) | gx/gy still command m/s² |
| `GZ_NOMINAL_HOVER_THROTTLE` | −8 (hand-tuned accel-ish) | `−9.81·m` ≈ −3.74 N | = −m·g by definition |
| `GZ_MAX_THRUST` (new) | — | `4·(0.7·m)/1000·9600` ≈ 10.24 N | Σ\|G1_thrust\|/1000·MAX_PPRZ; must track the stabilizer G1 thrust row |
| `STABILIZATION_MFC_WLS_PRIORITIES` | {1000,1000,1,100} | {1000/I_xx, 1000/I_yy, 1/I_zz, 100/m} | Wv_i/row_scale keeps Wv·v (cost balance) identical — **REVISIT** once real inertia is measured |
| stab u_min/u_max | ±MAX_PPRZ/scale (PPRZ arithmetic) | ±0.5·MAX_PPRZ·Σ\|g1g2 row\| (≈ ±1.83 N·m roll/pitch, ±0.24 N·m yaw) | computed at init |
| gz u_min/u_max | [−57.6, 0] | [−10.24, 0] N | physical thrust envelope |

Because alpha, G1 and Wv were all rescaled by the *same* constants, the
closed-loop behavior is numerically identical to the pre-refactor tuning —
the placeholders cancel out everywhere **except** the new absolute meanings
(hover ≈ −m·g, clamps in N·m). Wrong placeholder values therefore do not
change flight behavior of the SISO loops, but they do shift the clamps and
the normalized-thrust ceiling — measure before trusting the envelope.

## Physical constants needing bench measurement

- `MODEL_MASS` (placeholder 0.381 kg = JSBSim empty weight)
- `MODEL_INERTIA_XX/YY` (0.0068 kg·m²), `MODEL_INERTIA_ZZ` (0.0136 kg·m²)

## Verification done

- `./pprz.sh build Hoops_111_MFC ap` ✓ and `nps` ✓ (only automated check).
- NPS sim (`./sim.sh Hoops_111_MFC --rc_script 5`, NAV takeoff → Z_HOLD):
  altitude tracks −2.000 m to the mm; mode-entry gz seed reads exactly
  −m·g = −3.7376 N; steady hover cmd_z ≈ −2.55 N which matches the
  G1-predicted thrust of the observed 4×2400 actuator counts to 3 digits —
  the counts↔N round trip is unit-exact. The −2.55 vs −3.74 N gap is the
  JSBSim motor model being ~1.46× more effective than the INDI-identified
  −0.7 thrust row (plant-model mismatch absorbed by the F-estimator), not a
  units error.
- **NOT validated**: functional correctness in the real world — requires
  bench/flight test. In-flight mode toggling exercised only structurally
  (rc_script 5 NAV→Z_HOLD in sim).

## Knock-on for other airframes (flagged, not changed)

- `anton_mfc.xml` still carries accel-scaled G1 and the deleted
  `THRUST_TO_PPRZ`/`*_PPRZ_SCALE` defines (now inert). ANTON_MFC compiles,
  but its gz path now emits normalized thrust against the *default*
  `GZ_MAX_THRUST` (2·g·1 kg) — port the MODEL-section pattern before flying
  ANTON on this branch.
- `oneloop_mfc.c` keeps its own private copies of the old scale macros —
  untouched, unaffected.
