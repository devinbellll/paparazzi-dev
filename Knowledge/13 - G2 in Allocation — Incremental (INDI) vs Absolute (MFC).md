# G2 in the Allocation Matrix — Why (G1+G2) Is Correct for INDI's Incremental Commands but Wrong for MFC's Absolute Commands

Both `stabilization_indi.c` and `stabilization_mfc.c` build the allocation matrix the same way (`sum_g1_g2()`): the yaw row is `(G1[2] + G2) / 1000`, all other rows `G1 / 1000`. For INDI this is exact. For MFC it silently overstates the steady-state yaw effectiveness by **(G1_yaw + G2)/G1_yaw ≈ 31×** on ANTON. This note explains why the *same matrix* is right in one controller and wrong in the other.

Companion note: [[12 - First-Order Actuator Dynamics (JSBSim vs Firmware)]] (lag model, where G2's physics comes from).

---

## 1. The two G entries have different units

The continuous physical yaw model (u = actuator state in PPRZ 0..9600):

```
yaw_accel = f + a·u + b·u̇          [rad/s²]
```

- `a = G1_yaw/1000` — steady drag torque: **rad/s² per PPRZ unit** of standing rotor speed.
- `b = G2/(1000·f_s)` — rotor spin-up reaction torque: **rad/s² per PPRZ/s** of rotor slew. The transient contributes **zero at steady state** (u̇ = 0).

The hidden `f_s` (= `PERIODIC_FREQUENCY` = 500 Hz) in G2's definition is the crux. The firmware stores G2 in "per command increment" units: the yaw accel produced by changing the command by Δu **within one 500 Hz step**. Using the finite-difference approximation `u̇ ≈ Δu·f_s`:

```
b·u̇ = b·f_s·Δu = (G2/1000)·Δu
```

So **G1/1000 multiplies u, while G2/1000 multiplies Δu-per-step.** They are dimensionally different objects. Two independent confirmations in the code:

- `lms_estimation()`: `ddu_estimation` is scaled by an extra `/PERIODIC_FREQUENCY` relative to `du_estimation` — the same f_s appearing where the estimator works in derivative units instead of increments.
- ANTON numbers (`anton_indi_aruco.xml`): G1_yaw = ±5, G2 = ±150 → a = 0.005 rad/s²/PPRZ, b = 150/(1000·500) = 3×10⁻⁴ rad/s² per PPRZ/s.

---

## 2. INDI: incremental allocation makes the sum legal

INDI solves for a **per-step increment**:

```
Δν = B·Δu,     B_yaw = (G1_yaw + G2)/1000
```

Both terms now multiply the *same* quantity Δu — the f_s from `u̇ ≈ Δu·f_s` exactly cancels the 1/f_s in G2's physical coefficient, so the entries become dimensionally compatible and summing them is exact (per step). INDI additionally adds the `g2_times_du` feedforward to `indi_v[2]` so the transient torque the *previous* increment is still producing is accounted for. The whole scheme lives in increment space; G2 only ever meets actual per-step changes — the basis it was identified in.

---

## 3. MFC: absolute allocation breaks it

MFC (both allocator paths, `stabilization_mfc.c:850-868`) solves for the **total** command:

```
pinv:  u = (B)⁺ · v          WLS:  min ‖Wv(B·u − v)‖² + …
```

with the same `B_yaw = (G1+G2)/1000`. But the true *static* map is `v_yaw = (G1/1000)·u` — at steady state u̇ = 0 and the G2 term contributes nothing. Feeding a sustained `u` through `(G1+G2)/1000` treats every PPRZ of standing command as if it were a **fresh increment re-applied every 500 Hz step, forever** — a spin-up transient that never decays. Equivalently: it asserts `u̇ = 500·u` permanently.

**Magnitude on ANTON:** assumed steady yaw effectiveness (5+150)/1000 = 0.155 vs true 5/1000 = 0.005 → **31× overstatement**. A yaw virtual command produces ~1/31 of the differential motor command physically required.

### Why the aircraft still flies

- The X-quad axis sign-vectors are mutually orthogonal, so the error stays confined to the yaw channel (roll/pitch/thrust mixing is uncorrupted).
- It presents as a constant plant-gain error — exactly the class of error MFC's ultra-local model (F̂ + α tuning) absorbs. The yaw gains were implicitly tuned *around* the wrong matrix, so closed-loop behavior is fine; only the physical interpretation of the yaw α is off by ~31×.

### Where it genuinely hurts: WLS saturation

Unsaturated with a square 4×4 system, WLS ≈ exact inversion — identical error to pinv, absorbed the same way. But at actuator saturation WLS arbitrates using effectiveness × `WLS_PRIORITIES`, and there the fiction matters: WLS believes yaw is ~31× cheaper than it is, allocates almost no budget to it, and real yaw tracking collapses faster than the priority weights (already lowest at 1) suggest. The mispricing corrupts the trade-off logic precisely in the regime WLS exists for.

---

## 4. The fix (2026-07-09, `feat/shadow-handoff`)

`sum_g1_g2()` no longer adds G2 to the yaw row — **all rows of the allocation
matrix are `G1/1000`, unconditionally** (no compile-time define, no runtime
toggle). The yaw spin-up transient becomes an unmodeled disturbance absorbed by
the MFC ultra-local estimator F̂ — squarely in the MFC philosophy. `g2` stays in
the code solely for the adaptive effectiveness estimator (`lms_estimation`,
off by default).

An earlier iteration added a `STABILIZATION_MFC_G2_IN_ALLOCATION` opt-out flag
with a GCS setting; that was rejected in favor of the direct fix — MFC's
absolute allocation with `(G1+G2)` was simply wrong, and keeping the wrong
behavior selectable adds machinery without value.

**⚠️ Retune required:** the corrected matrix makes the allocator command ~31×
more differential motor per unit of yaw virtual command than before (ANTON
numbers). Yaw MFC gains tuned against the old matrix are now ~31× hot — re-tune
in NPS before flight. Also revisit `WLS_PRIORITIES`: the old weights were tuned
around yaw appearing nearly free.

## 5. Implications for external plant models (Simulink)

The plant obeys the continuous physics, never the allocator's bookkeeping:

```
u_cmd ─► ω/(s+ω), ω = 30.5 ─► u_act          (u̇_act = ω·(u_cmd − u_act), free tap)
yaw_accel = (G1_yaw/1000)·u_act + (G2/(1000·500))·u̇_act
τ = J·ω̇,  F_z = m·a_z                        (G rows are accelerations; J, m re-attach units)
```

Do **not** model the plant as `(G1+G2)/1000·u_act` — that applies the spin-up torque permanently. `(G1+G2)` belongs only on the controller side, and only when the controller is incremental. If simulating the legacy MFC allocator faithfully, reproduce its absolute `(G1+G2)` allocation *including* the modeling error — the controller-vs-plant mismatch is part of the real system.

Transient vs steady intuition (ANTON): for a step Δu the spin-up peak is `b·ω·Δu ≈ 1.8×` the steady G1 change, decaying with τ ≈ 33 ms; the crossover is at `a/b ≈ 16.7 rad/s ≈ 2.7 Hz` — above that, yaw response is dominated by rotor inertia torque, below by drag torque.

---

## 6. Related

- Bug log: `bug-185` in `.wolf/buglog.json`
- `Knowledge/12` — actuator lag model, JSBSim washout ↔ firmware G2 correspondence
- `lms_estimation()` sidebar: the adaptive estimator regresses the once-differentiated model (`rate_dd` vs `g1·u̇ + g2·ü`) to eliminate the unknown disturbance term — G2 pairing with the *second* derivative there is consistent, not a bug.
