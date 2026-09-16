# Flatness Trajectory Setpoints (Pos–Vel–Accel–Jerk–Snap + Psi)

## Context

Quadrotor guidance is differentially flat in `(x, y, z, psi)`: a smooth flat trajectory and its derivatives fully determine the vehicle's attitude, rate, and thrust references. An external planner (e.g. a min-snap generator, or Simulink/offline optimizer) can produce that flat trajectory and stream it to the autopilot — but the current onboard plumbing only carries it up to acceleration, and even then doesn't interpolate it properly. This note is a design plan for extending it, motivated by two gaps found while auditing the existing pipeline:

1. **Wire format stops at acceleration.** `GUIDED_FULL_NED` (pprzlink message id 41, `sw/ext/pprzlink/message_definitions/v1.0/messages.xml:2827-2865`) carries `x,y,z, vx,vy,vz, ax,ay,az, heading`. No jerk, no snap, no heading-rate/heading-accel/heading-jerk/heading-snap.
2. **No interpolation on the "full setpoint" path.** When pos+vel+accel all arrive together, `guidance_h_set_all()` sets `h_mask = GUIDANCE_H_SP_ALL`, which routes through `gh_set_ref()` — a **direct field overwrite**, re-executed every control-loop tick with whatever `guidance_h.sp` currently holds (`guidance_h.c:284`, `guidance_h_ref.c:95-103`). Between two datalink packets this is a **zero-order hold** (staircase), unlike the genuine 2nd-order spring-damper smoothing (`gh_update_ref_from_pos_sp`) used when only a bare position is supplied.

The intended use here is connecting a flatness-based feedforward trajectory to **MFC** (`guidance_mfc.c`), but the design below is deliberately generic and works for INDI too — see [[03 - INDI Guidance Deep Dive]].

## Why this is generic across INDI and MFC

Both `guidance_indi.c` and `guidance_mfc.c` plug into the **same** shared structs via the same generic dispatch functions declared in `guidance_h.h`/`guidance_v.h` (`guidance_h_run_pos/speed/accel`, `guidance_v_run_pos/speed/accel`):

- `guidance_indi_controller()` reads `gh->ref.accel` as its INDI feedforward term (`guidance_indi.c:442-491`, `accel_sp.x = speed_fb.x + ACCEL_FLOAT_OF_BFP(gh->ref.accel.x)`).
- `guidance_mfc.c:564` reads `gh->ref.pos`; `guidance_mfc.c:586` reads `gh->sp.heading`.

**Key design decision:** jerk/snap do not need to be plumbed into either controller's control law at all. They only need to make the *reference model* (`guidance_h_ref.c`/`guidance_v_ref.c`) Taylor-extrapolate `pos/vel/accel` forward between sparse datalink packets instead of holding them constant. Since both controllers already read `gh->ref.pos/speed/accel` every tick, richer reference values flow to both automatically — this is the main economy of the design, and it fixes the ZOH staircase problem in the same stroke.

## Current architecture (recap)

```
GUIDED_FULL_NED (datalink)
        │
        ▼
autopilot_guided_parse_GUIDED_FULL()   [autopilot_guided.c:166-188]
        │  calls
        ▼
guidance_h_set_all(x,y,vx,vy,ax,ay)    [guidance_h.c:536] → h_mask = GUIDANCE_H_SP_ALL
guidance_v_set_all(z,vz,az)            [guidance_v.h:172]
        │
        ▼
guidance_h_update_reference()          [guidance_h.c:273] every control tick
        │  h_mask == SP_ALL →
        ▼
gh_set_ref(pos, speed, accel)          [guidance_h_ref.c:95] ← direct overwrite, NOT integrated
        │
        ▼
guidance_h.ref.{pos,speed,accel}       ← read by both guidance_indi.c and guidance_mfc.c
```

Contrast with the bare-position path, which *does* integrate a smooth reference:

```
gh_update_ref_from_pos_sp(pos_sp)      [guidance_h_ref.c:105]
  → 2nd-order critically-damped spring-damper (omega, zeta)
  → produces a converging pos/vel/accel triplet every tick
```

## Proposed design

### 1. New pprzlink message: `GUIDED_TRAJECTORY_NED`

Add **alongside** `GUIDED_FULL_NED` in `sw/ext/pprzlink/message_definitions/v1.0/messages.xml` — additive, not a replacement, so existing senders/GCS builds using the fixed-layout `GUIDED_FULL_NED` keep working unmodified (pprzlink messages have no optional fields; extending the existing message in place would break any sender still on the old field set).

Fields: `x,y,z, vx,vy,vz, ax,ay,az, jx,jy,jz, sx,sy,sz, heading, heading_rate, heading_accel, heading_jerk, heading_snap, order (uint8), ac_id`.

`order` tells the receiver how many derivative levels the sender actually populated (0 = pos-only … 4 = snap), so trailing fields the sender didn't compute are ignored rather than trusted as zero. Exact message id to be assigned at implementation time (next free id in the file).

### 2. Struct extensions

- `guidance_h.h`: add `jerk`/`snap` (`struct FloatVect2`, reference is float-precision already for heading — pos/speed/accel are fixed-point `Int32Vect2`, so jerk/snap should follow that convention) to `struct HorizontalGuidanceSetpoint` and `struct HorizontalGuidanceReference`; add `heading_accel`, `heading_jerk`, `heading_snap` (floats) alongside the existing `heading`/`heading_rate`.
- `guidance_v.h`: matching `jdd_sp/ref`-style (or clearer `j_sp/ref`, `s_sp/ref`) jerk/snap scalars, mirroring the existing `zdd_sp/ref` naming.
- `guidance_h_ref.h`/`guidance_v_ref.h`: add matching `jerk`/`snap` state to `struct GuidanceHRef` and the vertical equivalent, so the reference model has somewhere to hold the last-received higher-order terms between packets.

### 3. New reference-model path (this is what fixes the ZOH gap)

- `gh_set_flat_ref(pos, vel, accel, jerk, snap, order)` — stores state, analogous to `gh_set_ref()`.
- `gh_update_ref_from_flat_ref(dt)` — runs every control tick (same cadence as `gh_update_ref_from_pos_sp`) and Taylor-extrapolates forward using whichever order was actually supplied:
  ```
  pos   += vel*dt + 1/2*accel*dt^2 + 1/6*jerk*dt^3 (+ 1/24*snap*dt^4 if order ≥ 4)
  vel   += accel*dt + 1/2*jerk*dt^2 (+ 1/6*snap*dt^3 if order ≥ 4)
  accel += jerk*dt (+ 1/2*snap*dt^2 if order ≥ 4)
  jerk  += snap*dt (if order ≥ 4)
  ```
  Re-synced (state reset) every time a fresh packet arrives, so extrapolation error only accumulates over one inter-packet gap (~20-50 Hz companion link vs ~512 Hz control loop — a small window).
- Same treatment for `guidance_v_ref.c`.
- Heading: extend `guidance_h.c:354`'s current single-term `sp.heading += sp.heading_rate/PERIODIC_FREQUENCY` to the equivalent quartic Taylor update using `heading_accel/jerk/snap` when present.
- In-tree precedent for higher-order reference-model integration: `stabilization_andi.c`'s existing 3rd-order attitude reference model (`x_3d_ref`, `max_ang_jerk`) — same pattern, one level deeper here.

### 4. No changes needed in `guidance_indi.c` or `guidance_mfc.c`

Both already read `gh->ref.pos/speed/accel` each tick; the richer, Taylor-corrected reference flows through with zero controller-specific code.

### 5. MFC follow-up (flagged, not solved by this plan)

`guidance_mfc.c` currently only reads `gh->ref.pos` (`guidance_mfc.c:564`) and computes its own internal feedback/derivative via `mfc_siso_run` rather than consuming `ref.speed`/`ref.accel` as feedforward (see `guidance_mfc.c:564-586`, `accel_to_att_sp`). Getting MFC to actually *use* the richer trajectory as feedforward — not just have it available — is a separate follow-on step: wiring `gh->ref.speed/accel` (and possibly `jerk`) into `mfc_siso_run`/`accel_to_att_sp` as an explicit feedforward term alongside MFC's own feedback loop. Out of scope here; this plan only gets the data to `gh->ref` generically.

### 6. New setter API + datalink parser

- `guidance_h_set_flat(...)` / `guidance_v_set_flat(...)` in `guidance_h.c`/`guidance_v.c`, calling `gh_set_flat_ref`/`gv_set_flat_ref`.
- New `autopilot_guided_parse_GUIDED_TRAJECTORY()` in `autopilot_guided.c`, additive next to the existing `autopilot_guided_parse_GUIDED_FULL` (left untouched).

## Open questions / design rationale

- **New message vs. extending `GUIDED_FULL_NED`**: new message chosen to preserve binary-layout compatibility with any existing sender/GCS build still targeting the old field set.
- **Psi taken to full snap-order parity** with position, per explicit choice, even though yaw dynamics rarely need jerk/snap-level smoothness in practice — kept symmetric with the position API rather than truncated.
- **`AP_MODE_GUIDED` is not currently enabled on any ENAC airframe** (checked during this investigation) — a target aircraft's airframe XML will need `AP_MODE_GUIDED` wired in before this is flight-usable.
- **Fixed-point vs float**: pos/speed/accel in `guidance_h` are fixed-point (`Int32Vect2`, `POS_BFP_OF_REAL` etc.); jerk/snap magnitudes are typically small and could plausibly stay float to avoid extending the fixed-point scaling scheme — worth deciding at implementation time based on measured jerk/snap ranges for the target trajectories.

## Verification plan

1. Small Python/Ivy script publishing a synthetic minimum-snap trajectory (known closed-form polynomial, so ground truth is available) over `GUIDED_TRAJECTORY_NED` at a realistic companion-computer rate (20-50 Hz) against the NPS sim.
2. Inspect `guidance_h.ref.pos/speed/accel` via the existing PlotJuggler telemetry pipeline (`pj_json_relay.py`) to confirm smooth Taylor-extrapolated motion between packets — compare against current `GUIDED_FULL_NED` behavior as a staircase baseline.
3. Offline unit check of the Taylor-extrapolation math against the known polynomial trajectory (pure numerical comparison, no hardware) before any flight test.
4. Only after (1)-(3) look good: enable `AP_MODE_GUIDED` on a bench/sim-only airframe copy and confirm MFC/INDI attitude commands stay smooth across a live packet stream.

## Source files touched

| File | Purpose |
|------|---------|
| `sw/ext/pprzlink/message_definitions/v1.0/messages.xml` | New `GUIDED_TRAJECTORY_NED` message |
| `sw/airborne/firmwares/rotorcraft/guidance/guidance_h.h` / `.c` | Setpoint/reference structs, `guidance_h_set_flat` |
| `sw/airborne/firmwares/rotorcraft/guidance/guidance_v.h` / `.c` | Vertical equivalent |
| `sw/airborne/firmwares/rotorcraft/guidance/guidance_h_ref.h` / `.c` | Taylor-extrapolation reference model |
| `sw/airborne/firmwares/rotorcraft/guidance/guidance_v_ref.h` / `.c` | Vertical equivalent |
| `sw/airborne/firmwares/rotorcraft/autopilot_guided.h` / `.c` | New `GUIDED_TRAJECTORY_NED` parser |
| `sw/airborne/firmwares/rotorcraft/guidance/guidance_mfc.c` | Follow-up only: wire `ref.speed/accel` into MFC feedforward |

## See Also

- [[03 - INDI Guidance Deep Dive]]
- [[07 - All Touch Points Cheatsheet]]
- [[Sessions/2026-06-24-mfc-usekd-trajec-sp]]
- [[Sessions/2026-06-16-guidance-mfc-filters]]
