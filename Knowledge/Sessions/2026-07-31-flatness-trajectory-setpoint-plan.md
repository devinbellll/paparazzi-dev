# 2026-07-31 — Flatness trajectory setpoint plan

## Changed

- Wrote [[Plans/Flatness Trajectory Setpoints (Pos-Vel-Accel-Jerk-Snap + Psi)]] — a design plan
  for extending the rotorcraft GUIDED-mode datalink setpoint interface from pos/vel/accel up to
  jerk/snap, plus heading/psi through full snap-order parity. No source code changed this
  session — this was documentation/planning only.
- Added a row to `00 - Index.md`'s Structure table and a new "Add Higher-Order (Jerk/Snap)
  Trajectory Setpoints" section to `07 - All Touch Points Cheatsheet.md`.

## Learned

- **GVF (`gvf/`, `gvf_parametric/`) vs GUIDED-mode datalink setpoints are two separate trajectory
  mechanisms in this tree.** GVF computes onboard, closed-form paths (implicit gradient/Hessian,
  or explicit parametric f/f'/f'') and outputs a heading/turn-rate command — it's a guidance-level
  vector field, not a state-setpoint feed. Only `PANACHE_1` (fixed-wing) in the ENAC fleet
  actually wires GVF in; no quad/hybrid does.
- **`GUIDED_FULL_NED` is the actual "external trajectory box" hook** — an offboard companion
  computer streams pos+vel+accel over the datalink, `guidance_h_set_all()`/`guidance_v_set_all()`
  write straight into `guidance_h.sp`, and (because `h_mask` becomes `GUIDANCE_H_SP_ALL`) those
  values get copied directly into `guidance_h.ref` every control tick via `gh_set_ref()` — a
  **zero-order hold**, not the 2nd-order spring-damper smoothing (`gh_update_ref_from_pos_sp`)
  used for a bare position setpoint. Between two datalink packets the reference is a staircase.
  This is `guidance_h.c:273-357`, `guidance_h_ref.c:95-141`.
- **`guidance_mfc.c` already plugs into the exact same generic `gh`/`gv` struct interface as
  `guidance_indi.c`** (same `guidance_h_run_pos/speed/accel` dispatch functions declared in
  `guidance_h.h`). This means a jerk/snap-aware reference model doesn't need any
  controller-specific code — both consumers read `gh->ref.pos/speed/accel` and benefit for free.
  MFC today only reads `gh->ref.pos` though (`guidance_mfc.c:564`), not speed/accel — so it
  currently derives its own accel command internally rather than taking guidance's feedforward;
  actually using the richer reference in MFC is flagged as a separate follow-up in the plan.
- `stabilization_andi.c`'s existing 3rd-order attitude reference model (`x_3d_ref`,
  `max_ang_jerk`) is the in-tree precedent for how a higher-order reference-model integration
  should be structured — cited in the plan as the template for the new
  `gh_update_ref_from_flat_ref()`.

## Next

- Implement the plan: new `GUIDED_TRAJECTORY_NED` pprzlink message, `guidance_h`/`guidance_v`
  struct + reference-model extensions (Taylor extrapolation using jerk/snap between sparse
  datalink packets), new datalink parser in `autopilot_guided.c`.
- Follow-up once the above lands: wire `gh->ref.speed/accel` into `guidance_mfc.c`'s
  `mfc_siso_run`/`accel_to_att_sp` as an explicit feedforward term — MFC doesn't consume it yet.
- No ENAC airframe currently enables `AP_MODE_GUIDED` — will need enabling on a target aircraft
  (bench/sim first) before any of this is flight-usable.
