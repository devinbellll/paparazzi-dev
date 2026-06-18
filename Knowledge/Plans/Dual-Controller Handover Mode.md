# Plan — Dual-Controller Handover Mode (switch MFC ↔ INDI in flight)

**Goal:** with both the MFC and INDI stacks compiled in (guidance + stabilization),
switch **which one drives the motors** in flight — bumplessly — via an RC switch and
GCS dropdown, exactly like the oneloop ANDI/INDI precedent. Only one law has motor
authority at a time. Pairs with `Dual-Controller Shadow Mode.md`: shadow proves MFC
is sane *passively*; handover then *gives it the motors* once trusted.

Date drafted: 2026-06-18. Target airframe: `conf/airframes/ENAC/quadrotor/anton_mfc.xml`.
Read first: `Knowledge/11 - In-Flight Controller Switching (Oneloop Pattern).md`.

---

## Relationship to the Shadow plan

This plan **reuses Phase 0 (de-confliction) verbatim** from
`Dual-Controller Shadow Mode.md` — both stacks must link into one `.elf` before
either approach is possible. The recommended sequencing is:

1. Do Shadow Phase 0 + Phase 1 (both stacks linked, MFC shadowed, INDI driving).
2. Validate MFC in shadow (Shadow Phase 2).
3. **Then** add this plan's switching layer to promote MFC from shadow to active.

If shadow validation is skipped, this plan still stands alone on top of Phase 0, but
first flight with MFC active is then higher risk.

---

## The handover mechanism (faithful to oneloop)

Oneloop flips `oneloop_andi.ctrl_type` and re-`enter()`s. We do the analogous thing
at the **dual wrapper** level: a single selector chooses which underlying law's `cmd[]`
reaches `commands[]`, and entering a law re-linearizes it about the current state.

### 1. Selector state
In the dual wrapper:
```c
#define CTRL_INDI 0   // trusted incumbent — default / startup
#define CTRL_MFC  1
extern uint8_t dual_ctrl_active;   // which law drives motors
```

### 2. Both laws keep running (cheap insurance)
Keep computing **both** every tick (as in Shadow Phase 1) so the inactive law's
filters stay warm and a switch is instantaneous. The selector only changes **which
buffer is committed**:
```c
stabilization_attitude_run_indi(in_flight, sp, thrust, cmd_indi);
stabilization_attitude_run_mfc (in_flight, sp, thrust, cmd_mfc);
// dual set_rotorcraft_commands() commits the ACTIVE one:
pprz_t *src = (dual_ctrl_active == CTRL_MFC) ? cmd_mfc : cmd_indi;
set_rotorcraft_commands(commands, src, in_flight, motors_on);
```
Same pattern at guidance: feed the active law's `StabilizationSetpoint` to the active
stabilizer. (Keep guidance/stabilization selection coupled — switch both together —
unless you explicitly want mixed pairings; if so, two independent selectors.)

### 3. Bumpless transfer — re-enter on switch
On any change of `dual_ctrl_active`, call the **newly active** law's `enter()` before
its first commit:
```c
void dual_ctrl_set_active(uint8_t want) {
  if (want == dual_ctrl_active) return;
  if (want == CTRL_MFC) { stabilization_mfc_enter();  guidance_mfc_enter_equiv();  }
  else                  { stabilization_indi_enter(); guidance_indi_enter();       }
  dual_ctrl_active = want;
}
```
`*_enter()` re-linearizes the incremental law about the current measured state /
actual actuator positions → no step at the switch (this is precisely why oneloop calls
`oneloop_andi_enter()` in each mode's `<on_enter>`). Because both laws were already
running warm (step 2), the new law's internal filters are already converged.

---

## Wiring the trigger (RC + GCS), oneloop-style

Two implementation options — pick **A** for least disruption:

### Option A — selector inside the wrapper, driven by a GCS setting + RC channel
- Add a `<dl_setting>` for `dual_ctrl_active` (INDI|MFC) in the airframe's settings or
  the dual module XML → GCS dropdown.
- Add an RC gate: define `AP_MODE_SWITCH = RADIO_AUXn` in `anton_mfc.xml` (mirror the
  rotwing airframe's `RADIO_AUX7`), read it in the wrapper each tick via
  `rc_mode_switch(AP_MODE_SWITCH, pos, 3)` and call `dual_ctrl_set_active()`.
- The existing autopilot mode machine is untouched; this is a sub-mode within whatever
  flight mode is active (NAV/ATTITUDE). Simplest, and keeps NAV behavior intact.

### Option B — full oneloop-style autopilot XML (two modes)
Write `conf/autopilot/anton_dual_switch.xml` with two NAV-class modes whose only
difference is `<on_enter>` (`dual_ctrl_set_active(CTRL_INDI)` vs `…(CTRL_MFC)`), gated
by `<select cond="RCMode2() && RCAP2() && DLModeNav()">` etc., exactly like
`rotorcraft_oneloop_switch.xml` modes NAV(13)/MODULE(17). Reference
`conf/autopilot/anton_mfc_autopilot.xml` for the ENAC mode set to extend. More
faithful to the precedent, more surface area to get right. Choose this only if you
want the switch surfaced as a first-class autopilot mode.

**Recommendation:** Option A. It delivers the same in-flight RC/GCS switch with far
less codegen risk, and the wrapper already owns the commit point.

---

## Safety gating (do not skip)

- **Default to INDI** at startup and on any failsafe/RC-loss (`RCLost()` → force
  `CTRL_INDI`). MFC is the experimental law.
- **Arming guard:** only allow promotion to MFC when `autopilot_in_flight()` and INDI
  has been stable — optionally require a GCS "arm MFC" confirm settling like the
  shadow agreement band before the switch is accepted.
- **One-way panic:** a single RC switch position (or kill-adjacent channel) forces
  back to INDI regardless of GCS state.
- **Re-enter on every promotion**, never trust warm state alone across a long dwell.

---

## Touch points (checklist)

- [ ] Everything in **Shadow Phase 0** (de-confliction) — shared prerequisite
- [ ] Dual wrapper: add `dual_ctrl_active`, `dual_ctrl_set_active()`, active-buffer commit
- [ ] Dual wrapper guidance: select active law's setpoint; add `guidance_mfc_enter_equiv()`
- [ ] `anton_mfc.xml`: define `AP_MODE_SWITCH = RADIO_AUXn`; add `dual_ctrl_active` dl_setting
- [ ] (Option B only) NEW `conf/autopilot/anton_dual_switch.xml` + `<autopilot name=…>`
- [ ] Failsafe path forces `CTRL_INDI`
- [ ] Telemetry: log `dual_ctrl_active` + per-axis `cmd` of both (reuse `DUAL_CTRL` msg)
- [ ] Rebuild clean: `./pprz.sh rebuild ANTON_MFC nps`

## Validation sequence
1. NPS: switch INDI→MFC→INDI in hover; confirm **no command step** at the switch
   (plot committed `cmd[]` across the transition). Then in gentle translation.
2. NPS: trigger `RCLost()` while MFC active → must snap to INDI immediately.
3. Flight: only after shadow validation + clean NPS handover. Switch at altitude with
   margin; be ready to flip back to INDI on the RC panic position.

## Risks
- **Bump at transfer** if `enter()` is incomplete — the biggest failure mode. Verify
  bumplessness in NPS exhaustively before flight.
- **CPU/flash** of two warm stacks at 1000 Hz on Tawaki — profile; prescale the
  inactive law if needed (but a prescaled inactive law is less warm → larger switch
  transient; re-`enter()` covers this).
- **Mixed guidance/stabilization pairing** (e.g. MFC guidance + INDI stab) multiplies
  validation cases — keep them coupled unless there is a specific reason.
- Compile ≠ correct. NPS then flight test.
