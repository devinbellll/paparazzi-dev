# Research report: precedents for self-contained canned in-flight maneuver trigger

(No code changes made — read-only research task.)

## 1. Flight-plan block calling a C function (best precedent)

`conf/flight_plans/ENAC/fish_outdoor.xml:85-89`:
```xml
<block group="fish" name="Guided_run" strip_button="Guided run">
  <exception cond="!InsideSafety(GetPosX(),GetPosY())" deroute="Standby"/>
  <call_once fun="autopilot_set_mode(AP_MODE_GUIDED)"/>
  <call fun="nav_fish_velocity_run()"/>
</block>
```
`<call_once>` fires once on block entry, `<call>` fires every nav loop while in the block (~4 Hz). Implementation: `sw/airborne/modules/nav/nav_fish.c:487` `bool nav_fish_velocity_run(void)` — it directly calls `autopilot_guided_update(GUIDED_FLAG_XY_BODY | GUIDED_FLAG_XY_VEL, ...)`, the same low-level entry point your `GUIDED_TRAJECTORY_NED` datalink parser calls. Module wiring in `conf/modules/nav_fish.xml` (`<depends>@navigation</depends>`, `<makefile><file name="nav_fish.c"/></makefile>`). The `strip_button="Guided run"` attribute also auto-adds a GCS one-click button that deroutes into this block — a second, complementary trigger for free.

`NavGuided` macro (`sw/airborne/firmwares/rotorcraft/autopilot_guided.h:118`) is the simpler declarative alternative for flight plans that just need a single guided setpoint per block entry, but `nav_fish` shows the pattern for a per-tick canned trajectory, which is what you need for the flat-trajectory Taylor reference.

## 2. dl_settings → C function handler

`conf/modules/sys_id_chirp.xml:37-46` defines multiple `<dl_setting ... handler="activate_handler"/>` entries; the "Chirp" toggle (values `Inactive|Active`) calls `sys_id_chirp_activate_handler(uint8_t activate)` in `sw/airborne/modules/system_identification/sys_id_chirp.c:137`. The module runs via `<periodic fun="sys_id_chirp_run()" freq="60" autorun="TRUE"/>` and the handler just sets a running flag/index; it does not itself change AP mode — the airframe file's `<command_laws>` mixes the chirp output in regardless of mode, and docs tell the operator to switch to the right mode manually first. Useful as the "GCS button that calls a C function with a value" pattern, but it is not mode-aware/self-gating the way `nav_fish`'s flight-plan block is.

## 3. RC-triggered canned maneuvers

- `--rc_script` is NPS/SITL-only test harness code: `sw/simulator/nps/nps_radio_control.c:60-70` defines `typedef void (*rc_script)(double); static rc_script scripts[] = {...}`, invoked from `nps_autopilot_rotorcraft.c`/`nps_main_sitl.c`. It fakes RC stick motion for regression testing (e.g. `anton_mfc.xml:60`/`anton_mfc_attitude_direct.xml`), it does not exist on real firmware and is not usable as an in-flight trigger.
- No firmware-side precedent of an AUX-channel threshold gating entry into a *mode or maneuver* was found in guidance/stabilization/flight-plan code. Existing `radio_control.values[RADIO_AUXn]` uses (`oneloop_andi.c:1718/1815`, `eff_scheduling_nederdrone.c:223`) only feed continuous values into control logic, not discrete mode/maneuver triggers. The only real "3-way switch → mode" mechanism is the standard `MODE_AUTO1`/`MODE_AUTO2` mapping (see #4) — there is no extra AUX-channel-gated maneuver trigger to imitate.

## 4. ENAC AP_MODE_GUIDED airframes

Both `conf/airframes/ENAC/quadrotor/hoops_111_indoor.xml:203-205` and `hoops_112_hinf_outdoor.xml:245-247` map identically:
```xml
<define name="MODE_MANUAL" value="AP_MODE_ATTITUDE_DIRECT"/>
<define name="MODE_AUTO1"  value="AP_MODE_GUIDED"/>
<define name="MODE_AUTO2"  value="AP_MODE_NAV"/>
```
So AUTO1 position of the standard 3-way RC mode switch already drops the vehicle straight into `AP_MODE_GUIDED` — no AUX channel needed, just the normal mode switch.

`hoops_111_indoor` is used with `flight_plan="flight_plans/ENAC/rotorcraft_survey_vto.xml"` per `conf/conf.xml:49`. `hoops_112_hinf_outdoor.xml` exists in `conf/airframes/ENAC/quadrotor/` but is **not currently referenced in `conf/conf.xml`** (nor `conf/userconf/ENAC/conf_mfc.xml`) — it is effectively orphaned/unused in the current aircraft list, so there is no "its flight plan" to point to.

## (a) Clearest pattern to reuse
`nav_fish.c`/`nav_fish.xml` + `fish_outdoor.xml` "Guided_run" block: `<call_once fun="autopilot_set_mode(AP_MODE_GUIDED)"/>` then `<call fun="your_func()"/>` calling into guidance every tick — directly mirrors what `guidance_h_set_flat`/`guidance_v_set_flat` need (called once per control tick while GUIDED is active).

## (b) Recommendation
Use a **flight-plan block**, not a dl_setting or RC gesture:
- It is fully self-contained (no companion computer, matches your "no external stream" requirement).
- It can both set `AP_MODE_GUIDED` and call a trigger function per tick, exactly like `nav_fish`.
- It's still reachable from three places for free: GCS strip button (`strip_button=`), `deroute`/exception from other blocks, and — if you also want a hardware trigger — an `<exception cond="radio_control.values[RADIO_AUXn] @GT ...">` inside another block that deroutes into it, giving you an RC-gesture trigger on top of the flight-plan mechanism without inventing new firmware plumbing. A pure dl_setting handler is weaker here because (per `sys_id_chirp`) it typically doesn't manage AP mode transitions itself, so you'd have to duplicate the `autopilot_set_mode(AP_MODE_GUIDED)` + timing/state logic that a flight-plan block gives you for free via `<call_once>`/`<call>`/`<exception>`.

Suggested shape:
```xml
<block name="Flat_trajectory_demo" strip_button="Run Flat Traj">
  <exception cond="!InsideSafety(GetPosX(),GetPosY())" deroute="Standby"/>
  <call_once fun="autopilot_set_mode(AP_MODE_GUIDED)"/>
  <call_once fun="flat_traj_start()"/>
  <call fun="flat_traj_run()"/>
  <exception cond="flat_traj_is_done()" deroute="Standby"/>
</block>
```
where `flat_traj_start()`/`flat_traj_run()` (new module, e.g. `modules/nav/nav_flat_traj.c`, modeled 1:1 on `nav_fish.c`) walk the canned array and call `guidance_h_set_flat()`/`guidance_v_set_flat()` each tick, same as `autopilot_guided_parse_GUIDED_TRAJECTORY()` does per packet.

## (c) Gotchas entering AP_MODE_GUIDED from a trigger (vs. always-on external stream)
- `autopilot_set_mode(AP_MODE_GUIDED)` → `guidance_h_mode_changed(GUIDANCE_H_MODE_GUIDED)` calls `guidance_h_hover_enter()` (`sw/airborne/firmwares/rotorcraft/guidance_h.c:142-144`, dispatch in `autopilot_static.c:278-280`), which seeds the guided setpoint to the **current** position/heading. If your canned trajectory's `x0,y0` doesn't match wherever the vehicle happens to be when the block/trigger fires, there will be a discontinuous jump the instant `flat_traj_start()` overwrites `sp.h_mask`/reference-model state. With an external stream this is less of an issue because the GCS operator typically starts the vehicle already hovering at the trajectory's origin and streams continuously; a canned/self-triggered maneuver needs an explicit "wait until at x0/y0 (or offset the trajectory relative to entry position)" step, e.g. capture `stateGetPositionEnu_f()` in `flat_traj_start()` and add it as an offset to the array.
- The vertical side has the analogous entry via `guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED)` (`autopilot_static.c:326`) — same current-altitude-seeding concern for `z0`.
- `AP_MODE_GUIDED` can only be reached from states where `autopilot_static.c:207` allows it (`mode == AP_MODE_NAV || mode == AP_MODE_GUIDED`-style guard plus in-flight/motors-on checks) — a flight-plan block, unlike a raw RC switch, can add its own `<exception>` safety gates (geofence, min altitude, in-flight check) before/while running the maneuver, which is harder to express cleanly as a bare RC threshold check.
- Your existing datalink parser is gated on `#ifdef AP_MODE_GUIDED && autopilot_get_mode() == AP_MODE_GUIDED` for every packet; if you reuse `flat_traj_run()` inside a flight-plan `<call>`, add the same guard (or rely on the block's own `deroute` exception) so a stray call after mode changes externally doesn't keep clobbering `guidance_h.sp`.
