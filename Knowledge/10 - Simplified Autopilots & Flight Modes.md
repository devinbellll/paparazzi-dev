# 10 - Simplified Autopilots & Flight Modes

## Overview

Paparazzi supports two autopilot strategies for rotorcraft:

| Strategy | Header selected | Mode values |
|---|---|---|
| **Static** (default) | `autopilot_static.h` | Fixed (KILL=0, FAILSAFE=1, HOME=2, RATE_DIRECT=3, ATTITUDE_DIRECT=4, ..., NAV=13) |
| **Generated** (custom XML) | `autopilot_core_ap.h` (generated) | Sequential from 0 in XML order |

The generated strategy is activated by placing `<autopilot name="..."/>` at the `<firmware>` level in the airframe XML.

---

## Custom Autopilot XML

File location: `paparazzi/conf/autopilot/<name>.xml`

### Minimal 4-mode rotorcraft example

```xml
<autopilot name="MY Autopilot">
  <state_machine name="ap" freq="PERIODIC_FREQUENCY" gcs_mode="true"
                 settings_mode="true"
                 settings_handler="autopilot_generated|SetModeHandler">

    <modules>
      <module name="nav"          type="rotorcraft"/>
      <module name="guidance"     type="rotorcraft"/>
      <module name="stabilization" type="rotorcraft"/>
    </modules>

    <includes>
      <include name="generated/airframe.h"/>
      <include name="autopilot_rc_helpers.h"/>
      <include name="modules/radio_control/radio_control.h"/>
      <include name="modules/gps/gps.h"/>
      <!-- add custom guidance headers here -->
      <define name="MODE_MANUAL" value="AP_MODE_ATTITUDE_DIRECT" cond="ifndef MODE_MANUAL"/>
      <define name="MODE_AUTO2"  value="AP_MODE_NAV"             cond="ifndef MODE_AUTO2"/>
    </includes>

    <exceptions>
      <exception cond="kill_switch_is_on()" deroute="KILL"/>
    </exceptions>

    <!-- Reusable blocks -->
    <control_block name="set_commands">
      <call fun="SetRotorcraftCommands(stabilization.cmd, autopilot_in_flight(), autopilot_get_motors_on())"/>
    </control_block>

    <mode name="ATTITUDE_DIRECT" shortname="ATT">
      <on_enter>
        <call fun="guidance_h_mode_changed(GUIDANCE_H_MODE_NONE)"/>
        <call fun="guidance_v_mode_changed(GUIDANCE_V_MODE_RC_DIRECT)"/>
        <call fun="stabilization_mode_changed(STABILIZATION_MODE_ATTITUDE, STABILIZATION_ATT_SUBMODE_HEADING)"/>
      </on_enter>
      <control>
        <call fun="guidance_v_run(autopilot_in_flight())" store="struct ThrustSetpoint thrust_sp"/>
        <call fun="stabilization_run(autopilot_in_flight(), &stabilization.rc_sp, &thrust_sp, stabilization.cmd)"/>
        <call_block name="set_commands"/>
      </control>
      <exception cond="RadioControlIsLost()" deroute="FAILSAFE"/>
    </mode>

    <mode name="NAV">
      <select cond="$DEFAULT_MODE"/>
      <on_enter>
        <call fun="guidance_h_mode_changed(GUIDANCE_H_MODE_NAV)"/>
        <call fun="guidance_v_mode_changed(GUIDANCE_V_MODE_NAV)"/>
        <call fun="stabilization_mode_changed(STABILIZATION_MODE_ATTITUDE, STABILIZATION_ATT_SUBMODE_HEADING)"/>
      </on_enter>
      <control freq="NAVIGATION_FREQUENCY">
        <call fun="nav_periodic_task()"/>
      </control>
      <control>
        <!-- call your guidance here -->
        <call_block name="set_commands"/>
      </control>
      <exception cond="GpsIsLost() && autopilot_in_flight()" deroute="FAILSAFE"/>
    </mode>

    <mode name="FAILSAFE" shortname="FAIL">
      <on_enter>
        <call fun="guidance_h_mode_changed(GUIDANCE_H_MODE_NONE)"/>
        <call fun="guidance_v_mode_changed(GUIDANCE_V_MODE_CLIMB)"/>
        <call fun="stabilization_mode_changed(STABILIZATION_MODE_ATTITUDE, STABILIZATION_ATT_SUBMODE_HEADING)"/>
        <call fun="guidance_v_set_vz(FAILSAFE_DESCENT_SPEED)"/>
      </on_enter>
      <control>
        <call fun="stabilization_get_failsafe_sp()" store="struct StabilizationSetpoint stab_failsafe"/>
        <call fun="guidance_v_run(autopilot_in_flight())" store="struct ThrustSetpoint thrust_sp"/>
        <call fun="stabilization_run(autopilot_in_flight(), &stab_failsafe, &thrust_sp, stabilization.cmd)"/>
        <call_block name="set_commands"/>
      </control>
      <exception cond="!GpsIsLost()" deroute="$LAST_MODE"/>
    </mode>

    <mode name="KILL">
      <select cond="kill_switch_is_on()"/>
      <on_enter>
        <call fun="guidance_h_mode_changed(GUIDANCE_H_MODE_NONE)"/>
        <call fun="stabilization_mode_changed(STABILIZATION_MODE_NONE, 0)"/>
        <call fun="guidance_v_mode_changed(GUIDANCE_V_MODE_KILL)"/>
        <call fun="autopilot_set_in_flight(false)"/>
        <call fun="autopilot_set_motors_on(false)"/>
      </on_enter>
      <control>
        <call fun="SetCommands(commands_failsafe)"/>
      </control>
    </mode>

  </state_machine>
</autopilot>
```

---

## Wiring the Custom Autopilot

### In the airframe XML

```xml
<firmware name="rotorcraft">
  <autopilot name="my_autopilot.xml"/>
  <!-- rest of modules -->
</firmware>
```

The generator creates:
- `var/aircrafts/<AC>/ap/generated/autopilot_core_ap.h`
- `var/aircrafts/<AC>/nps/generated/autopilot_core_nps.h` (or `simsitl`)

### Mode constants

Modes are numbered 0, 1, 2, … in XML declaration order. In the ANTON_MFC autopilot:

```
AP_MODE_ATTITUDE_DIRECT = 0
AP_MODE_NAV             = 1
AP_MODE_FAILSAFE        = 2
AP_MODE_KILL            = 3
```

These are defined in `autopilot_core_ap.h` and override the static header values.

---

## Stack-Switch Pattern (ANTON_MFC)

Two `<control_block>` elements define alternative guidance stacks. Only one is called from NAV; swap the `<call_block name="..."/>` line and rebuild.

```xml
<!-- Stock INDI guidance → MFC stabilizer -->
<control_block name="run_indi_stack">
  <call fun="guidance_v_run(autopilot_in_flight())"         store="struct ThrustSetpoint thrust_sp"/>
  <call fun="guidance_h_run(autopilot_in_flight())"         store="struct StabilizationSetpoint stab_sp"/>
  <call fun="stabilization_run(autopilot_in_flight(), &stab_sp, &thrust_sp, stabilization.cmd)"/>
</control_block>

<!-- Pure MFC guidance → MFC stabilizer -->
<control_block name="run_mfc_stack">
  <call fun="guidance_mfc_run_vert(autopilot_in_flight(), &guidance_v)"                  store="struct ThrustSetpoint thrust_sp"/>
  <call fun="guidance_mfc_run_horiz(autopilot_in_flight(), &guidance_h, &thrust_sp)"     store="struct StabilizationSetpoint stab_sp"/>
  <call fun="stabilization_run(autopilot_in_flight(), &stab_sp, &thrust_sp, stabilization.cmd)"/>
</control_block>

<!-- In NAV mode control: -->
<call_block name="run_mfc_stack"/>   <!-- swap to run_indi_stack to use INDI -->
```

---

## Gotchas

### Makefile.ac caching bug

The generator only overwrites `Makefile.ac` if it is older than the airframe XML. If you run an NPS build first and then an AP build without changing any XML, the AP build reuses the NPS-generated `Makefile.ac`, which has `USE_GENERATED_AUTOPILOT = TRUE` only in the NPS block. Result: `autopilot_static.c` is compiled alongside the generated header → duplicate case value errors.

**Fix:** Delete `var/aircrafts/<AC>/Makefile.ac` before switching build targets:

```bash
rm paparazzi/var/aircrafts/ANTON_MFC/Makefile.ac
./build_fw.sh ANTON_MFC conf/airframes/ENAC/conf_enac.xml ap
```

### `&&` in exception conditions

Use plain `&&` in `cond=` strings, not `&amp;&amp;`. The autopilot code generator outputs the string verbatim into C; `&amp;&amp;` becomes `&amp;&amp;` in the generated C code which is a parse error.

### `&` address-of in `<call fun=...>`

In `<call fun="...&ptr...">` pass-by-pointer arguments, use `&` not `&amp;`. Same reason — the generator copies the string to C verbatim.

---

## Mode Switch via GCS

With a custom autopilot, the GCS "Mode" panel uses `AP_MODE_*` names from `autopilot_core_ap.h`. The `SetModeHandler` (wired via `settings_handler`) accepts the integer value over datalink.

The airframe XML `MODE_MANUAL`, `MODE_AUTO1`, `MODE_AUTO2` defines must match values that exist in the custom autopilot (not the static header values).

---

## Related files

- `conf/autopilot/anton_mfc_autopilot.xml` — ANTON_MFC custom autopilot
- `sw/airborne/firmwares/rotorcraft/autopilot_static.h` — fixed mode constants (not used when generated autopilot is active)
- `sw/airborne/firmwares/rotorcraft/autopilot_generated.c` — generic generated-autopilot dispatch (included when `USE_GENERATED_AUTOPILOT=1`)
- `sw/lib/ocaml/aircraft.ml` — sets `autopilot = true` flag per target during code generation
- `sw/tools/generators/gen_makefile.ml` — writes `USE_GENERATED_AUTOPILOT = TRUE` inside target block when flag is set
