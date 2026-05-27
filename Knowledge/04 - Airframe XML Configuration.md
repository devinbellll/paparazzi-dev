# Airframe XML Configuration

## File Location

```
paparazzi/conf/airframes/ENAC/quadrotor/anton_indi_aruco.xml
```
Registered in `paparazzi/conf/airframes/ENAC/conf_enac.xml`.

## Top-Level Structure

```xml
<airframe name="Quadricopter ANTON Tawaki">
  <firmware name="rotorcraft">
    <target name="ap" board="tawaki_1.0">...</target>
    <module name="stabilization" type="indi">...</module>
    <module name="guidance" type="indi"/>
    <module name="ins" type="ekf2"/>
    ...
  </firmware>
  <servos driver="DShot">...</servos>
  <commands>...</commands>
  <command_laws>...</command_laws>
  <section name="IMU" ...>...</section>
  <section name="STABILIZATION_ATTITUDE_INDI" ...>...</section>
  <section name="GUIDANCE_INDI" ...>...</section>
  <section name="GUIDANCE_V" ...>...</section>
  <section name="GUIDANCE_H" ...>...</section>
</airframe>
```

## Module Selection

The `type` attribute picks which variant is compiled.

```xml
<module name="stabilization" type="indi"/>   <!-- → stabilization_indi.c -->
<module name="stabilization" type="int_quat"/> <!-- → classical PID -->
<module name="guidance"      type="indi"/>   <!-- → guidance_indi_quadrotor.c -->
```

The module XML file at `conf/modules/stabilization_indi.xml` maps this to the actual `.c` files added to the Makefile.

## `<configure>` vs `<define>`

| Tag | Effect | When resolved |
|-----|--------|--------------|
| `<configure name="X" value="V"/>` | Sets a Makefile variable | Build time (Makefile) |
| `<define name="X" value="V"/>` | Adds `-DX=V` to CFLAGS | Compile time (C preprocessor) |

For INDI sizing, these must appear inside the `<module>` tag:
```xml
<module name="stabilization" type="indi">
  <define name="WLS_N_U_MAX" value="4"/>
  <define name="WLS_N_V_MAX" value="4"/>
</module>
```

## INDI Stabilization Section

```xml
<section name="STABILIZATION_ATTITUDE_INDI" prefix="STABILIZATION_INDI_">

  <!-- Outer attitude PD gains -->
  <define name="REF_ERR_P"   value="101"/>   <!-- kp roll  -->
  <define name="REF_ERR_Q"   value="101"/>   <!-- kp pitch -->
  <define name="REF_ERR_R"   value="124"/>   <!-- kp yaw   -->
  <define name="REF_RATE_P"  value="12.6"/>  <!-- kd roll  -->
  <define name="REF_RATE_Q"  value="14.0"/>  <!-- kd pitch -->
  <define name="REF_RATE_R"  value="14.0"/>  <!-- kd yaw   -->

  <!-- Angular acceleration filter (Hz) -->
  <define name="FILT_CUTOFF"   value="4.0"/>
  <define name="FILT_CUTOFF_R" value="4.0"/>

  <!-- G1: control effectiveness [outputs × actuators] -->
  <!-- rows: roll / pitch / yaw / thrust -->
  <!-- cols: FR / BR / BL / FL           -->
  <define name="G1" type="matrix">
    <field value="{-40, -40, 40, 40}"/>    <!-- roll    -->
    <field value="{ 40, -40,-40, 40}"/>    <!-- pitch   -->
    <field value="{  5,  -5,  5, -5}"/>    <!-- yaw     -->
    <field value="{-1.5,-1.5,-1.5,-1.5}"/> <!-- thrust  -->
  </define>

  <!-- G2: propeller inertia per actuator -->
  <define name="G2" value="{150, -150, 150, -150}"/>

  <!-- Actuator first-order bandwidth (rad/s) -->
  <define name="ACT_FREQ" value="{30.5, 30.5, 30.5, 30.5}"/>

  <!-- WLS priorities: roll, pitch, yaw, thrust -->
  <define name="WLS_PRIORITIES" value="{1000, 1000, 1, 100}"/>

  <!-- Map INDI outputs to COMMANDS_* enum values -->
  <define name="COMMANDS" value="{COMMAND_FR, COMMAND_BR, COMMAND_BL, COMMAND_FL}"/>

</section>
```

## INDI Guidance Section

```xml
<section name="GUIDANCE_INDI" prefix="GUIDANCE_INDI_">
  <!-- Must match ACT_FREQ above -->
  <define name="THRUST_DYNAMICS_FREQ" value="30.5"/>
</section>
```

## Vertical Guidance Section

```xml
<section name="GUIDANCE_V" prefix="GUIDANCE_V_">
  <define name="NOMINAL_HOVER_THROTTLE" value="0.30"/>
  <define name="REF_MIN_ZDD" value="-0.4*9.81"/>
  <define name="REF_MAX_ZDD" value=" 0.4*9.81"/>
  <define name="REF_MIN_ZD"  value="-1.5"/>
  <define name="REF_MAX_ZD"  value=" 1."/>
</section>
```

## Servo / Command Mapping

```xml
<servos driver="DShot">
  <servo name="FR" no="3" min="0" neutral="100" max="2000"/>
  <!-- ... -->
</servos>

<commands>
  <axis name="FR" failsafe_value="MOTOR_STOP"/>
  <!-- ... -->
</commands>

<command_laws>
  <set servo="FR" value="@FR"/>
  <!-- ... -->
</command_laws>
```

The `COMMANDS` define in the INDI section wires INDI outputs to these command axes.

## conf_enac.xml Entry

Each aircraft entry lists:
- `settings_modules` — XML panels loaded into the GCS for live tuning

```xml
settings_modules="... modules/stabilization_indi.xml ..."
```

The `modules/stabilization_indi.xml` settings file is in `conf/settings/modules/`, not `conf/modules/`.

## See Also

- [[02 - INDI Stabilization Deep Dive]]
- [[05 - Module System]]
- [[06 - Modifying ANTON Stabilization]]
