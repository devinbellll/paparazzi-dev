# Modifying ANTON's Stabilization — Worked Examples

## Before You Start: Understand What You're Changing

ANTON uses **full INDI** (`stabilization type="indi"`), not the simple variant.  
The control loop runs at **1000 Hz** (`PERIODIC_FREQUENCY=1000`).  
Actuators are 4× DShot motors on a Tawaki 1.0 board.

---

## Case 1: Tune Gains (No Recompile Needed)

The outer-loop PD gains can be changed **live via the GCS** without rebuilding.

### In the GCS

Open the settings panel: **Settings → indi** (from `stabilization_indi.xml` panel).

| GCS variable | Meaning | Safe starting range |
|---|---|---|
| `indi_gains.att.p` (kp_p/q/r) | Attitude error gain | 50–200 |
| `indi_gains.rate.p` (kd_p/q/r) | Rate damping | 5–25 |
| `stabilization_indi_filter_freq` | Actuator + rate filter cutoff Hz | 2–15 |

### To persist gains into firmware

Edit `paparazzi/conf/airframes/ENAC/quadrotor/anton_indi_aruco.xml`:

```xml
<section name="STABILIZATION_ATTITUDE_INDI" prefix="STABILIZATION_INDI_">
  <define name="REF_ERR_P"  value="NEW_VALUE"/>
  <define name="REF_ERR_Q"  value="NEW_VALUE"/>
  <define name="REF_ERR_R"  value="NEW_VALUE"/>
  <define name="REF_RATE_P" value="NEW_VALUE"/>
  <define name="REF_RATE_Q" value="NEW_VALUE"/>
  <define name="REF_RATE_R" value="NEW_VALUE"/>
</section>
```

Then rebuild: `./build_fw.sh ANTON paparazzi/conf/airframes/ENAC/conf_enac.xml`

---

## Case 2: Retune the G1 Matrix (Control Effectiveness)

The G1 matrix encodes how much each motor contributes to roll/pitch/yaw/thrust.  
Retune this when you change motor positions, props, or observe bad allocation.

**File to edit:**
```
paparazzi/conf/airframes/ENAC/quadrotor/anton_indi_aruco.xml
```

**Section to modify:**
```xml
<define name="G1" type="matrix">
  <field value="{-40, -40, 40, 40}"/>    <!-- roll: FR−, BR−, BL+, FL+ -->
  <field value="{ 40, -40,-40, 40}"/>    <!-- pitch -->
  <field value="{  5,  -5,  5, -5}"/>    <!-- yaw -->
  <field value="{-1.5,-1.5,-1.5,-1.5}"/> <!-- thrust (negative = up = -Z body) -->
</define>
```

**How to derive new values:**
1. Measure angular acceleration response per motor (system ID)
2. Values are in units of rad/s² per normalized motor step (scaled by `INDI_G_SCALING=1000`)
3. Signs follow right-hand rule in body frame (X=forward, Y=left, Z=up for paparazzi)

After changing G1, rebuild and verify the adaptive estimator converges (`USE_ADAPTIVE=TRUE` helps during initial tuning).

---

## Case 3: Change the Actuator Dynamics Model

The `ACT_FREQ` parameter tells INDI how fast the motors respond. Wrong values cause INDI to over/under-shoot.

```xml
<define name="ACT_FREQ" value="{30.5, 30.5, 30.5, 30.5}"/>
```

This is the first-order corner frequency in rad/s. To measure it:
1. Apply a step input and measure the 63% rise time `τ`
2. `ACT_FREQ = 1/τ`

Also update `THRUST_DYNAMICS_FREQ` in `GUIDANCE_INDI` to match:
```xml
<section name="GUIDANCE_INDI" prefix="GUIDANCE_INDI_">
  <define name="THRUST_DYNAMICS_FREQ" value="30.5"/>
</section>
```

---

## Case 4: Change the Stabilization Algorithm Entirely

To replace INDI with classical PID (e.g. for debugging):

**In the airframe XML**, change:
```xml
<!-- FROM -->
<module name="stabilization" type="indi">
  <define name="WLS_N_U_MAX" value="4"/>
  <define name="WLS_N_V_MAX" value="4"/>
</module>

<!-- TO -->
<module name="stabilization" type="int_quat"/>
```

And replace the `STABILIZATION_ATTITUDE_INDI` section with `STABILIZATION_ATTITUDE` gains (kp, ki, kd).

**Note:** Also remove the `guidance type="indi"` and replace with `guidance type="pid"` or leave it (guidance_indi can still output setpoints to a PID stabilizer).

---

## Case 5: Write a Custom Stabilization Module

1. **Create the C source** — copy `stabilization_indi.c` as a starting point:
   ```
   sw/airborne/firmwares/rotorcraft/stabilization/stabilization_myctrl.c
   sw/airborne/firmwares/rotorcraft/stabilization/stabilization_myctrl.h
   ```
   You must implement the interface:
   ```c
   void stabilization_myctrl_init(void);
   void stabilization_myctrl_enter(void);
   void stabilization_myctrl_run(bool in_flight,
       struct StabilizationSetpoint *sp,
       struct ThrustSetpoint *thrust,
       int32_t *cmd);
   ```

2. **Create the module XML** at `conf/modules/stabilization_myctrl.xml`:
   ```xml
   <module name="stabilization_myctrl" dir="stabilization" task="control">
     <dep>
       <depends>stabilization_rotorcraft,@attitude_command</depends>
       <provides>commands</provides>
     </dep>
     <init fun="stabilization_myctrl_init()"/>
     <makefile target="ap|nps" firmware="rotorcraft">
       <file name="stabilization_myctrl.c" dir="$(SRC_FIRMWARE)/stabilization"/>
     </makefile>
   </module>
   ```

3. **Wire it in the airframe XML**:
   ```xml
   <module name="stabilization" type="myctrl"/>
   ```

4. **Rebuild**: `./build_fw.sh ANTON paparazzi/conf/airframes/ENAC/conf_enac.xml`

---

## Case 6: Modify WLS Priorities

To make yaw more aggressive at the expense of thrust variation:

```xml
<!-- default: roll=1000, pitch=1000, yaw=1, thrust=100 -->
<define name="WLS_PRIORITIES" value="{1000, 1000, 100, 100}"/>
```

Higher number = higher priority = the WLS will sacrifice other axes less for this axis.

---

## Build, Flash, and Verify

```bash
# Build
./build_fw.sh ANTON paparazzi/conf/airframes/ENAC/conf_enac.xml

# Output ELF
paparazzi/var/aircrafts/ANTON/ap/obj/ap.elf

# Flash via GCS or paparazzi uploader
```

The build succeeds = firmware is valid. Correctness requires flight test.

## See Also

- [[02 - INDI Stabilization Deep Dive]] — algorithm details
- [[04 - Airframe XML Configuration]] — full XML reference
- [[07 - All Touch Points Cheatsheet]] — quick file checklist
