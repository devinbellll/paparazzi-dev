# Control System Architecture

## The Four-Layer Signal Chain

Paparazzi separates autonomy into four layers. Each layer outputs a setpoint consumed by the layer below it.

```
┌─────────────────────────────────────────────────────────────┐
│  NAVIGATION  (flight plan / GCS waypoints)                  │
│  conf/flight_plans/*.xml                                    │
│  sw/airborne/firmwares/rotorcraft/navigation.c              │
│  Output: position/speed setpoints                           │
└──────────────────────────┬──────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────┐
│  GUIDANCE  (position/velocity → attitude command)           │
│  sw/airborne/firmwares/rotorcraft/guidance/guidance_indi.c  │
│  Output: StabilizationSetpoint  (attitude quaternion)       │
│        + ThrustSetpoint                                     │
└──────────────────────────┬──────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────┐
│  STABILIZATION  (attitude → actuator commands)              │
│  sw/airborne/firmwares/rotorcraft/stabilization/            │
│    stabilization_indi.c                                     │
│  Output: int32_t cmd[COMMANDS_NB]  (−9600 … +9600)         │
└──────────────────────────┬──────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────┐
│  ACTUATORS  (commands → ESC/servo signals)                  │
│  sw/airborne/modules/actuators/actuators_dshot.*            │
│  Output: PWM / DShot pulses                                 │
└─────────────────────────────────────────────────────────────┘
```

## Key Interface Types

Defined in `sw/airborne/firmwares/rotorcraft/stabilization.h`:

```c
struct StabilizationSetpoint  // attitude desired by guidance
struct ThrustSetpoint          // thrust desired by guidance
struct Stabilization           // holds mode, RC input, current sp, output cmd[]
```

The guidance layer calls:
```c
stabilization_run(in_flight, &sp, &thrust, cmd)
```
which dispatches to the active stabilization module (INDI, PID, etc.).

## Autopilot Mode Dispatch

`autopilot_static.c` holds the state machine for `AP_MODE_*`.  
Modes relevant to stabilization:

| Mode | Stab input source |
|------|------------------|
| `AP_MODE_ATTITUDE_DIRECT` | RC sticks directly |
| `AP_MODE_ATTITUDE_Z_HOLD` | RC for attitude, guidance_v for altitude |
| `AP_MODE_NAV` | Full guidance (guidance_indi) drives everything |

## Sensors and State

All sensor fusion output lives in `state.c` (`stateGetNedToBodyQuat_f()`, `stateGetBodyRates_f()`, etc.).  
The INS/EKF2 module writes to this; stabilization reads from it.

## See Also

- [[02 - INDI Stabilization Deep Dive]]
- [[03 - INDI Guidance Deep Dive]]
- [[04 - Airframe XML Configuration]]
