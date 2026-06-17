# NPS Simulation Telemetry — Firmware Logs in Python

How to access firmware-internal state from a Python script while running NPS simulation.  
Reference implementation: `/workspace/sim_anton.py`.

---

## Architecture: Two Message Paths

There are **two completely separate paths** by which messages reach the Ivy bus. Confusing them is the most common source of "I subscribed but got nothing."

```
┌─────────────────────────────────────────────────────────┐
│  NPS simsitl process                                    │
│                                                         │
│  ┌──────────────┐     ABI events      ┌──────────────┐  │
│  │  JSBSim FDM  │ ←────────────────── │  Firmware    │  │
│  │  (physics)   │ ──────────────────► │  (your C     │  │
│  └──────────────┘   actuator cmds     │   code)      │  │
│                                       └──────┬───────┘  │
│  ┌──────────────────────────────┐            │           │
│  │  nps_ivy.c display thread    │            │ UDP       │
│  │  IvySendMsg(NPS_* directly)  │            │ port 4242 │
│  └──────────────┬───────────────┘            │           │
└─────────────────┼──────────────────────────────┼────────┘
                  │ PATH A                        │ PATH B
                  ▼ (direct Ivy)                  ▼ (PPRZ binary)
           Ivy bus ◄─────────── link -udp ◄───────┘
                  │             (decodes binary,
                  │              broadcasts on Ivy)
                  ▼
           Python IvyBindMsg callbacks
```

### Path A — Direct Ivy (NPS truth messages)

`nps_ivy.c` calls `IvySendMsg(...)` directly. No binary encoding; no `link` needed.  
Messages: `NPS_RATE_ATTITUDE`, `NPS_POS_LLH`, `NPS_SPEED_POS`, `NPS_GYRO_BIAS`, `NPS_SENSORS_SCALED`, `NPS_WIND`.

### Path B — PPRZ binary UDP → `link` → Ivy (firmware telemetry)

The firmware's periodic telemetry system encodes messages as PPRZ binary frames and sends them over UDP to port 4242. The `link` process listens on that port, decodes the frames, and re-broadcasts them on Ivy.

Messages: everything registered with `register_periodic_telemetry()` — `STAB_ATTITUDE`, `STAB_MFC`, `ROTORCRAFT_STATUS`, `INS_EKF2`, etc.

**`link` must be running or Path B messages never appear on Ivy.** The GCS `server` process alone is not enough.

---

## Required Processes

Three processes must be running simultaneously:

| Process | Binary | Purpose | Key flag |
|---------|--------|---------|----------|
| `server` | `sw/ground_segment/tmtc/server` | GCS state aggregator (Ivy) | `-n` to suppress aircraft-not-found errors |
| `link` | `sw/ground_segment/tmtc/link` | UDP→Ivy bridge for PPRZ binary | **`-udp`** (mandatory for NPS) |
| `simsitl` | `var/aircrafts/AIRCRAFT/nps/simsitl` | NPS firmware + JSBSim | `--norc` in headless mode |

All three must connect to the same Ivy bus (default `127.255.255.255:2010`).

```python
PPRZ = "/workspace/paparazzi"
IVY_BUS = "127.255.255.255:2010"

server = subprocess.Popen([f"{PPRZ}/sw/ground_segment/tmtc/server", "-b", IVY_BUS, "-n"], ...)
link   = subprocess.Popen([f"{PPRZ}/sw/ground_segment/tmtc/link",   "-b", IVY_BUS, "-udp"], ...)
sim    = subprocess.Popen([f"{PPRZ}/var/aircrafts/AIRCRAFT/nps/simsitl", "--norc"], ...)
```

---

## Ivy Message Format

### Path A messages (NPS_*)

All values space-separated. First token is `AC_ID`, second is message name.

```
"217 NPS_RATE_ATTITUDE  p  q  r  phi  theta  psi"
```

### Path B messages (firmware telemetry)

Same prefix convention, but **`float[]` array fields are comma-separated within a single token** — no spaces inside an array.

```
"217 STAB_ATTITUDE  att_des  phi,theta,psi  ref_phi,ref_theta,ref_psi  ..."
                    ↑single  ↑one token, three values CSV
                    float
```

Single `float` fields remain space-separated individual tokens.  
`int32`, `uint8`, etc. are always individual tokens.

### Parsing float[] arrays in Python

```python
parts = msg.split()           # split on spaces → one token per field
att = [float(x) for x in parts[3].split(',')]  # split CSV token → list of floats
phi, theta, psi = att[0], att[1], att[2]
```

---

## Adding a New Telemetry Message

To expose a new firmware variable over telemetry, four files must change.

### Step 1 — Define the message in `messages.xml`

File: `sw/ext/pprzlink/message_definitions/v1.0/messages.xml`

Find a free ID (check with `grep 'id=' messages.xml | sort -t'"' -k4 -n | tail -5`).

```xml
<message name="MY_MSG" id="NNN">
  <field name="scalar_val"  type="float">description</field>
  <field name="array_val"   type="float[]">description</field>  <!-- [] = variable-length array -->
</message>
```

Use scalar `float` fields where possible — they are simpler to parse on the Python side. Use `float[]` only for naturally array-valued quantities (e.g. per-axis attitude).

### Step 2 — Register a send callback in firmware

File: `sw/airborne/firmwares/rotorcraft/stabilization/stabilization_MY_MODULE.c`

```c
#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_my_msg(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_MY_MSG(trans, dev, AC_ID,
                       &my_scalar,
                       3, my_array);   // 3 = array length, then pointer
}
#endif

void my_module_init(void)
{
  // ... existing init ...
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_MY_MSG, send_my_msg);
#endif
}
```

`pprz_msg_send_MY_MSG` is auto-generated from `messages.xml` when the firmware is built. The generated prototype is in `var/aircrafts/AIRCRAFT/nps/generated/` after the first build.

### Step 3 — Add to telemetry XML

File: `conf/telemetry/default_rotorcraft.xml`

Add to the `default` mode (and any other modes you want it in):

```xml
<message name="MY_MSG" period="0.04"/>   <!-- 25 Hz -->
```

Period is in seconds. Common choices: `0.04` (25 Hz), `0.1` (10 Hz), `0.25` (4 Hz).

### Step 4 — Subscribe in Python

```python
def on_my_msg(agent, msg):
    parts = msg.split()
    if len(parts) < 4:
        return
    scalar_val = float(parts[2])
    array_vals = [float(x) for x in parts[3].split(',')]

IvyBindMsg(on_my_msg, r"(\d+ MY_MSG .*)")
```

### Step 5 — Rebuild the NPS target

```bash
./pprz.sh build AIRCRAFT nps
```

Use the **absolute path** for the conf XML — relative paths fail when the build wrapper changes directory.

---

## Reference: STAB_ATTITUDE

Sent by `stabilization_indi.c` → `send_att_full_indi()`, registered in `stabilization_indi_init()`.  
Period: `0.25 s` in `default_rotorcraft.xml` default mode.

```
"AC_ID STAB_ATTITUDE  att_des  att[3]  att_ref[3]  rate[3]  rate_ref[3]  ang_acc[3]  ang_acc_ref[3]  jerk_ref  u"
 [0]   [1]            [2]      [3]     [4]          [5]      [6]          [7]         [8]             [9]       [10]
```

All `[3]` tokens are `phi,theta,psi` CSV. Units: radians.

Key diagnostic columns:

| Token | What it shows |
|-------|--------------|
| `att[3]` (`parts[3]`) | Actual aircraft attitude from state estimator |
| `att_ref[3]` (`parts[4]`) | Attitude setpoint being tracked |
| `ang_acc_ref[3]` (`parts[8]`) | INDI virtual command — zero means INDI is not outputting corrections |

---

## Reference: STAB_MFC

Sent by `stabilization_mfc.c` → `send_stab_mfc()`, registered in `stabilization_mfc_init()`.  
Period: `0.04 s` in `default_rotorcraft.xml` default mode (25 Hz).  
Only present in ANTON_MFC (AC_ID 218) — slot is NULL in the INDI binary.

```
"AC_ID STAB_MFC  sp_phi sp_theta sp_psi  me_phi me_theta me_psi  err_phi err_theta err_psi  fk_phi fk_theta fk_psi  cmd_phi cmd_theta cmd_psi  u0 u1 u2 u3"
 [0]   [1]       [2]    [3]      [4]     [5]    [6]      [7]     [8]     [9]       [10]     [11]   [12]     [13]    [14]    [15]      [16]     [17][18][19][20]
```

All fields are individual scalar floats (no CSV). Total: 21 tokens.  
Units: `sp/me/err` in radians; `fk/cmd` in rad/s²; `u[]` in pprz units (−9600 to +9600).

Key diagnostic columns:

| Field | What it shows |
|-------|--------------|
| `fk_phi/theta` | Ultra-local model F_k estimate — should be bounded, non-diverging |
| `err_phi/theta` | Attitude error; should shrink toward zero in flight |
| `u[]` | WLS motor outputs — all −9600 means no thrust commanded |

---

## Troubleshooting

**"Path B messages never arrive (STAB_ATTITUDE, STAB_MFC always zero)"**  
→ `link` is not running. Start it with `-udp` alongside `server`.

**"Message arrives but all values are 0.0 / parser returns immediately"**  
→ Check the `len(parts) < N` guard matches the actual token count.  
→ Run a quick sniff: `IvyBindMsg(lambda a,m: print(repr(m)), r"(\d+ MY_MSG .*)")` and count tokens.

**"float[] field parses to garbage or raises ValueError"**  
→ Arrays are comma-separated within one token. Use `parts[N].split(',')`, not `float(parts[N])`.

**"Message type not seen in sniff at all"**  
→ Verify `register_periodic_telemetry(...)` is called for that message in the firmware init function.  
→ Verify the message is in the telemetry XML `default` mode with a non-zero period.  
→ Verify the firmware was rebuilt after adding the message to `messages.xml`.  
→ Check the generated `var/aircrafts/AIRCRAFT/nps/generated/periodic_telemetry.h` — the message IDX must appear and the slot must be non-NULL at runtime.

**"server log shows `A/C 'NNN' not found`"**  
→ Normal when running headless. The `-n` flag suppresses the abort; messages still flow through `link` unaffected.

---

## See Also

- [[01 - Control System Architecture]] — where stabilization fits in the stack
- [[02 - INDI Stabilization Deep Dive]] — STAB_ATTITUDE field meanings
- [[07 - All Touch Points Cheatsheet]] — file checklist for adding new modules
