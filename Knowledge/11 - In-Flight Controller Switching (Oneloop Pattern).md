# 11 - In-Flight Controller Switching (Oneloop Pattern)

How Paparazzi switches between two control laws in flight, traced end-to-end from
the autopilot XML through the OCaml code generator to the runtime C. This is the
canonical precedent for "run two controllers and switch between them," and it
defines both what that phrase **does** mean in Paparazzi and what it **does not**.

Reference files:
- `paparazzi/conf/autopilot/rotorcraft_oneloop_switch.xml` — the switch state machine
- `paparazzi/conf/airframes/tudelft/rotwing_v3c_oneloop_simulation.xml` — an airframe that uses it
- `paparazzi/sw/tools/generators/gen_autopilot.ml` — the XML→C generator
- `paparazzi/sw/airborne/firmwares/rotorcraft/oneloop/oneloop_andi.c/.h` — the dual law
- `paparazzi/sw/airborne/firmwares/rotorcraft/autopilot_rc_helpers.h` — mode/RC switch macros
- `paparazzi/sw/airborne/firmwares/rotorcraft/autopilot_utils.{h,c}` — `set_rotorcraft_commands`

---

## 1. The autopilot XML is a code generator input, not a runtime config

The airframe selects an autopilot core with `<autopilot name="rotorcraft_oneloop_switch.xml"/>`
inside `<firmware name="rotorcraft">`. At build time `gen_autopilot.ml` reads that
file and emits `generated/autopilot_core_ap.h`, compiled in by `autopilot_generated.c`
(selected by `USE_GENERATED_AUTOPILOT`). Nothing in the XML is parsed at runtime.

### How each XML element becomes C (gen_autopilot.ml)

| XML | Generated C |
|-----|-------------|
| `<call fun="F"/>` | `F;` |
| `<call fun="F" store="struct T v"/>` | `v = F;` + hoists `struct T v;` to top of periodic task |
| `<call fun="F" cond="C"/>` | `if (C) { [store] F; }` |
| `<control_block name="X">…</control_block>` | reusable macro, **not** emitted inline |
| `<call_block name="X"/>` | expands to every `<call>` inside control_block `X` |
| `<control freq="N">…</control>` | prescaler wrapper: `if (prescaler >= MAIN_FREQ/N) {…}` (RunOnceEvery) |
| `<control>…</control>` (no freq) | runs every periodic tick |
| `<mode name="M">` | a `case AP_MODE_M:` in `switch(private_autopilot_mode_ap)` |
| `<on_enter>` | runs once on entry, inside `autopilot_core_ap_set_mode()` |
| `<select cond="C" exception="E"/>` | `if (C) { return AP_MODE_M; }` in `mode_select()` |
| `<exception cond="C" deroute="D"/>` | `if (C) { return AP_MODE_D; }` in `mode_exceptions()` |

The generated periodic task each tick does, in order:
1. `mode = mode_select()` — evaluate every mode's `<select cond>` (RC/GCS driven)
2. `mode = mode_exceptions(mode)` then `global_exceptions(mode)`
3. `set_mode(mode, FALSE)` — if mode changed, run old mode's `on_exit` then new mode's `on_enter`
4. `switch(private_autopilot_mode_ap)` — run the selected mode's `<control>` body

So the `store=`/`call_block` plumbing is pure textual codegen — `store` values are
ordinary locals passing between sequential `<call>`s within one tick.

---

## 2. The switch is ONE law at a time, re-linearized on entry — not two in parallel

`oneloop_andi` compiles **both** ANDI and INDI into a single module sharing all
filters and state. A single integer selects which math runs:

```c
#define CTRL_ANDI 0
#define CTRL_INDI 1
oneloop_andi.ctrl_type;            // the live selector
void oneloop_andi_enter(bool half_loop_sp, int ctrl_type); // stores type + rebuilds G
```

The two NAV-class modes differ **only** in their `<on_enter>`:

| Mode (id) | `<on_enter>` | Effect |
|-----------|--------------|--------|
| `NAV` (13) | `oneloop_andi_enter(false, CTRL_INDI)` | `ctrl_type = 1` |
| `MODULE` (17) | `oneloop_andi_enter(false, CTRL_ANDI)` | `ctrl_type = 0` |

`oneloop_andi_enter()` (oneloop_andi.c:1455) stores `ctrl_type` then calls
`G1G2_oneloop(ctrl_type)` to **rebuild the effectiveness matrix** for that law, so
entry re-linearizes around the current state → bumpless handover. The single shared
`oneloop_andi_run()` then branches on `ctrl_type` everywhere it matters:
- `G1G2_oneloop()` (1869): ANDI scales effectiveness by `act_dynamics[i]`, INDI does not
- guidance error controller (1695): ANDI `ec_3rd_pos` with jerk + `k_pos_e`; INDI `ec_3rd` zero-jerk + `k_pos_e_indi`
- attitude EC (1708) and yaw feed-forward g2 (1677) likewise branch

**Only the selected law executes each tick.** There is no shadow/compare-against-truth.
The other law is dormant; switching is just flipping the int and re-entering.

---

## 3. What triggers the switch (RC + GCS)

Mode selection runs every tick from the generated `mode_select()`:

```xml
<mode name="NAV">    <select cond="RCMode2() && RCAP2() && DLModeNav()"    exception="HOME"/>
<mode name="MODULE"> <select cond="RCMode2() && RCAP0() && DLModeModule()" exception="HOME"/>
```

Decoded via `autopilot_rc_helpers.h` (`rc_mode_switch(chan,pos,3)` buckets one RC
channel into 3 positions with hysteresis):
- `RCMode2()` = main `RADIO_MODE` switch in the AUTO2 (top) position
- `RCAP2()` / `RCAP0()` = a **second** channel `AP_MODE_SWITCH`. The rotwing airframe
  sets `AP_MODE_SWITCH = RADIO_AUX7` → **AUX7 high = INDI (NAV), AUX7 low = ANDI (MODULE)**
- `DLModeNav()` / `DLModeModule()` = the GCS datalink dropdown (`autopilot_mode_auto2`,
  a `<dl_setting>` in the autopilot `<settings>`)

All three must agree. Flip AUX7 mid-flight → next tick `mode_select()` returns the
other mode → `set_mode()` runs its `on_enter` → `oneloop_andi_enter()` re-arms with
the new `ctrl_type`. That is the entire in-flight handover.

---

## 4. The actuator-commit chokepoint

`stabilization.cmd[]` is committed to the global `commands[]` via
`SetRotorcraftCommands(cmd, in_flight, motors_on)` (a `<control_block>` in the XML),
which expands to:

```c
#define SetRotorcraftCommands(_cmd,_if,_mo) set_rotorcraft_commands(commands,_cmd,_if,_mo)
void WEAK set_rotorcraft_commands(pprz_t *cmd_out, int32_t *cmd_in, bool in_flight, bool motors_on);
```

`set_rotorcraft_commands()` (autopilot_utils.c:123) is **`WEAK`** — it can be
overridden to choose between two command buffers before they reach `commands[]`.
This is the single cleanest interception point for a true shadow/handover design
(see Plans). The oneloop pattern does **not** use it (oneloop guidance commits its
own commands; the NAV mode body doesn't even call `set_commands`).

---

## 5. Implication for MFC vs INDI on ANTON

The oneloop pattern works because ANDI and INDI live in **one module sharing one
symbol set**, so a single `enter(type)` re-linearizes cleanly. MFC and INDI are
**two separate modules with colliding global symbols** (`g1g2`, `actuators_pprz`,
`act_is_servo`, `stab_thrust_filt`, …) and both `<provides>commands</provides>` /
`guidance,attitude_command`. You cannot naively compile them together — that is the
core problem both dual-controller plans must solve first. See:
- `Knowledge/Plans/Dual-Controller Shadow Mode.md`
- `Knowledge/Plans/Dual-Controller Handover Mode.md`
