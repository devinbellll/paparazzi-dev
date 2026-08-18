# Session — 2026-08-18: making the flat trajectory work from the GCS

Two independent bugs, both fixed and both confirmed in flight from the GCS. Plus
the Safety geofence the flight plan had been asking for in a comment, and the
ANTON_MFC tuning ported to Hoops_111_MFC.

## Bug 1 — the RC switch was cancelling GUIDED

**Symptom (user-reported):** running the flat trajectory from the GCS, the
aircraft reverted to a Standby-like hold on its own, with the mode indicator
showing only a *momentary* flicker to GUIDED.

**Cause:** `autopilot_static_on_rc_frame()` re-derives the autopilot mode from the
RC 3-way switch on **every RC frame**. `autopilot_set_mode(AP_MODE_GUIDED)` from
the flight plan therefore survived one frame before being stamped back to
`AP_MODE_NAV` (AUTO2). The block kept running and kept calling
`guidance_h_set_flat()`, but `guidance_h.mode` was back to `GUIDANCE_H_MODE_NAV`,
so `guidance_h_from_nav()` served the setpoint from `nav.carrot` — the previous
block's leftover waypoint, i.e. Standby's. Hence "Standby screws with the flat
trajectory".

**There is no RC link in SITL**, which is why this reproduced only from the GCS.

**Fix:** stay in `AP_MODE_NAV` and set `nav.horizontal_mode =
NAV_HORIZONTAL_MODE_GUIDED` / `nav.vertical_mode = NAV_VERTICAL_MODE_GUIDED` every
tick in `nav_flat_traj_run()`. These are NAV's own sub-mode selectors, not
autopilot modes, so the RC switch cannot touch them, and they dispatch to the
*same* `guidance_h_guided_run()` / `guidance_v_guided_run()` runners.
Mirrors the stock `NavGuided()` macro.

**Confirmed:** `MODE/ap` constant, `MODE/nav_v` steps 2 → 3 at the trigger, no
flicker.

## Bug 2 — the flat reference ratcheted upward one LSB per tick

Fixing bug 1 exposed this; it was pre-existing and unrelated.

`gv_update_ref_from_flat_ref()` round-tripped `gv_z_ref` (Q37.26) through Q23.8
every tick. `BFP_OF_REAL` truncates toward zero, NED altitude is negative, so the
reference ratcheted **up a whole LSB (3.9 mm) per 2 ms tick** — a phantom
**+1.95 m/s** climb — snapped back at the nav rate into a **10 Hz sawtooth**.
MFC double-differentiates its setpoint, turning each snap into an `rddot` impulse
of ~1e4 m/s² that pinned the thrust command at its clamp. The vehicle climbed
~1.7 m past the endpoint and held there.

Full write-up: **`Knowledge/17 - The Flat Reference Fixed-Point Ratchet.md`**.

**Confirmed:** `sp_z` is now a clean flat line at the endpoint and the vehicle
sits on it. User: "the flat traj is working perfectly".

## Method notes worth keeping

- **I could not reproduce either bug in SITL.** Bug 1 needs an RC link, which
  SITL does not have. Hoops does not arm in the headless sandbox, and ANTON
  stopped arming too. Both bugs were found from **user-supplied GCS logs**, which
  is the right tool here — the sandbox is not a substitute.
- **I guessed wrong once and shipped it.** After reading the code without
  reproducing, I proposed three candidate mechanisms; the user picked the right
  one from operational experience. Lesson: when a bug is only observable on the
  real system, ask for the log first rather than reasoning from source.
- **Per-tick resolution is what found bug 2.** The sawtooth is invisible in a
  decimated plot — it averages to a clean line at the right value. Sampling the
  raw CSV at the 2 ms tick made the `+0.0039` (= 1/256) increments and the
  snap-backs obvious immediately.
- **Verify a fixed-point fix arithmetically before flying it.** A standalone
  harness replicating the old code reproduced +1.95 m/s of phantom climb —
  matching the rate measured in the flight log to three significant figures —
  and showed the new code tracking truth to ~1e-7 m. That was available in
  seconds and did not need an aircraft.
- **A regression I caused and caught:** my first pass at bug 1 also carried
  three "improvements" to the z channel that each caused clamp-to-clamp limit
  cycles (see the 2026-08-17 session note). One knob per run on a channel that
  works.

## Also landed

- **Safety geofence** in `flat_traj_demo.xml`: `S1`–`S4` corners, a `<sectors>`
  Safety polygon, and `<exception cond="!InsideSafety(...)" deroute="Standby"/>`
  as the block's first element so it is evaluated every nav tick and fires
  mid-trajectory. 50×50 m about HOME — **resize per site**;
  `max_dist_from_home=150` remains the outer backstop.
- **Hoops_111_MFC** takes the ANTON_MFC tuning (attitude `Kd` 6→12, horizontal
  `Kp/Kd` 0.64/2.4 → 2/25, decoupled, window 600). Transferable because both run
  500 Hz with identical MODEL constants. **`GZ_MAX_THRUST` was restored to
  Hoops's own 0.7** — it had picked up ANTON's 1.5 during the port, which would
  have scaled every thrust command by 2.14× and left it unable to climb. That
  factor must never cross airframes.
  Not SITL-validated on Hoops; watch `ACT_FREQ` 15 vs 30.5, `MAX_BANK` 45 vs 20,
  `REF_MAX_ACCEL` 5.0 vs 2.5 on the first flight.

## Still open

- `mfc_core_mimo.c:56` still defaults `est_use_presat_command = true` — the same
  defect fixed in `mfc_core.c` on 2026-08-17. Left alone because nothing
  currently exercises the MIMO path.
- The horizontal `Kp/Kd` gap vs Simulink (2/25 against 75/150) is real and
  unexplained; firmware holds 4.5 cm at 5 % rail, so it was left alone.
- Bench-measure `MODEL_MASS` / `INERTIA_*` on both airframes.
