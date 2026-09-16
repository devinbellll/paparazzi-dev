# Plan: Canned flat-trajectory demo (self-triggered, no external stream)

## Context

The [[Flatness Trajectory Setpoints (Pos-Vel-Accel-Jerk-Snap + Psi)]] feature (built on
`feature-diff-flatness` in `paparazzi/`) added `guidance_h_set_flat()` /
`guidance_v_set_flat()` plus a Taylor-extrapolating reference model, but the only way to
drive it today is the new `GUIDED_TRAJECTORY_NED` datalink message — i.e. an external
companion computer/GCS has to stream packets. The user wants to bench/sim-test the feature
**without** that external stream: a small built-in step maneuver (1.5 s, x0→xf, +1 m x, +1 m
y, +1 m up) whose data lives in a header-file array they'll hand-fill themselves (I'm asked
for stubs only, not real polynomial coefficients), plus a self-contained way to trigger it
in flight.

Research (Explore agent, read-only) found the exact precedent to mirror:
`conf/flight_plans/ENAC/fish_outdoor.xml`'s `Guided_run` block —
```xml
<block group="fish" name="Guided_run" strip_button="Guided run">
  <exception cond="!InsideSafety(GetPosX(),GetPosY())" deroute="Standby"/>
  <call_once fun="autopilot_set_mode(AP_MODE_GUIDED)"/>
  <call fun="nav_fish_velocity_run()"/>
</block>
```
backed by `sw/airborne/modules/nav/nav_fish.c` (`nav_fish_velocity_run()` calls
`autopilot_guided_update()` every nav tick) and `conf/modules/nav_fish.xml`. This is a
flight-plan block that (a) needs no companion computer, (b) gets a GCS strip button and
RC-switch/exception triggerability for free, and (c) calls into guidance every tick exactly
like the datalink parser does per packet. Chosen over a `dl_setting`-only trigger
(`sys_id_chirp.xml` precedent) because a dl_setting handler doesn't manage the
`AP_MODE_GUIDED` transition itself — the flight-plan block gets that via `<call_once>`.

User decisions (via AskUserQuestion):
- **Table format**: fixed-rate time-sampled array — each row a full flat state
  (pos/vel/accel/jerk/snap + heading terms), matching `guidance_h_set_flat()`/
  `guidance_v_set_flat()`'s argument list exactly, indexed by elapsed time since trigger (not
  by call count — the flight-plan `<call>` loop runs at the ~4 Hz nav rate, far slower than
  the array's intended sample rate, so indexing must be time-based; this also means the demo
  exercises the same sparse-update → Taylor-extrapolation path a real external streamer
  would, which is the point of the test).
- **Trigger location**: a new standalone flight plan, not wired into any operational
  aircraft's mission — safe to iterate on.

## Implementation

### 1. Data stub — `sw/airborne/modules/nav/flat_traj_demo_data.h`
Header-only (per request), included by exactly one TU (`nav_flat_traj.c`), so plain
`static const` is safe (no ODR issue).
```c
#define FLAT_TRAJ_DEMO_DURATION   1.5f
#define FLAT_TRAJ_DEMO_DT         0.02f   // 50 Hz; user may change
#define FLAT_TRAJ_DEMO_NB_SAMPLES ((int)(FLAT_TRAJ_DEMO_DURATION / FLAT_TRAJ_DEMO_DT) + 1)
#define FLAT_TRAJ_DEMO_ORDER      2       // pos/vel/accel populated below; bump to 3/4 once jerk/snap are filled in

/** One row = flat state at t = i * FLAT_TRAJ_DEMO_DT, RELATIVE to the position/heading
 * captured when the maneuver is triggered (nav_flat_traj_start() adds the offset). NED
 * frame: x=north, y=east, z=down (so "+1m up" is z = -1.0f).
 * TODO(user): replace with the real 1.5s step (x0->xf: +1m x, +1m y, +1m up == z=-1). */
struct FlatTrajSample {
  float x, y, z;
  float vx, vy, vz;
  float ax, ay, az;
  float jx, jy, jz;
  float sx, sy, sz;
  float heading, heading_rate, heading_accel, heading_jerk, heading_snap;
};

static const struct FlatTrajSample flat_traj_demo_samples[FLAT_TRAJ_DEMO_NB_SAMPLES] = {0};
// TODO(user): fill in — e.g. flat_traj_demo_samples[N] = { .x = ..., .vx = ..., ... };
// index 0 = t=0 (should be all-zero: start of the step), last index = t=1.5s (== xf offset).
```
`= {0}` is a valid, portable "zero the whole aggregate" initializer so the stub compiles
as-is (a held-position no-op) before the user fills it in.

### 2. Playback module — `sw/airborne/modules/nav/nav_flat_traj.c` / `.h`
Mirrors `nav_fish.c`'s structure. Key logic: time-indexed lookup (not a per-call counter), a
captured origin offset, and reuse of the exact `guidance_h_set_flat`/`guidance_v_set_flat`
signatures built in the previous session (`firmwares/rotorcraft/guidance/guidance_h.h`,
`guidance_v.h`).

```c
static uint32_t flat_traj_idx;
static bool flat_traj_done;
static float flat_traj_start_time;
static struct FloatVect3 flat_traj_origin;      // NED, captured at trigger
static float flat_traj_origin_heading;

void nav_flat_traj_init(void) { flat_traj_done = true; }

void nav_flat_traj_start(void)
{
  struct NedCoor_f *pos = stateGetPositionNed_f();
  flat_traj_origin.x = pos->x;
  flat_traj_origin.y = pos->y;
  flat_traj_origin.z = pos->z;
  flat_traj_origin_heading = stateGetNedToBodyEulers_f()->psi;
  flat_traj_start_time = get_sys_time_float();
  flat_traj_idx = 0;
  flat_traj_done = false;
}

bool nav_flat_traj_run(void)
{
  if (flat_traj_done) { return false; }

  float t = get_sys_time_float() - flat_traj_start_time;
  uint32_t idx = (uint32_t)(t / FLAT_TRAJ_DEMO_DT);
  if (idx >= FLAT_TRAJ_DEMO_NB_SAMPLES) {
    idx = FLAT_TRAJ_DEMO_NB_SAMPLES - 1;
    flat_traj_done = true;
  }
  const struct FlatTrajSample *s = &flat_traj_demo_samples[idx];

  guidance_h_set_flat(
      flat_traj_origin.x + s->x, flat_traj_origin.y + s->y,
      s->vx, s->vy, s->ax, s->ay, s->jx, s->jy, s->sx, s->sy,
      flat_traj_origin_heading + s->heading, s->heading_rate,
      s->heading_accel, s->heading_jerk, s->heading_snap,
      FLAT_TRAJ_DEMO_ORDER);
  guidance_v_set_flat(
      flat_traj_origin.z + s->z, s->vz, s->az, s->jz, s->sz,
      FLAT_TRAJ_DEMO_ORDER);

  return true;
}

bool nav_flat_traj_is_done(void) { return flat_traj_done; }
```
`nav_flat_traj_run()` returns `bool` so it can be called directly as
`<call fun="nav_flat_traj_run()"/>` (same convention as `nav_fish_velocity_run()`).

### 3. Module XML — `conf/modules/nav_flat_traj.xml`
```xml
<module name="nav_flat_traj" dir="nav" task="control">
  <doc><description>Canned flat-trajectory (pos/vel/accel/jerk/snap) demo maneuver, played back from a fixed-rate sample table without needing an external datalink stream.</description></doc>
  <dep><depends>@navigation</depends></dep>
  <header><file name="nav_flat_traj.h"/></header>
  <init fun="nav_flat_traj_init()"/>
  <makefile><file name="nav_flat_traj.c"/></makefile>
</module>
```

### 4. New standalone flight plan — `conf/flight_plans/ENAC/flat_traj_demo.xml`
Based on `conf/flight_plans/rotorcraft_basic.xml`'s minimal block set (Wait GPS / Holding
point / Start Engine / Takeoff / Standby / land / flare / landed), with the new block added
mirroring `fish_outdoor.xml`'s `Guided_run`:
```xml
<block name="Flat_Traj_Demo" strip_button="Run Flat Traj">
  <exception cond="!InsideSafety(GetPosX(),GetPosY())" deroute="Standby"/>
  <call_once fun="autopilot_set_mode(AP_MODE_GUIDED)"/>
  <call_once fun="nav_flat_traj_start()"/>
  <call fun="nav_flat_traj_run()"/>
  <exception cond="nav_flat_traj_is_done()" deroute="Standby"/>
</block>
```
`Standby` block calls `autopilot_set_mode(AP_MODE_NAV)` before `<stay wp="STDBY"/>` (matches
`fish_outdoor.xml`) so falling out of the demo returns to a normal held position instead of
staying latched in GUIDED.

**Not part of this change** (flagged, needs a user decision, not guessed): wiring this
flight plan into `conf/airframes/ENAC/conf_enac.xml`'s aircraft list requires an airframe
with `AP_MODE_GUIDED` already mapped to a mode switch — currently only
`hoops_111_indoor.xml` / `hoops_112_hinf_outdoor.xml` qualify (per prior research), and only
`Hoops_111` is currently a live aircraft entry. Swapping its `flight_plan=` attribute (or
adding a new aircraft entry) to point at `flat_traj_demo.xml` is left to the user so an
operational flight plan isn't silently replaced.

## Verification

1. Compile-check: `./pprz.sh build Hoops_111 ap` (or `nps`) after temporarily pointing its
   `flight_plan=` at the new file, to confirm the module/flight-plan/header all build clean
   (same bar used in the previous session — compile is the automated check here).
2. Once the user fills in `flat_traj_demo_data.h`: run in NPS sim, trigger the
   `Flat_Traj_Demo` block from the GCS strip button (or `AUTO1`/`AP_MODE_GUIDED` switch +
   block selection), and inspect `guidance_h.ref.pos/speed/accel` via the existing
   PlotJuggler pipeline (`pj_json_relay.py`) to confirm the smooth Taylor-extrapolated step
   — this reuses verification-plan step 2 from the original flatness-trajectory plan.
