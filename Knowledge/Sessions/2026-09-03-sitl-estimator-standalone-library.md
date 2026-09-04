# 2026-09-03 — `sitl_estimator`: the SITL sensor + estimator chain as a standalone library

Part 1 of the S-function work. Goal: run `truth -> NPS sensors -> ECL EKF2 ->
estimate` outside paparazzi, with its own Makefile, no built airframe, no GSL.

Branch `sitl-estimator-lib` off `mfc-development` in the `paparazzi` submodule,
commit `5d876ddda`. New code only, under
`paparazzi/sw/simulator/sitl_estimator/`. `hoops_111_fmfc.xml` was not touched.

## What changed

Nothing in any existing NPS or ECL source. The library adds
`sw/simulator/sitl_estimator/` and compiles the shared sources as they stand.
Full detail is in that directory's `README.md`; this note records what was
*learned*, which is the part that outlives the code.

## What the repository turned out to look like

- **`sw/ext/ecl` compiles standalone.** `-std=c++14 -D__PAPARAZZI
  -DECL_STANDALONE`, with only `sw/ext/matrix` and `sw/include/std.h` on the
  include path. The exact file list is already written down, in
  `conf/modules/ins_ekf2.xml`.
- **All twelve `nps_sensor_*.c` / `nps_sensors*.c` compile with plain gcc**
  against a hand-written `generated/airframe.h` on the include path. The
  `#include "generated/airframe.h"` in those files does not have to be removed
  — it has to be *satisfied*.
- **`nps_sensors_utils.c` is the only NPS file that touches glib**, and only for
  six `GSList` calls. A ~60-line shim removes libglib as a dependency without
  editing it.
- **The NPS IMU calibration path is not the airframe's.** `imu_nps_init()`
  registers under `IMU_NPS_ID` (23); `hoops_111_fmfc.xml`'s `IMU_ACCEL_CALIB` /
  `IMU_GYRO_CALIB` carry ABI id 24 (mag: 3). `imu_get_gyro()` therefore hands the
  NPS sender a *separate, uncalibrated* slot, and the NPS-derived scale/neutral
  apply instead. **The flight calibration never reaches the SITL estimator
  input.** Worth remembering before anyone tunes against a SITL estimate and
  assumes the bench calibration was in the loop.
- **`nps_scope.c` logs `body_ecef_rotvel` as `TRUTH/p,q,r`, but
  `nps_sensor_gyro.c` reads `body_inertial_rotvel`.** Different quantity, earth
  rate apart. Anything replaying the scope CSV into the sensor harness inherits
  that as a bias.
- **Stock NPS is reproducible run-to-run, it just has no seed control.**
  `gsl_rng_alloc()` without `gsl_rng_set()` uses `gsl_rng_default_seed` = 0, and
  mt19937 maps 0 onto 4357 internally. Verified against libgsl rather than
  assumed: the replacement RNG is bit-identical over 100 k raw draws, 100 k
  gaussian draws at the default seed, and 2 000 draws at each of five seeds.

## The bug the frame test caught

`struct LlaCoor_d` is `{lat, lon, alt}`. The LTP origin was built with a
positional initialiser in `{lon, lat, alt}` order, so the entire library ran on
a transposed datum — and the NED→ECEF→NED round trip closed to 4e-10 m anyway,
because a consistently wrong transform is still invertible. Only the independent
geodetic assertions (north raises latitude, east raises longitude, −down raises
altitude and hmsl, 100 m NED = 100 m of ECEF displacement, ned(0,0,0) returns
the configured datum) caught it.

**Rule for next time:** designated initialisers for every pprz geodetic struct,
and never test a transform only against its own inverse.

## Acceptance 5 — the honest result

SITL runs fine here. `./sim.sh Hoops_111_FMFC --nav "Start Engine,Takeoff,+20,Standby,+10"`
gave 291 s at 500 Hz. Replayed through the library:

| channel | result |
|---|---|
| magnetometer | **exact**, all 416 718 samples |
| GPS hmsl | **exact**, all samples |
| GPS velocity | 6.2e-9 m/s rms |
| gyro | 1.83 BFP LSB rms |
| accel | 72.4 BFP LSB rms |
| EKF NED position | 0.0154 m rms, 0.0513 m max |
| EKF NED velocity | 0.0191 m/s rms |
| EKF yaw | 1.9 mrad rms |

Every **deterministic** channel matches bit-exactly. The magnetometer matching
to zero is the strongest single result here: it validates `ltp_h`, the
body-to-IMU matrix, the quaternion path, the BFP scaling and the params header
all at once.

The **stochastic** channels do not match, and the number says exactly why: the
accel residual is 72.4 LSB against a noise std dev of 51.2 LSB, and
`sqrt(2) x 51.2 = 72.4`. That is two independent realisations of the *same*
distribution — the noise model is right, the realisation is not. The RNG draw
sequence is not aligned with the SITL run's, and no small step offset fixes it
(swept ±6 steps).

Cause: the scope capture starts 4.46 s into the run and timestamps rows with
`fdm.time`, not the harness step count, so the number of `get_gaussian_noise()`
draws consumed before the first logged row is unrecoverable. Winding the
harness forward from t=0 on NPS's own 1 ms accumulated clock does not recover
it either.

**So acceptance 5 is not met as specified, and the blocker is the capture, not
the library.** The EKF numbers are a real result — an identically configured
EKF2 fed statistically identical sensors tracks the logged estimate to 1.5 cm —
but they are not a 1:1 proof and must not be described as one.

## What is needed next, and it needs a decision

A full-precision truth + sensor tap in the sim, written at every
`nps_sensors_run_step()`: the `NpsFdm` fields the sensors actually read (in
double, including `body_inertial_rotvel`, `pressure`, `ecef_*`, `lla_pos`,
`ltp_h`), the `struct NpsSensors` values, and the sim step index from 0.

That is one new file under `sw/simulator/nps/` — but wiring it into a build
means touching either the module set or the airframe, both out of scope for the
brief that produced this work. **It was not written. It needs approval first.**

Part 2 (the Simulink S-function) should not start until that tap exists and the
replay closes, since 1:1 fidelity is the whole justification for the approach.
