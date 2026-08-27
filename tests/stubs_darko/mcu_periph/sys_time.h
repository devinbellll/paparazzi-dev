/* Host-harness stub for the Darko FMFC checks.
 *
 * SEPARATE FROM tests/stubs/ ON PURPOSE. That directory also stubs
 * math/pprz_algebra_float.h down to a single float_vect_zero(), which is all
 * mfc_core.c reaches for -- but the Darko spine and the FINDI law need the
 * REAL quaternion and rotation algebra. Putting tests/stubs on the include
 * path would shadow it. This directory therefore stubs the clock and nothing
 * else, and the real math/ is linked in alongside.
 *
 * The harness drives this clock explicitly so each step lands on an exact
 * k*Ts. Returns double so the clock itself does not reintroduce
 * single-precision granularity into `time`, which the algebraic estimator
 * squares and divides by sample_time^2.
 */
#ifndef SYS_TIME_H
#define SYS_TIME_H
extern double stub_sys_time;
static inline double get_sys_time_float(void) { return stub_sys_time; }
#endif
