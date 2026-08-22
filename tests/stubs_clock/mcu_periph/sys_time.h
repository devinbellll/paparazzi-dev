/* Host-harness stub: the clock ONLY.
 *
 * tests/stubs/ cannot be used for the FMFC checks: it also shadows
 * math/pprz_algebra_float.h, and oneloop_fmfc_law.h's callers need the REAL
 * pprz algebra. Putting this directory first on the include path replaces the
 * clock and nothing else.
 *
 * Returns double so the clock itself does not reintroduce single-precision
 * granularity into `time`, which the algebraic estimator squares and divides
 * by sample_time^2. Same contract as tests/stubs/mcu_periph/sys_time.h.
 */
#ifndef SYS_TIME_H
#define SYS_TIME_H
extern double stub_sys_time;
static inline double get_sys_time_float(void) { return stub_sys_time; }
#endif
