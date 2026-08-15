/* Host-harness stub: the harness drives this clock explicitly so each replay
 * step lands on an exact k*Ts. Returns double so the clock itself does not
 * reintroduce single-precision granularity into `time`, which the algebraic
 * estimator squares and divides by sample_time^2. */
#ifndef SYS_TIME_H
#define SYS_TIME_H
extern double stub_sys_time;
static inline double get_sys_time_float(void) { return stub_sys_time; }
#endif
