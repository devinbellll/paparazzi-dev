/* Host-harness stub: only float_vect_zero is reached from mfc_core.c. */
#ifndef PPRZ_ALGEBRA_FLOAT_H
#define PPRZ_ALGEBRA_FLOAT_H
static inline void float_vect_zero(float *a, const int n) {
  for (int i = 0; i < n; i++) { a[i] = 0.f; }
}
#endif
