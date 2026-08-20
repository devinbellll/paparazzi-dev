/*
 * Host harness: replay the sim repo's reference traces through the ACTUAL
 * firmware sources (mfc_core.c, mfc_core_mimo.c), compiled against minimal
 * stubs for sys_time and pprz_algebra_float.
 *
 * The traces in tests/golden/ are authoritative outputs of the sim repo. If a
 * check here fails, the port is wrong -- do not touch the traces.
 *
 * Verification ladder (see Knowledge/Plans/HEOL MIMO Horizontal Channel):
 *   0. scalar regression  -- mfc_core.c still reproduces 2nd_decoupled_alg.csv
 *                            after the recursion primitives were factored out.
 *   1. mimo_diag          -- diagonal gain, two identical channels.
 *   2. reduction check    -- with alpha = I, each mimo_diag channel must equal
 *                            the SISO trace. This is the step that actually
 *                            proves the port: that trace predates the MIMO code
 *                            entirely. Tolerance, not bit-exactness -- the
 *                            matrix solve takes a different if mathematically
 *                            equivalent path, and the cores run in float while
 *                            the traces were generated in double. See TOL.
 *   3. mimo_cross         -- off-diagonal gain, two different plants. This is
 *                            what pins the per-element numerator against the
 *                            shared scalar denominator; mimo_diag alone cannot
 *                            distinguish them.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "firmwares/rotorcraft/stabilization/mfc_core.h"
#include "firmwares/rotorcraft/stabilization/mfc_core_mimo.h"
#include "firmwares/rotorcraft/stabilization/heol_mimo.h"
#include "firmwares/rotorcraft/guidance/heol_input_sensitivity.h"

double stub_sys_time = 0.0;

/* ── Shared conditions that generated every trace ──────────────────────────── */
#define TS        0.01
#define NSAMP     601
#define WFILTER   10.0    /* estimator integration window */
#define FFILTER   10.0    /* reference-trajectory smoother */
#define HOLDTIME  0.1
#define KI        0.0
#define KP        16.0    /* p^2,  p = (1/0.05)/5 = 4 */
#define KD        8.0     /* 2*p */
#define STEP_T    0.2    /* unit step on every channel at t >= 0.2 */

/* Agreement tolerance against the reference traces.
 *
 * The firmware runs this estimator in SINGLE precision, and its numerator is a
 * second difference of t-weighted history divided by sample_time^2, so it loses
 * digits to cancellation. Measured worst-case disagreement with the traces at
 * float: 7.3e-4 (mimo_diag / reduction) and 1.2e-3 (mimo_cross). That is the
 * arithmetic, not the port -- a genuine porting error shows up as O(1), several
 * orders above this, so 5e-3 still catches one.
 *
 * Do not tighten this to double-precision levels without ALSO building the
 * cores in double; the traces were generated in double and cannot be matched
 * to 1e-5 by float arithmetic. */
#define TOL 5e-3

struct Plant { double g, d, b, tau, y, dy, u_act; };

static void plant_step(struct Plant *p, double u)
{
  p->u_act += (u - p->u_act) / p->tau * TS;
  double ddy = -p->g - p->d * p->dy + p->b * p->u_act;
  p->dy += ddy * TS;
  p->y  += p->dy * TS;
}

/* ── CSV ───────────────────────────────────────────────────────────────────── */
static int load_csv(const char *path, double *out, int ncol, int nrow)
{
  FILE *f = fopen(path, "r");
  if (!f) { fprintf(stderr, "cannot open %s\n", path); return 0; }
  char line[4096];
  if (!fgets(line, sizeof line, f)) { fclose(f); return 0; }  /* header */
  for (int r = 0; r < nrow; r++) {
    if (!fgets(line, sizeof line, f)) { fclose(f); return 0; }
    char *s = line;
    for (int c = 0; c < ncol; c++) {
      out[r * ncol + c] = strtod(s, &s);
      if (*s == ',') s++;
    }
  }
  fclose(f);
  return 1;
}

static int g_fail = 0;

static void report(const char *name, double worst, int wr, int wc, double tol)
{
  int ok = (worst <= tol);
  printf("  %-34s worst rel err %.3e  (tol %.0e)  %s",
         name, worst, tol, ok ? "PASS\n" : "FAIL");
  if (!ok) { printf("   <- row %d col %d\n", wr, wc); g_fail = 1; }
}

/* ── Check 0: the scalar core, after factoring ─────────────────────────────── */
static void check_scalar(void)
{
  static double gold[NSAMP * 6];
  if (!load_csv("tests/golden/2nd_decoupled_alg.csv", gold, 6, NSAMP)) { g_fail = 1; return; }

  struct MfcParameters m;
  stub_sys_time = 0.f;
  mfc_siso_init(&m, TS);
  m.decoupled = 1; m.use_trajec_sp = 1;
  m.time_trajec = FFILTER; m.int_window = WFILTER;
  m.kp = KP; m.kd = KD; m.ki = KI;
  m.est_hold_time = HOLDTIME; m.command_filter = 1.0;
  m.alpha = 1.0; m.setpoint = 0.0;
  m.u_min = -1e30; m.u_max = 1e30;
  mfc_siso_reset(&m);

  struct Plant p = { 9.81, 0.20, 1.0, 0.05, 0.f, 0.f, 0.f };
  double worst = 0; int wr = -1, wc = -1;

  for (int k = 0; k < NSAMP; k++) {
    double t = k * TS;
    if (k > 0) { plant_step(&p, m.command[0]); }
    stub_sys_time = t;
    m.setpoint = (t >= STEP_T) ? 1.0 : 0.0;
    mfc_siso_run(false, &m, p.y);

    double got[6] = { m.command[0], m.estimator, m.setpoint_trajec[0],
                      m.error[0], m.command_presat,
                      (m.estimator_den[0] != 0.0 && t > HOLDTIME) ? 1.0 : 0.0 };
    /* setpoint_trajec/error were shifted by run(); read the pre-shift values
     * back out of history slot 1, which now holds this step's value. */
    got[2] = m.setpoint_trajec[1];
    got[3] = m.error[1];
    for (int c = 0; c < 6; c++) {
      double ref = gold[k * 6 + c];
      double e = fabs(ref - got[c]) / (fabs(ref) > 1.0 ? fabs(ref) : 1.0);
      if (e > worst) { worst = e; wr = k; wc = c; }
    }
  }
  report("scalar core vs SISO trace", worst, wr, wc, TOL);
}

/* ── MIMO replay ───────────────────────────────────────────────────────────── */
static void run_mimo(const float alpha[2][2], const struct Plant plant0[2],
                     double *rec /* NSAMP x 11 */)
{
  struct MfcMimoParameters m;
  stub_sys_time = 0.f;
  mfc_mimo_init(&m, TS, 2);

  /* The core takes a capacity-sized alpha and reads the leading n x n block.
   * These traces are 2-vector, so widen once here and reuse it every tick. */
  float alpha_w[MFC_MIMO_N_MAX][MFC_MIMO_N_MAX] = {{0.f}};
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) { alpha_w[i][j] = alpha[i][j]; }
  }
  m.decoupled = 1; m.use_trajec_sp = 1;
  m.time_trajec = FFILTER; m.int_window = WFILTER;
  m.kp = KP; m.kd = KD; m.ki = KI;
  m.est_hold_time = HOLDTIME; m.command_filter = 1.0;
  for (int i = 0; i < 2; i++) { m.u_min[i] = -1e30; m.u_max[i] = 1e30; }
  mfc_mimo_reset(&m);

  struct Plant p[2] = { plant0[0], plant0[1] };

  for (int k = 0; k < NSAMP; k++) {
    double t = k * TS;
    if (k > 0) { for (int i = 0; i < 2; i++) { plant_step(&p[i], m.command[0][i]); } }
    stub_sys_time = t;

    float sp = (t >= STEP_T) ? 1.0 : 0.0;
    m.setpoint[0] = sp; m.setpoint[1] = sp;

    float y[2] = { p[0].y, p[1].y };
    /* One alpha per tick, reaching both the estimator and the command law. */
    mfc_mimo_set_alpha(&m, alpha_w);
    mfc_mimo_run(false, &m, y);

    double *r = rec + k * 11;
    r[0] = m.command[0][0];      r[1] = m.command[0][1];
    r[2] = m.estimator[0];       r[3] = m.estimator[1];
    r[4] = m.setpoint_trajec[1][0]; r[5] = m.setpoint_trajec[1][1];
    r[6] = m.error[1][0];        r[7] = m.error[1][1];
    r[8] = m.command_presat[0];  r[9] = m.command_presat[1];
    r[10] = (m.estimator_den[0] != 0.0 && t > HOLDTIME) ? 1.0 : 0.0;
  }
}

static void check_mimo(const char *name, const char *path,
                       const float alpha[2][2], const struct Plant p[2],
                       double *rec)
{
  static double gold[NSAMP * 11];
  if (!load_csv(path, gold, 11, NSAMP)) { g_fail = 1; return; }
  run_mimo(alpha, p, rec);

  double worst = 0; int wr = -1, wc = -1;
  for (int k = 0; k < NSAMP; k++) {
    for (int c = 0; c < 11; c++) {
      double ref = gold[k * 11 + c], got = rec[k * 11 + c];
      double e = fabs(ref - got) / (fabs(ref) > 1.0 ? fabs(ref) : 1.0);
      if (e > worst) { worst = e; wr = k; wc = c; }
    }
  }
  report(name, worst, wr, wc, TOL);
}

int main(void)
{
  printf("HEOL MIMO port -- golden trace verification\n\n");

  printf("[0] scalar regression (factoring did not change mfc_core.c)\n");
  check_scalar();

  static double diag[NSAMP * 11], cross[NSAMP * 11];

  printf("\n[1] mimo_diag: alpha = I, two identical channels\n");
  const float a_diag[2][2] = { { 1.0, 0.0 }, { 0.0, 1.0 } };
  const struct Plant p_diag[2] = { { 9.81, 0.20, 1.0, 0.05, 0, 0, 0 },
                                   { 9.81, 0.20, 1.0, 0.05, 0, 0, 0 } };
  check_mimo("mimo_diag", "tests/golden/mimo_diag.csv", a_diag, p_diag, diag);

  printf("\n[2] reduction check: each mimo_diag channel vs the SISO trace\n");
  {
    static double gold[NSAMP * 6];
    if (!load_csv("tests/golden/2nd_decoupled_alg.csv", gold, 6, NSAMP)) { g_fail = 1; }
    else {
      for (int ch = 0; ch < 2; ch++) {
        double worst = 0; int wr = -1;
        for (int k = 0; k < NSAMP; k++) {
          double ref = gold[k * 6 + 0];             /* u column */
          double got = diag[k * 11 + ch];           /* u1 / u2  */
          double e = fabs(ref - got) / (fabs(ref) > 1.0 ? fabs(ref) : 1.0);
          if (e > worst) { worst = e; wr = k; }
        }
        char nm[64]; snprintf(nm, sizeof nm, "mimo_diag u%d vs SISO u", ch + 1);
        report(nm, worst, wr, 0, TOL);
      }
    }
  }

  printf("\n[3] mimo_cross: off-diagonal alpha, two different plants\n");
  const float a_cross[2][2] = { { 1.00f, 0.35 }, { -0.20, 1.20f } };
  const struct Plant p_cross[2] = { { 9.81, 0.20, 1.0, 0.05, 0, 0, 0 },
                                    { 9.81, 0.35, 1.4, 0.08, 0, 0, 0 } };
  check_mimo("mimo_cross", "tests/golden/mimo_cross.csv", a_cross, p_cross, cross);

  /* ── Rung 4: the firmware loop -- sensitivity block + heol_mimo wrapper ──── */
  printf("\n[4] firmware loop: alpha_xy block + heol_mimo wrapper\n");
  {
    const double m = 0.8, hover = -9.81 * 0.8;   /* thrust_ref at hover, NED */
    struct HeolInputSensitivity a;

    /* 4a. alpha_xy at level attitude, psi = 0, must be anti-diagonal
     *     [[0, -9.81], [+9.81, 0]]: xdd responds to theta, ydd to phi. */
    heol_input_sensitivity_compute(&a, hover, 0.0, 0.0, 0.0, m);
    double e = fabs(a.alpha_xy[0][0]) + fabs(a.alpha_xy[1][1])
             + fabs(a.alpha_xy[0][1] + 9.81) + fabs(a.alpha_xy[1][0] - 9.81);
    report("alpha_xy level/psi=0 anti-diagonal", e, 0, 0, TOL);

    /* 4b. alpha_z is order-independent and positive: c_th*c_phi/m = 1.25. */
    report("alpha_z at level = 1/m", fabs(a.alpha_z - 1.25), 0, 0, 1e-6);

    /* 4c. |det| at the trajectory's WORST-CONDITIONED sample -- row 541 of the
     *     generated table, the actual argmin over all 751 rows. Using an
     *     invented (T, phi, theta, psi) combination here gives a point the
     *     aircraft never flies and a det outside the plan's range. */
    heol_input_sensitivity_compute(&a, -6.4086, -0.107880, 0.622290, 0.636419, m);
    double det = fabs(a.alpha_xy[0][0] * a.alpha_xy[1][1]
                    - a.alpha_xy[0][1] * a.alpha_xy[1][0]);
    printf("  %-34s |det| = %.2f  (plan min 51.5)  %s\n",
           "alpha_xy worst-case conditioning", det,
           (det > 51.4 && det < 51.7) ? "PASS" : "FAIL");
    if (!(det > 51.4 && det < 51.7)) { g_fail = 1; }

    /* 4d. heol_mimo: command must be u_ff + u_fb, and the TOTAL must be what
     *     the bank clamp bites on -- not the increment. */
    struct HeolMimoParameters h;
    stub_sys_time = 0.0;
    heol_mimo_init(&h, 1.0 / 500.0);
    h.mfc.kp = 4.0; h.mfc.kd = 2.8; h.mfc.ki = 0.0;
    h.mfc.int_window = 500.0; h.mfc.command_filter = 1.0;
    h.mfc.est_hold_time = 0.1;
    const double bank = 0.349;                    /* 20 deg */
    for (int i = 0; i < 2; i++) { h.u_max[i] = bank; h.u_min[i] = -bank; }
    heol_mimo_reset(&h);
    heol_input_sensitivity_compute(&a, hover, 0.0, 0.0, 0.0, m);
    heol_mimo_set_alpha(&h, a.alpha_xy);

    double worst_sum = 0.0; int clamped = 0;
    for (int k = 1; k < 2000; k++) {
      stub_sys_time = k / 500.0;
      /* a big steady position error, to drive the channel into the bank limit */
      float meas[2] = { 5.0, -5.0 }, ref[2] = { 0.0, 0.0 };
      float uff[2]  = { 0.05, -0.02 };     /* nominal attitude [rad] */
      heol_mimo_run(false, &h, meas, ref, uff);
      for (int i = 0; i < 2; i++) {
        double d = fabs(h.command[i] - (uff[i] + h.mfc.command[0][i]));
        if (d > worst_sum) { worst_sum = d; }
        if (fabs(h.command[i]) > bank + 1e-6) { clamped = 1; }
      }
    }
    report("command == u_ff + u_fb", worst_sum, 0, 0, 1e-9);
    printf("  %-34s %s\n", "total attitude within max bank",
           clamped ? "FAIL" : "PASS");
    if (clamped) { g_fail = 1; }
  }

  printf("\n%s\n", g_fail ? "RESULT: FAIL" : "RESULT: all checks passed");
  return g_fail;
}
