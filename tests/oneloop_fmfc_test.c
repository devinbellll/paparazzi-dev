/*
 * Host harness: analytic property checks on the flatness-MFC brackets
 * (oneloop_fmfc_law.h) and on their composition into the two increments
 * oneloop_fmfc.c replaces. Same fixture convention as
 * tests/oneloop_findi_darko_test.c.
 *
 * WHY PROPERTIES AND NOT A GOLDEN TRACE
 * -------------------------------------
 * There is no MATLAB in this environment and there will not be one, so there
 * are no golden traces for this controller and none are coming. The checks
 * below were decided UP FRONT, before the port, rather than discovered
 * mid-way: each is a case where the correct answer is known in closed form,
 * chosen so that the failure modes THIS transcription can actually have are
 * the ones that get caught.
 *
 * And offline agreement is NOT evidence the loop flies. The HEOL MIMO port was
 * bit-exact against its traces and diverged the moment the loop engaged. These
 * checks buy one thing: that a divergence in SITL is a control problem and not
 * a transcription problem.
 *
 * WHAT EACH GROUP DISCRIMINATES
 * -----------------------------
 *  A. bracket algebra   That u = u* + alpha^-1(-F_hat - f_b) is what actually
 *                       comes out: f_f grounded at zero, the caller's f_b
 *                       arriving with no gain of the wrapper's own, the
 *                       INVERSE (not alpha) multiplying, the nominal added
 *                       strictly outside, and the estimator's delayed-command
 *                       lane carrying du alone. That last one is the HEOL
 *                       invariant; breaking it turns HEOL back into an
 *                       ordinary setpoint tracker, silently.
 *  B. core config       That the bracket really is width 3 and decoupled, that
 *                       the estimator is inert before its hold time, that the
 *                       denominator is scalar and SHARED (the one structural
 *                       asymmetry of the MIMO form), and that the MIMO reset
 *                       path does NOT have the setpoint_trajec hole that
 *                       mfc_siso_reset() had until 2026-08-20.
 *  C. 2-vector callers  That the flying horizontal channel is untouched. A
 *                       2-vector heol_mimo sequence must come out BIT-IDENTICAL
 *                       whether or not a 3-vector bracket is running
 *                       interleaved against the same core.
 *  D. conventions       The two things this port can get backwards and that no
 *                       compiler will catch: the linear loop's errors are
 *                       MEASUREMENT MINUS REFERENCE (opposite to FINDI's), and
 *                       there is NO omega_dot_ref term inside omega_dot_c. Both
 *                       are checked by their observable consequence, not by
 *                       reading the code back.
 *  E. alpha             That alpha_fi = I/m and alpha_m = J^-1 are constant,
 *                       diagonal, and INVERTED the right way round. A J vs
 *                       J^-1 swap is a factor of ~2e4 on this airframe and
 *                       would look like a wild gain, not like a bug.
 *  F. carried-forward   float_mat_inv_4d's 0-is-success convention and its
 *                       undivided-adjugate failure path, asserted rather than
 *                       remembered. Cost a full session once.
 *
 * NOT CHECKED HERE, deliberately: the allocation pseudo-inverse and the
 * flatness transform. Both are carried over verbatim from the FLOWN FINDI
 * spine (oneloop_findi.c, ANTON_FINDI) and are covered by its own harness.
 * This file is scoped to the delta.
 */

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "firmwares/rotorcraft/oneloop/oneloop_fmfc_law.h"
#include "firmwares/rotorcraft/stabilization/heol_mimo.h"
#include "math/pprz_algebra_float.h"

/* The harness owns the clock (tests/stubs_clock/mcu_periph/sys_time.h) so each
 * step lands on an exact k*Ts and `time` is not a source of noise. */
double stub_sys_time = 0.0;

static int g_fail = 0;
static int g_checks = 0;

static void ck(const char *name, int ok, const char *detail)
{
  g_checks++;
  printf("  %-56s %s", name, ok ? "PASS\n" : "FAIL");
  if (!ok) { printf("   <- %s\n", detail); g_fail = 1; }
}

static void ck_near(const char *name, double got, double want, double tol)
{
  char d[192];
  int ok = fabs(got - want) <= tol;
  snprintf(d, sizeof d, "got %.9g, want %.9g (tol %g)", got, want, tol);
  ck(name, ok, d);
}

static void ck_exact(const char *name, double got, double want)
{
  char d[192];
  int ok = (got == want);
  snprintf(d, sizeof d, "got %.17g, want %.17g", got, want);
  ck(name, ok, d);
}

/* ── Airframe constants: the real ANTON values from anton_fmfc.xml ─────────── */
#define TS      (1.f / 500.f)
#define MASS    0.8f
#define IXX     0.0068f
#define IYY     0.0068f
#define IZZ     0.0136f
#define GRAV    9.81f

/* Estimator parameters, from flat_mfc_quad_params.m via the airframe. */
#define WIN_FI    500.f
#define HOLD_FI   0.5f
#define WIN_M     20.f
#define HOLD_M    0.5f

/* Gains: the UNTUNED placeholders. Their VALUES are irrelevant to every check
 * below -- each property holds for any gain set -- but using the shipped ones
 * means a check that fires is about structure at the operating point that
 * actually flies. */
#define KX      1.8898f
#define ZETA_X  1.1093f
#define KA      0.2062f
#define K_ATT   7.0745f
#define K_RATE  18.6741f

/** A linear/force bracket configured exactly as oneloop_fmfc.c's init_brackets()
 * configures fmfc_fi. Kept in one place so the checks exercise the shipped
 * configuration and not a harness-local approximation of it. */
static void make_fi(struct FmfcBracket *b)
{
  /* The clock is global and the estimator latches start_time in its reset, so
   * a bracket built while the clock is still at the previous group's value
   * would run at NEGATIVE elapsed time and its estimator would never go
   * valid. Zero it here so every group is independent of the ones before. */
  stub_sys_time = 0.0;
  fmfc_bracket_init(b, TS);
  b->mfc.int_window    = WIN_FI;
  b->mfc.est_hold_time = HOLD_FI;
  b->mfc.command_filter = 1.f;
  b->mfc.est_use_presat_command = false;
  b->use_applied_u_prev = true;
  fmfc_bracket_set_alpha_diag(b, 1.f / MASS, 1.f / MASS, 1.f / MASS);
  const float rail = 4.f * MASS * GRAV;
  for (int i = 0; i < FMFC_BRACKET_N; i++) { b->du_min[i] = -rail; b->du_max[i] = rail; }
  fmfc_bracket_reset(b);
}

/** The angular/moment bracket, likewise. */
static void make_m(struct FmfcBracket *b)
{
  const float I[3] = { IXX, IYY, IZZ };
  stub_sys_time = 0.0;              /* see make_fi() */
  fmfc_bracket_init(b, TS);
  b->mfc.int_window    = WIN_M;
  b->mfc.est_hold_time = HOLD_M;
  b->mfc.command_filter = 1.f;
  b->mfc.est_use_presat_command = false;
  b->use_applied_u_prev = true;
  fmfc_bracket_set_alpha_diag(b, 1.f / IXX, 1.f / IYY, 1.f / IZZ);
  for (int i = 0; i < FMFC_BRACKET_N; i++) {
    b->du_min[i] = -200.f * I[i];
    b->du_max[i] =  200.f * I[i];
  }
  fmfc_bracket_reset(b);
}

static const float Z3[3] = { 0.f, 0.f, 0.f };

/* ══════════════════════════════════════════════════════════════════════════
 *  A. Bracket algebra
 * ══════════════════════════════════════════════════════════════════════════ */

static void group_A(void)
{
  printf("\nA. Bracket algebra -- u = u* + alpha^-1 (-F_hat - f_b)\n");

  /* A1-A4 run inside the estimator's hold time, where F_hat is identically 0
   * by construction. That is what makes the command law checkable in closed
   * form: the estimator contributes nothing and every remaining term is the
   * caller's own. */
  struct FmfcBracket b;
  make_fi(&b);
  stub_sys_time = 0.0;

  const float eps[3]    = { 0.3f, -0.2f, 0.05f };
  const float u_star[3] = { 1.f, -2.f, -MASS * GRAV };
  const float f_b[3]    = { 0.7f, 1.3f, -0.4f };

  stub_sys_time = TS;
  fmfc_bracket_run(false, &b, eps, u_star, f_b, NULL);

  /* A1: f_f is grounded. setpoint is held at 0 and use_trajec_sp is false, so
   * the core's ddot_sp is exactly 0 -- the whole nominal input must arrive
   * through u*, never through the core's reference smoother. If ddot_sp were
   * live, du would carry a second-difference-of-setpoint term. */
  ck_exact("A1 F_hat inert before hold time (channel 0)", b.mfc.estimator[0], 0.0);
  ck_exact("A1 F_hat inert before hold time (channel 2)", b.mfc.estimator[2], 0.0);

  /* A2/A3: du == alpha^-1 (-f_b) == -m f_b. The INVERSE multiplies, so the
   * correction is m TIMES f_b, not f_b over m. Off by m^2 = 0.64 either way on
   * this airframe -- a plausible-looking wrong answer, which is why it is
   * checked as an exact identity and not eyeballed. */
  for (int i = 0; i < 3; i++) {
    char n[80];
    snprintf(n, sizeof n, "A2 du[%d] == alpha^-1(-f_b) == -m*f_b", i);
    ck_near(n, b.du[i], -(double)MASS * f_b[i], 1e-6);
  }

  /* A4: the nominal is added strictly OUTSIDE the correction. */
  for (int i = 0; i < 3; i++) {
    char n[80];
    snprintf(n, sizeof n, "A4 command[%d] == u*[%d] + du[%d]", i, i, i);
    /* Widen only AFTER the float addition: the module adds these as floats and
     * a double-precision right-hand side would fail on rounding alone. */
    ck_exact(n, b.command[i], (float)(u_star[i] + b.du[i]));
  }

  /* A5: THE HEOL INVARIANT. The estimator's delayed-command lane must carry
   * the correction alone. With use_applied_u_prev off, that is the core's own
   * command -- and it must be du, never u* + du. This is the check that
   * catches HEOL silently degenerating into an ordinary setpoint tracker. */
  struct FmfcBracket c;
  make_fi(&c);
  c.use_applied_u_prev = false;
  stub_sys_time = TS;
  fmfc_bracket_run(false, &c, eps, u_star, f_b, NULL);
  for (int i = 0; i < 3; i++) {
    char n[80];
    snprintf(n, sizeof n, "A5 command_est[%d] is du alone, not u*+du", i);
    ck_exact(n, c.mfc.command_est[i], c.du[i]);
  }

  /* A6: use_applied_u_prev overrides that lane with the MEASURED applied
   * correction, and writes it BEFORE the run, so it is what THIS tick's
   * estimator consumes -- not the previous tick's. The discriminating setup:
   * three brackets driven identically, with u_prev applied only on the FINAL
   * tick, at 0, U and 2U. If the write landed a tick late (or not at all) the
   * three would agree. If it lands this tick they must differ, and -- since
   * the estimator's alpha@d2u term is LINEAR in the previous command -- the
   * two differences must be in exact 1:2 ratio.
   *
   * Note command_est cannot be inspected after the call: mfc_mimo_run()'s
   * history shift rewrites it with its own command on the way out. F_hat is
   * the only observable, which is why the check is built this way. */
  struct FmfcBracket p, q, r2;
  make_fi(&p); make_fi(&q); make_fi(&r2);
  const float up_0[3] = { 0.f, 0.f, 0.f };
  const float up_1[3] = { 5.f, 0.f, 0.f };
  const float up_2[3] = { 10.f, 0.f, 0.f };
  const int LAST = 400;                     /* 400 * 2 ms = 0.8 s > HOLD_FI */
  for (int k = 1; k <= LAST; k++) {
    stub_sys_time = k * (double)TS;
    const int last = (k == LAST);
    fmfc_bracket_run(true, &p, eps, u_star, f_b, up_0);
    fmfc_bracket_run(true, &q, eps, u_star, f_b, last ? up_1 : up_0);
    fmfc_bracket_run(true, &r2, eps, u_star, f_b, last ? up_2 : up_0);
  }
  const double d1 = q.mfc.estimator[0] - p.mfc.estimator[0];
  const double d2 = r2.mfc.estimator[0] - p.mfc.estimator[0];
  ck("A6 u_prev_applied reaches THIS tick's estimator", d1 != 0.0,
     "F_hat unchanged by the final tick's applied command: written too late, "
     "or not written at all");
  ck_near("A6 the estimator lane is linear in u_prev (1:2)", d2, 2.0 * d1,
          fabs(d1) * 1e-4);
  ck_exact("A6 channels 1,2 untouched by a channel-0 u_prev",
           q.mfc.estimator[1], p.mfc.estimator[1]);

  /* A7: the rails clamp the CORRECTION, and the total is u* + clamp(du). */
  struct FmfcBracket r;
  make_fi(&r);
  const float rail = 4.f * MASS * GRAV;
  const float big_f_b[3] = { 1000.f, 0.f, 0.f };   /* -m*1000 N, way past rail */
  stub_sys_time = TS;
  fmfc_bracket_run(false, &r, Z3, u_star, big_f_b, NULL);
  ck_near("A7 du clamped to the correction rail", r.du[0], -rail, 1e-4);
  ck_exact("A7 command == u* + clamped du", r.command[0], (double)u_star[0] + r.du[0]);
  ck_near("A7 presat value preserved for the tap", r.mfc.command_presat[0],
          -(double)MASS * 1000.f, 1e-2);

  /* A8: est_use_presat_command is honoured, and the two taps differ when the
   * rail bites. Set explicitly on every bracket precisely because a define
   * shadowing the core default is what hid the HEOL divergence defect. */
  struct FmfcBracket s;
  make_fi(&s);
  s.use_applied_u_prev = false;
  s.mfc.est_use_presat_command = true;
  stub_sys_time = TS;
  fmfc_bracket_run(false, &s, Z3, u_star, big_f_b, NULL);
  ck("A8 presat tap differs from postsat tap at the rail",
     s.mfc.command_est[0] != r.mfc.command[0][0],
     "the two taps agree: est_use_presat_command is not being read");
  ck_exact("A8 presat tap == command_presat", s.mfc.command_est[0], s.mfc.command_presat[0]);
}

/* ══════════════════════════════════════════════════════════════════════════
 *  B. Core configuration
 * ══════════════════════════════════════════════════════════════════════════ */

static void group_B(void)
{
  printf("\nB. Core configuration -- width, decoupling, hold, shared denominator, reset\n");

  struct FmfcBracket b;
  make_fi(&b);
  ck_exact("B1 channel width is 3", b.mfc.n, 3.0);
  ck_exact("B2 decoupled estimator (z = eps, nothing folded)", b.mfc.decoupled, 1.0);
  ck_exact("B2 use_trajec_sp off (ddot_sp identically 0)", b.mfc.use_trajec_sp, 0.0);
  ck_exact("B2 kp = 0 (feedback is the caller's)", b.mfc.kp, 0.0);
  ck_exact("B2 kd = 1 (structural pass-through)", b.mfc.kd, 1.0);
  ck_exact("B2 ki = 0 (no integral in either bracket)", b.mfc.ki, 0.0);
  ck_exact("B2 deriv_filter = 0 (pass-through)", b.mfc.deriv_filter, 0.0);
  ck_exact("B2 use_external_derivative on", b.mfc.use_external_derivative, 1.0);

  /* B3: the third channel is really live. If n had stayed 2 -- the width the
   * shared core was written for -- a z-only demand would come out as nothing
   * at all, and the aircraft would simply have no vertical correction. */
  struct FmfcBracket z;
  make_fi(&z);
  const float fb_z[3] = { 0.f, 0.f, 1.f };
  stub_sys_time = TS;
  fmfc_bracket_run(false, &z, Z3, Z3, fb_z, NULL);
  ck_exact("B3 channel 2 is live (z-only f_b gives z-only du)", z.du[2] != 0.f, 1.0);
  ck_exact("B3 no cross-talk into channel 0 (alpha is diagonal)", z.du[0], 0.0);
  ck_exact("B3 no cross-talk into channel 1", z.du[1], 0.0);

  /* B4: the estimator is inert until est_hold_time and live after. The hold is
   * the parameter that was wrong on the HEOL horizontal channel (0.1 against a
   * reference 0.8), so it is asserted rather than assumed. */
  struct FmfcBracket h;
  make_fi(&h);
  const float drive[3] = { 0.5f, 0.5f, 0.5f };
  const float fb0[3]   = { 1.f, 1.f, 1.f };
  int k = 1;
  for (; k * (double)TS < (double)HOLD_FI; k++) {
    stub_sys_time = k * (double)TS;
    fmfc_bracket_run(true, &h, drive, Z3, fb0, NULL);
  }
  ck_exact("B4 F_hat == 0 up to est_hold_time", h.mfc.estimator[0], 0.0);
  for (int j = 0; j < 200; j++, k++) {
    stub_sys_time = k * (double)TS;
    fmfc_bracket_run(true, &h, drive, Z3, fb0, NULL);
  }
  ck("B4 F_hat becomes live after est_hold_time", h.mfc.estimator[0] != 0.f,
     "estimator still zero well past the hold time");

  /* B5: PER-ELEMENT NUMERATOR, SCALAR SHARED DENOMINATOR -- the one structural
   * asymmetry of the MIMO form, and the thing most easily got wrong in a port.
   * Three channels driven differently must divide by the SAME number. */
  struct FmfcBracket d;
  make_fi(&d);
  const float drive3[3] = { 0.2f, -0.7f, 1.4f };
  for (int j = 1; j <= 400; j++) {
    stub_sys_time = j * (double)TS;
    fmfc_bracket_run(true, &d, drive3, Z3, fb0, NULL);
  }
  const double den = d.mfc.estimator_den[0];
  ck("B5 shared denominator is non-zero", den != 0.0, "denominator collapsed");
  for (int i = 0; i < 3; i++) {
    char n[80];
    snprintf(n, sizeof n, "B5 F_hat[%d] == num[%d] / the SHARED den", i, i);
    const double want = d.mfc.estimator_num[0][i] / den;
    /* Relative tolerance: the core divides in float and this recomputes in
     * double, so one float rounding separates them by construction. The claim
     * under test is WHICH denominator is used, not the last mantissa bit --
     * and a per-channel denominator would be wrong by percent, not by 1e-7. */
    ck_near(n, d.mfc.estimator[i], want, fabs(want) * 1e-6);
  }

  /* B6: the mfc_siso_reset() hole does NOT exist in the MIMO reset path.
   * mfc_siso_reset() never cleared setpoint_trajec[], so on re-entry the
   * reference smoother carried the previous engagement's setpoint into the
   * first ddot_sp (fixed 2026-08-20). Checked directly here rather than
   * inferred from the source. */
  struct FmfcBracket rr;
  make_fi(&rr);
  for (int hh = 0; hh < 3; hh++) {
    for (int i = 0; i < MFC_MIMO_N_MAX; i++) {
      rr.mfc.setpoint_trajec[hh][i] = 42.f;
      rr.mfc.error[hh][i] = 42.f;
      rr.mfc.z[hh][i] = 42.f;
      rr.mfc.estimator_num[hh][i] = 42.f;
    }
    rr.mfc.estimator_den[hh] = 42.f;
  }
  fmfc_bracket_reset(&rr);
  int clean = 1;
  for (int hh = 0; hh < 3; hh++) {
    for (int i = 0; i < MFC_MIMO_N_MAX; i++) {
      if (rr.mfc.setpoint_trajec[hh][i] != 0.f) { clean = 0; }
      if (rr.mfc.error[hh][i] != 0.f || rr.mfc.z[hh][i] != 0.f) { clean = 0; }
      if (rr.mfc.estimator_num[hh][i] != 0.f) { clean = 0; }
    }
    if (rr.mfc.estimator_den[hh] != 0.f) { clean = 0; }
  }
  ck("B6 reset clears setpoint_trajec[] (the mfc_siso_reset hole)", clean,
     "a history entry survived the reset");

  /* B7: and the observable consequence -- the first du after a reset carries
   * no memory of the previous engagement. */
  struct FmfcBracket e1, e2;
  make_fi(&e1);
  make_fi(&e2);
  for (int j = 1; j <= 400; j++) {
    stub_sys_time = j * (double)TS;
    fmfc_bracket_run(true, &e1, drive3, Z3, fb0, NULL);
  }
  stub_sys_time = 401 * (double)TS;
  fmfc_bracket_reset(&e1);
  fmfc_bracket_reset(&e2);
  stub_sys_time = 402 * (double)TS;
  fmfc_bracket_run(true, &e1, drive, Z3, fb0, NULL);
  fmfc_bracket_run(true, &e2, drive, Z3, fb0, NULL);
  for (int i = 0; i < 3; i++) {
    char n[80];
    snprintf(n, sizeof n, "B7 first du[%d] after reset has no stale history", i);
    ck_exact(n, e1.du[i], e2.du[i]);
  }
}

/* ══════════════════════════════════════════════════════════════════════════
 *  C. The 2-vector callers must remain bit-identical
 * ══════════════════════════════════════════════════════════════════════════ */

/** Drive a heol_mimo channel through a fixed, deterministic sequence, writing
 * the whole trajectory of its command into out[]. Alpha is the horizontal
 * channel's ANTI-DIAGONAL accel-to-tilt shape at level -- the case that breaks
 * float_mat_invert() and is why the core uses a closed-form cofactor. */
#define C_STEPS 300
static void run_heol_2v(float out[C_STEPS][2], struct FmfcBracket *interleave)
{
  struct HeolMimoParameters h;
  stub_sys_time = 0.0;              /* see make_fi() */
  heol_mimo_init(&h, TS);
  h.mfc.int_window    = 500.f;
  h.mfc.est_hold_time = 0.8f;
  h.mfc.kp = 1.5f;
  h.mfc.kd = 2.5f;
  h.mfc.ki = 0.f;
  heol_mimo_reset(&h);

  const float alpha[2][2] = { { 0.f, -GRAV }, { GRAV, 0.f } };
  struct FmfcBracket dummy;
  const float fb0[3] = { 1.f, -1.f, 0.5f };
  const float drv[3] = { 0.1f, 0.2f, 0.3f };
  if (interleave == NULL) { (void)dummy; }

  for (int k = 1; k <= C_STEPS; k++) {
    stub_sys_time = k * (double)TS;
    const float meas[2] = { 0.01f * k, -0.02f * k };
    const float ref[2]  = { 0.005f * k, 0.f };
    const float uff[2]  = { 0.02f, -0.03f };
    heol_mimo_set_alpha(&h, alpha);
    heol_mimo_run(true, &h, meas, ref, uff);
    out[k - 1][0] = h.command[0];
    out[k - 1][1] = h.command[1];

    /* The 3-vector bracket runs against the SAME core code, between the
     * 2-vector steps and on the same clock. If mfc_core_mimo carried any
     * shared or static state -- or if the width generalisation leaked -- this
     * is where the horizontal channel's trajectory would move. */
    if (interleave != NULL) {
      fmfc_bracket_set_alpha_diag(interleave, 1.f / MASS, 1.f / MASS, 1.f / MASS);
      fmfc_bracket_run(true, interleave, drv, Z3, fb0, NULL);
    }
  }
}

static void group_C(void)
{
  printf("\nC. The flying 2-vector caller is untouched\n");

  static float alone[C_STEPS][2];
  static float mixed[C_STEPS][2];

  run_heol_2v(alone, NULL);

  struct FmfcBracket b;
  make_fi(&b);
  run_heol_2v(mixed, &b);

  int identical = 1;
  int first_bad = -1;
  for (int k = 0; k < C_STEPS; k++) {
    for (int i = 0; i < 2; i++) {
      if (alone[k][i] != mixed[k][i]) { identical = 0; if (first_bad < 0) { first_bad = k; } }
    }
  }
  char d[160];
  snprintf(d, sizeof d, "first divergence at step %d", first_bad);
  ck("C1 heol_mimo trajectory BIT-IDENTICAL with a 3-vector bracket live", identical, d);

  /* C2: and the 2-vector channel really did do something, so C1 is not
   * comparing two runs of zeros. */
  ck("C2 the 2-vector reference trajectory is non-trivial",
     alone[C_STEPS - 1][0] != 0.f && alone[C_STEPS - 1][0] != alone[0][0],
     "the 2-vector channel produced a constant: C1 proves nothing");

  /* C3: the anti-diagonal alpha inverted correctly. float_mat_invert()'s
   * Gauss-Jordan divides by a[0][0], which is exactly 0 here; the core's
   * closed-form cofactor has no such failure mode. */
  struct HeolMimoParameters h;
  heol_mimo_init(&h, TS);
  const float alpha[2][2] = { { 0.f, -GRAV }, { GRAV, 0.f } };
  heol_mimo_set_alpha(&h, alpha);
  ck("C3 anti-diagonal alpha accepted", h.mfc.alpha_ok, "set_alpha rejected it");
  ck_near("C3 alpha_inv[0][1] == 1/g", h.mfc.alpha_inv[0][1], 1.0 / GRAV, 1e-6);
  ck_near("C3 alpha_inv[1][0] == -1/g", h.mfc.alpha_inv[1][0], -1.0 / GRAV, 1e-6);
}

/* ══════════════════════════════════════════════════════════════════════════
 *  D. The two conventions that must not be harmonised
 * ══════════════════════════════════════════════════════════════════════════ */

/** The linear bracket exactly as oneloop_fmfc.c composes it. Reproduced here
 * (rather than calling into the module, which needs state.h) so the CHECK is
 * on the composition and not only on the wrapper. */
static void fmfc_linear_step(struct FmfcBracket *b,
                             const float p[3], const float p_ref[3],
                             const float v[3], const float v_ref[3],
                             const float a[3], const float a_ref[3],
                             float out_fi_c[3])
{
  const float kp = KX * KX;
  const float kv = 2.f * ZETA_X * KX;
  float e_p[3], a_c[3];
  for (int i = 0; i < 3; i++) {
    e_p[i] = p[i] - p_ref[i];                       /* MEASUREMENT MINUS REF */
    a_c[i] = kp * e_p[i]
           + kv * (v[i] - v_ref[i])
           + KA * (a[i] - a_ref[i]);
  }
  const float u_star[3] = { MASS * a_ref[0], MASS * a_ref[1],
                            MASS * (a_ref[2] - GRAV) };
  fmfc_bracket_set_alpha_diag(b, 1.f / MASS, 1.f / MASS, 1.f / MASS);
  fmfc_bracket_run(true, b, e_p, u_star, a_c, NULL);
  for (int i = 0; i < 3; i++) { out_fi_c[i] = b->command[i]; }
}

/** The angular bracket exactly as oneloop_fmfc.c composes it. */
static void fmfc_angular_step(struct FmfcBracket *b,
                              const float zeta_e[3], const float w[3],
                              const float w_ref[3], const float dw_ref[3],
                              float out_m_c[3])
{
  const float I[3] = { IXX, IYY, IZZ };
  float f_b[3], u_star[3], eps[3];
  for (int i = 0; i < 3; i++) {
    const float w_c  = K_ATT * zeta_e[i] + w_ref[i];
    const float dw_c = K_RATE * (w_c - w[i]);   /* NO dw_ref TERM HERE */
    f_b[i]    = -dw_c;
    u_star[i] = I[i] * dw_ref[i];               /* the feedforward lives here */
    eps[i]    = -zeta_e[i];                     /* z = -H(z) zeta_e; H = 1 here */
  }
  fmfc_bracket_set_alpha_diag(b, 1.f / IXX, 1.f / IYY, 1.f / IZZ);
  fmfc_bracket_run(true, b, eps, u_star, f_b, NULL);
  for (int i = 0; i < 3; i++) { out_m_c[i] = b->command[i]; }
}

static void group_D(void)
{
  printf("\nD. Conventions -- error sense, and the missing dw_ref\n");

  /* D1: HOVER IDENTITY. Everything at reference, no nominal accel, estimator
   * inert: the commanded inertial force must be exactly [0, 0, -m g]. The sign
   * is the allocator's -- NED, so negative z is up -- and getting it backwards
   * is a full-thrust inversion, not a tuning problem. */
  struct FmfcBracket b;
  make_fi(&b);
  stub_sys_time = TS;
  float fi_c[3];
  fmfc_linear_step(&b, Z3, Z3, Z3, Z3, Z3, Z3, fi_c);
  ck_exact("D1 hover: fi_c.x == 0", fi_c[0], 0.0);
  ck_exact("D1 hover: fi_c.y == 0", fi_c[1], 0.0);
  ck_near("D1 hover: fi_c.z == -m*g (negative is UP in NED)", fi_c[2],
          -(double)MASS * GRAV, 1e-5);

  /* D2: THE ERROR SENSE. The linear loop forms MEASUREMENT MINUS REFERENCE --
   * the MFC convention, and the OPPOSITE of the FINDI cascade in the file next
   * door. Checked by its only observable consequence: an aircraft NORTH of its
   * reference must be pushed SOUTH. Flip the sense and this check reports a
   * positive x force, which is the divergence, before it ever reaches SITL. */
  struct FmfcBracket n;
  make_fi(&n);
  stub_sys_time = TS;
  const float p_north[3] = { 1.f, 0.f, 0.f };   /* 1 m north of the reference */
  fmfc_linear_step(&n, p_north, Z3, Z3, Z3, Z3, Z3, fi_c);
  ck("D2 north position error commands a SOUTHWARD force", fi_c[0] < 0.f,
     "fi_c.x >= 0 for a northward error: the error sense is inverted");
  ck_near("D2 magnitude is m * kx^2 * e_p", fi_c[0],
          -(double)MASS * KX * KX * 1.0, 1e-4);

  /* D3: and the vertical channel with the same sense. Below the reference
   * altitude means z (down-positive) is LARGER than z_ref, so the correction
   * must push further up, i.e. more negative than -m g. */
  struct FmfcBracket lo;
  make_fi(&lo);
  stub_sys_time = TS;
  const float p_low[3] = { 0.f, 0.f, 1.f };    /* 1 m below the reference */
  fmfc_linear_step(&lo, p_low, Z3, Z3, Z3, Z3, Z3, fi_c);
  ck("D3 below-reference commands MORE upward force", fi_c[2] < -(float)MASS * GRAV,
     "the vertical error sense is inverted");

  /* D4: NO dw_ref INSIDE dw_c. The feedforward enters through m* = I dw_ref
   * and NOWHERE else, unlike FINDI's dw_c = k_rate(w_c - w) + dw_ref. The
   * discriminating test is the SENSITIVITY: doubling dw_ref must move m_c by
   * exactly I*dw_ref. If dw_ref were also inside dw_c the same doubling would
   * move it by I*dw_ref*(1 + k_rate*0)... no -- it would appear twice, so the
   * step would be (1 + 1) * I * dw_ref. A factor of two, and nothing else in
   * the loop produces one. */
  struct FmfcBracket a1, a2;
  make_m(&a1);
  make_m(&a2);
  const float dwr1[3] = { 1.f, 2.f, 3.f };
  const float dwr2[3] = { 2.f, 4.f, 6.f };
  float mc1[3], mc2[3];
  stub_sys_time = TS;
  fmfc_angular_step(&a1, Z3, Z3, Z3, dwr1, mc1);
  stub_sys_time = TS;
  fmfc_angular_step(&a2, Z3, Z3, Z3, dwr2, mc2);
  const float I[3] = { IXX, IYY, IZZ };
  for (int i = 0; i < 3; i++) {
    char nm[96];
    snprintf(nm, sizeof nm, "D4 d(m_c[%d])/d(dw_ref) == I[%d], exactly once", i, i);
    ck_near(nm, mc2[i] - mc1[i], (double)I[i] * dwr1[i], 1e-9);
  }

  /* D5: angular hover identity. No attitude error, no rates, no nominal,
   * estimator inert -> zero commanded moment, exactly. */
  struct FmfcBracket z;
  make_m(&z);
  stub_sys_time = TS;
  float mc0[3];
  fmfc_angular_step(&z, Z3, Z3, Z3, Z3, mc0);
  for (int i = 0; i < 3; i++) {
    char nm[80];
    snprintf(nm, sizeof nm, "D5 trimmed angular bracket commands m_c[%d] == 0", i);
    ck_exact(nm, mc0[i], 0.0);
  }

  /* D6: the angular cascade's own sense. A positive attitude error zeta_e is
   * COMMAND MINUS MEASUREMENT, so it must command a POSITIVE moment on that
   * axis. m_c = I * k_rate * k_att * zeta_e in closed form here. */
  struct FmfcBracket r;
  make_m(&r);
  stub_sys_time = TS;
  const float ze[3] = { 0.1f, 0.f, 0.f };
  float mcr[3];
  fmfc_angular_step(&r, ze, Z3, Z3, Z3, mcr);
  ck_near("D6 m_c == I * k_rate * k_att * zeta_e", mcr[0],
          (double)IXX * K_RATE * K_ATT * 0.1, 1e-7);
  ck("D6 positive attitude error gives a positive moment", mcr[0] > 0.f,
     "the attitude cascade sign is inverted");

  /* D7: the estimator drive of the angular bracket carries the NEGATION.
   * z = -H(z) zeta_e, which is what makes it a measurement-minus-reference
   * residual like the linear bracket's e_p. */
  ck_near("D7 angular estimator drive is -zeta_e", r.epsilon[0], -0.1, 1e-7);
}

/* ══════════════════════════════════════════════════════════════════════════
 *  E. alpha: constant, diagonal, and inverted the right way round
 * ══════════════════════════════════════════════════════════════════════════ */

static void group_E(void)
{
  printf("\nE. alpha -- constant, diagonal, correctly inverted\n");

  struct FmfcBracket fi;
  make_fi(&fi);
  ck("E1 alpha_fi accepted", fi.mfc.alpha_ok, "set_alpha rejected I/m");
  ck_near("E1 alpha_fi[0][0] == 1/m", fi.mfc.alpha[0][0], 1.0 / MASS, 1e-7);
  ck_near("E1 alpha_fi^-1[0][0] == m", fi.mfc.alpha_inv[0][0], MASS, 1e-6);
  ck_exact("E1 alpha_fi off-diagonal is exactly 0", fi.mfc.alpha[0][1], 0.0);
  ck_near("E1 alpha_fi^-1 off-diagonal is 0", fi.mfc.alpha_inv[0][2], 0.0, 1e-9);
  ck_near("E1 det(alpha_fi) == 1/m^3", fi.mfc.alpha_det,
          1.0 / ((double)MASS * MASS * MASS), 1e-6);

  /* E2: alpha_m = J^-1, so its INVERSE is J. A J vs J^-1 swap puts a factor of
   * 1/Ixx^2 ~ 2.2e4 on the moment command -- it would look like a wild gain
   * rather than like a bug, which is exactly why it is checked. */
  struct FmfcBracket m;
  make_m(&m);
  ck("E2 alpha_m accepted", m.mfc.alpha_ok, "set_alpha rejected J^-1");
  ck_near("E2 alpha_m[0][0] == 1/Ixx", m.mfc.alpha[0][0], 1.0 / IXX, 1e-4);
  ck_near("E2 alpha_m^-1[0][0] == Ixx (NOT 1/Ixx)", m.mfc.alpha_inv[0][0], IXX, 1e-9);
  ck_near("E2 alpha_m^-1[2][2] == Izz", m.mfc.alpha_inv[2][2], IZZ, 1e-9);
  ck_exact("E2 alpha_m off-diagonal is exactly 0", m.mfc.alpha[1][2], 0.0);

  /* E3: constant. There is no schedule and no live Jacobian in this
   * controller, so re-scheduling with the same numbers every tick must leave
   * alpha and its inverse bit-identical after a long run. */
  const double a00 = m.mfc.alpha[0][0], ai00 = m.mfc.alpha_inv[0][0];
  const float fb0[3] = { 1.f, 1.f, 1.f };
  for (int k = 1; k <= 500; k++) {
    stub_sys_time = k * (double)TS;
    fmfc_bracket_set_alpha_diag(&m, 1.f / IXX, 1.f / IYY, 1.f / IZZ);
    fmfc_bracket_run(true, &m, Z3, Z3, fb0, NULL);
  }
  ck_exact("E3 alpha unchanged after 500 ticks", m.mfc.alpha[0][0], a00);
  ck_exact("E3 alpha_inv unchanged after 500 ticks", m.mfc.alpha_inv[0][0], ai00);
}

/* ══════════════════════════════════════════════════════════════════════════
 *  F. Carried-forward facts, asserted rather than remembered
 * ══════════════════════════════════════════════════════════════════════════ */

static void group_F(void)
{
  printf("\nF. float_mat_inv_4d's return convention and its failure path\n");

  /* F1: 0 IS SUCCESS, 1 is failure -- the opposite of the usual convention.
   * Cost a full session once. */
  float in[4][4] = { {2,0,0,0}, {0,2,0,0}, {0,0,2,0}, {0,0,0,2} };
  float out[4][4];
  ck_exact("F1 float_mat_inv_4d returns 0 on success", float_mat_inv_4d(out, in), 0.0);
  ck_near("F1 the inverse is right", out[0][0], 0.5, 1e-6);

  /* F2: and on the failure path it returns 1 AND leaves the undivided adjugate
   * behind -- a matrix wrong by orders of magnitude, with no error unless the
   * caller checks. Both oneloop_findi.c and oneloop_fmfc.c check. */
  float tiny[4][4] = { {0.02f,0,0,0}, {0,0.02f,0,0}, {0,0,0.02f,0}, {0,0,0,0.02f} };
  float bad[4][4];
  const float det = 0.02f * 0.02f * 0.02f * 0.02f;   /* 1.6e-7, below the 1e-4 guard */
  ck_exact("F2 float_mat_inv_4d returns 1 when |det| is small",
           float_mat_inv_4d(bad, tiny), 1.0);
  ck("F2 the failure path leaves the UNDIVIDED adjugate",
     fabsf(bad[0][0] - 0.02f * 0.02f * 0.02f) < 1e-9f,
     "the failure path did something else; re-check the assumption");
  ck("F2 the undivided value is wrong by ~1/det",
     fabsf(bad[0][0] * (1.f / det) - (1.f / 0.02f)) < 1e-2f,
     "the 1/det relationship does not hold");
}

int main(void)
{
  printf("oneloop_fmfc: analytic property checks\n");
  printf("=====================================\n");
  group_A();
  group_B();
  group_C();
  group_D();
  group_E();
  group_F();
  printf("\n%d checks, %s\n", g_checks, g_fail ? "FAILURES PRESENT" : "all pass");
  return g_fail;
}
