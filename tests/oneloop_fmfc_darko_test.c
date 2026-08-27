/*
 * Host harness: analytic property checks on the Darko FMFC controller law
 * (oneloop_fmfc_darko_law.h) -- the two HEOL/MFC brackets, their constant
 * alpha matrices, their nominal inputs -- and on its composition with the
 * stage-1 spine (flatness_darko.c) and the inherited FINDI law
 * (oneloop_findi_darko_law.h).
 *
 * WHY PROPERTIES AND NOT A GOLDEN TRACE
 * -------------------------------------
 * There is no MATLAB in this environment and there will not be one, so there
 * are no golden traces for this controller. Worse than for the FINDI twin:
 * the Simulink reference fmfc_darko.slx was itself UNTUNED AND NOT FLYING as
 * of 2026-08-18, so even with MATLAB there would be no reference behaviour to
 * compare against.
 *
 * That is precisely why the checks below matter and why they are all
 * STRUCTURAL. Every one of them is a statement that is true of the controller
 * regardless of what the gains eventually turn out to be: a sign convention, a
 * plant identity, an algebraic equivalence, a clamp placement. None of them
 * says the loop flies, and none of them can -- offline agreement never has.
 * What they buy is that when this is finally flown, a divergence is a control
 * problem and not a transcription problem.
 *
 * What each group discriminates:
 *   [1] bracket    the HEOL invariant: epsilon = MEASURE - REFERENCE, the
 *                  nominal added OUTSIDE, and the estimator tapping the
 *                  CORRECTION alone. Getting the error sign backwards still
 *                  produces negative feedback (the core subtracts fb), so it
 *                  is invisible until the estimator diverges.
 *   [2] presat     the estimator's command tap, explicitly set on both
 *                  brackets and discriminated with a clamp that actually
 *                  bites. This is the defect that hid in HEOL for three days.
 *   [3] alpha      alpha_fi = I/m and alpha_m = inv(J) -- through the core's
 *                  own setter, so the det guard and the closed-form 3x3
 *                  inverse are exercised, not just the diagonal.
 *   [4] nominal    f* and m* reproduce the reference trajectory EXACTLY in the
 *                  nominal plant, which is the property that makes F_hat's
 *                  true value zero there.
 *   [5] tsPD       THE key identity: the bracket's own decoupled PD IS
 *                  eq. (tsPD), checked numerically against findi_darko_tspd().
 *                  If this ever stops holding, the attitude law has silently
 *                  changed.
 *   [6] clamp      the bound is on the TOTAL command, not on the correction.
 *   [7] estimator  the measured windows and hold times, and the
 *                  per-element-numerator / shared-scalar-denominator
 *                  asymmetry that a MIMO port most easily gets wrong.
 *   [8] n=2        the 2-vector callers of mfc_core_mimo are BIT-IDENTICAL
 *                  with a 3-vector channel in the same program. Verified, not
 *                  assumed.
 *   [9] chain      transform -> attitude error -> bracket -> allocate at a
 *                  hover trim closes.
 *  [10] frames     the residual is fed in the AERO frame, and a hover-roll
 *                  attitude error must come out on the aero z axis.
 */

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "firmwares/rotorcraft/stabilization/flatness_darko.h"
#include "firmwares/rotorcraft/oneloop/oneloop_fmfc_darko_law.h"

/* The harness owns the clock; see tests/stubs_darko/mcu_periph/sys_time.h. */
double stub_sys_time = 0.0;

static int g_fail = 0;
static int g_checks = 0;

static void ck(const char *name, int ok, const char *detail)
{
  g_checks++;
  printf("  %-58s %s", name, ok ? "PASS\n" : "FAIL");
  if (!ok) { printf("   <- %s\n", detail); g_fail = 1; }
}

static void ck_near(const char *name, double got, double want, double tol)
{
  char d[200];
  int ok = fabs(got - want) <= tol;
  snprintf(d, sizeof d, "got %.9g, want %.9g (tol %g)", got, want, tol);
  ck(name, ok, d);
}

static void ck_exact(const char *name, double got, double want)
{
  char d[200];
  int ok = (got == want);
  snprintf(d, sizeof d, "got %.17g, want %.17g", got, want);
  ck(name, ok, d);
}

static struct FloatVect3 v3(float x, float y, float z)
{
  struct FloatVect3 v = { x, y, z }; return v;
}

/** Body -> NED, i.e. the transpose of the NED -> body rmat. */
static struct FloatRMat transp(const struct FloatRMat *R)
{
  struct FloatRMat T;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) { RMAT_ELMT(T, i, j) = RMAT_ELMT(*R, j, i); }
  }
  return T;
}

/* The sample time every bracket in this harness runs at. Darko flies at 512 Hz
 * (PERIODIC_FREQUENCY in darko_fmfc.xml); the Simulink reference is 500. The
 * properties below are all rate-independent, which is itself worth knowing. */
#define TEST_TS (1.f / 512.f)

/** Reset a bracket with the harness clock at t = 0, so that `time` afterwards
 * is exactly the number of steps taken times TEST_TS. */
static void bracket_reset_at_zero(struct FmfcDarkoBracket *b)
{
  stub_sys_time = 0.0;
  fmfc_darko_bracket_reset(b);
}

/** Advance the clock one sample and run one bracket step. */
static void bracket_step(struct FmfcDarkoBracket *b, int k,
                         const float meas[3], const float ref[3], const float uff[3])
{
  stub_sys_time = (double)k * (double)TEST_TS;
  fmfc_darko_bracket_run(true, b, meas, ref, uff);
}

/** A bracket configured the way oneloop_fmfc_darko.c configures the ANGULAR
 * one, except that the estimator is held OFF (hold time in the far future) so
 * F_hat is identically zero and the feedback path can be read in isolation.
 *
 * Duplicating the configuration rather than including the .c is unavoidable --
 * the module pulls in state.h and generated/airframe.h -- so a change to the
 * module's init must be made in both places. This comment is the only thing
 * that says so, exactly as tests/oneloop_findi_darko_test.c says it about the
 * two calibration constants. */
static void make_angular_bracket(struct FmfcDarkoBracket *b, const float J[3],
                                 float k_xi, float k_om, bool estimator_on)
{
  fmfc_darko_bracket_init(b, TEST_TS);
  b->mfc.kp = k_xi;
  b->mfc.kd = k_om;
  b->mfc.ki = 0.f;
  b->mfc.int_window    = 20.f;
  b->mfc.est_hold_time = estimator_on ? 0.5f : 1.0e9f;
  b->mfc.est_use_presat_command  = false;
  b->mfc.use_external_derivative = true;
  b->mfc.deriv_filter            = 0.f;
  for (int i = 0; i < FMFC_DARKO_N; i++) {
    b->u_min[i] = -1.0f;
    b->u_max[i] =  1.0f;
  }
  float a[FMFC_DARKO_N];
  fmfc_darko_alpha_m_diag(J, a);
  fmfc_darko_bracket_set_alpha_diag(b, a);
  bracket_reset_at_zero(b);
}

/** The LINEAR bracket as the module configures it, estimator optionally held
 * off. */
static void make_linear_bracket(struct FmfcDarkoBracket *b, float mass,
                                float kx, float zeta_x, bool estimator_on)
{
  fmfc_darko_bracket_init(b, TEST_TS);
  b->mfc.kp = kx * kx;
  b->mfc.kd = 2.f * zeta_x * kx;
  b->mfc.ki = 0.f;
  b->mfc.int_window    = 500.f;
  b->mfc.est_hold_time = estimator_on ? 0.5f : 1.0e9f;
  b->mfc.est_use_presat_command  = false;
  b->mfc.use_external_derivative = true;
  b->mfc.deriv_filter            = 0.f;
  for (int i = 0; i < FMFC_DARKO_N; i++) {
    b->u_min[i] = -2.f * FLATNESS_DARKO_M * FLATNESS_DARKO_G;
    b->u_max[i] =  2.f * FLATNESS_DARKO_M * FLATNESS_DARKO_G;
  }
  float a[FMFC_DARKO_N];
  fmfc_darko_alpha_fi_diag(mass, a);
  fmfc_darko_bracket_set_alpha_diag(b, a);
  bracket_reset_at_zero(b);
}

/* ======================================================================== */
/* [1] the bracket: the HEOL invariant                                      */
/* ======================================================================== */

static void check_bracket_structure(const float J[3])
{
  struct FmfcDarkoBracket b;
  make_angular_bracket(&b, J, 4.f, 6.f, false);

  ck_exact("bracket: channel width is 3", b.mfc.n, 3.0);
  ck("bracket: the embedded core is DECOUPLED", b.mfc.decoupled == true,
     "the coupled structure folds kp/kd into F_hat and there would be no PD to be tsPD");
  ck("bracket: use_trajec_sp is off (the setpoint is held at 0)",
     b.mfc.use_trajec_sp == false, "a smoothed setpoint would put a nonzero ddot_sp in the law");
  ck("bracket: the core's setpoint is held at 0 on every channel",
     b.mfc.setpoint[0] == 0.f && b.mfc.setpoint[1] == 0.f && b.mfc.setpoint[2] == 0.f,
     "a nonzero setpoint double-counts the reference, which is already in epsilon");

  /* THE SIGN. epsilon = MEASURE - REFERENCE, the MFC convention, OPPOSITE to
   * the FINDI outer loop sitting next to it. */
  const float meas[3] = {  0.30f, -0.20f, 0.05f };
  const float ref[3]  = {  0.10f,  0.10f, 0.25f };
  const float uff[3]  = {  0.00f,  0.00f, 0.00f };
  const float d0[3]   = {  0.f, 0.f, 0.f };
  fmfc_darko_bracket_set_derivative(&b, d0);
  bracket_step(&b, 1, meas, ref, uff);

  ck_near("bracket: epsilon is MEASURE - REFERENCE (x)", b.epsilon[0],  0.20, 1e-6);
  ck_near("bracket: epsilon is MEASURE - REFERENCE (y)", b.epsilon[1], -0.30, 1e-6);
  ck_near("bracket: epsilon is MEASURE - REFERENCE (z)", b.epsilon[2], -0.20, 1e-6);
  ck("bracket: the core is fed epsilon, not the raw measurement",
     fabsf(b.mfc.measure[0] - b.epsilon[0]) < 1e-9f,
     "mfc.measure holds something other than the residual");
  ck_near("bracket: the core's own error equals epsilon", b.mfc.error[0][0], 0.20, 1e-6);

  /* THE NOMINAL IS ADDED OUTSIDE THE CORRECTION, and the estimator sees the
   * correction ALONE. Both halves of the HEOL invariant, in one step. */
  struct FmfcDarkoBracket c;
  make_angular_bracket(&c, J, 4.f, 6.f, false);
  const float uff2[3] = { 0.02f, -0.03f, 0.011f };
  fmfc_darko_bracket_set_derivative(&c, d0);
  bracket_step(&c, 1, meas, ref, uff2);

  for (int i = 0; i < 3; i++) {
    char n[80];
    snprintf(n, sizeof n, "bracket: command == u_ff + du, channel %d", i);
    ck_near(n, c.command[i], (double)uff2[i] + (double)c.mfc.command[0][i], 1e-6);
  }
  ck("bracket: the estimator's delayed command is the CORRECTION alone",
     c.mfc.command_est[0] == c.mfc.command[0][0]
     && c.mfc.command_est[1] == c.mfc.command[0][1]
     && c.mfc.command_est[2] == c.mfc.command[0][2],
     "the estimator was handed the total command; it would then try to explain the feedforward");
  ck("bracket: the correction differs from the total by exactly u_ff",
     fabsf((c.command[0] - c.mfc.command_est[0]) - uff2[0]) < 1e-6f,
     "the u_ff bookkeeping does not close");

  /* Nothing to correct: the bracket is a pass-through for the nominal input.
   * With the estimator held off and a zero residual this must be EXACT. */
  struct FmfcDarkoBracket d;
  make_angular_bracket(&d, J, 4.f, 6.f, false);
  fmfc_darko_bracket_set_derivative(&d, d0);
  bracket_step(&d, 1, uff2, uff2, uff2);   /* measure == reference */
  ck_exact("bracket: zero residual passes u_ff through exactly (x)", d.command[0], uff2[0]);
  ck_exact("bracket: zero residual passes u_ff through exactly (y)", d.command[1], uff2[1]);
  ck_exact("bracket: zero residual passes u_ff through exactly (z)", d.command[2], uff2[2]);
}

/* ======================================================================== */
/* [2] the estimator's command tap                                          */
/* ======================================================================== */

static void check_presat_tap(const float J[3])
{
  /* The CORE's default must stay the post-saturation tap. If someone flips
   * mfc_mimo_init() back, this fires here rather than in flight. */
  struct MfcMimoParameters raw;
  mfc_mimo_init(&raw, TEST_TS, 3);
  ck("presat: the core's default tap is the APPLIED command (false)",
     raw.est_use_presat_command == false,
     "mfc_mimo_init() now defaults to the pre-saturation tap -- see Knowledge/16");

  /* And the bracket's init must not quietly override it either way: the
   * module sets it explicitly, and this asserts that setting sticks. */
  struct FmfcDarkoBracket b;
  fmfc_darko_bracket_init(&b, TEST_TS);
  b.mfc.est_use_presat_command = true;
  ck("presat: bracket_init does not overwrite an explicit choice",
     b.mfc.est_use_presat_command == true,
     "the bracket owns a field the caller is supposed to own");

  /* DISCRIMINATING: drive a channel hard enough to rail, and check the two
   * taps actually differ. A check that cannot tell them apart is vacuous. */
  const float meas[3] = { 5.0f, 0.f, 0.f };   /* a huge residual */
  const float ref[3]  = { 0.f, 0.f, 0.f };
  const float uff[3]  = { 0.f, 0.f, 0.f };
  const float d0[3]   = { 0.f, 0.f, 0.f };

  struct FmfcDarkoBracket post, pre;
  make_angular_bracket(&post, J, 4.f, 6.f, false);
  make_angular_bracket(&pre,  J, 4.f, 6.f, false);
  pre.mfc.est_use_presat_command = true;
  fmfc_darko_bracket_set_derivative(&post, d0);
  fmfc_darko_bracket_set_derivative(&pre,  d0);
  /* Bring the clamp in tight so it certainly bites. */
  for (int i = 0; i < 3; i++) {
    post.u_min[i] = -0.001f; post.u_max[i] = 0.001f;
    pre.u_min[i]  = -0.001f; pre.u_max[i]  = 0.001f;
  }
  bracket_step(&post, 1, meas, ref, uff);
  bracket_step(&pre,  1, meas, ref, uff);

  ck("presat: the clamp actually bit (the check is not vacuous)",
     fabsf(post.mfc.command_presat[0]) > fabsf(post.mfc.command[0][0]) + 1e-6f,
     "no saturation occurred, so the two taps cannot be distinguished");
  ck("presat: false taps the APPLIED (clamped) command",
     post.mfc.command_est[0] == post.mfc.command[0][0],
     "the post-saturation tap is not reading the applied command");
  ck("presat: true taps the PRE-saturation command",
     pre.mfc.command_est[0] == pre.mfc.command_presat[0],
     "the pre-saturation tap is not reading the raw inversion output");
  ck("presat: the two taps differ once the clamp bites",
     post.mfc.command_est[0] != pre.mfc.command_est[0],
     "the two configurations are indistinguishable, so the setting is inert");
}

/* ======================================================================== */
/* [3] the alpha matrices -- plant algebra                                  */
/* ======================================================================== */

static void check_alpha(struct FlatnessDarkoParams *p)
{
  const float J[3] = { p->Jxx, p->Jyy, p->Jzz };

  float afi[3], am[3];
  fmfc_darko_alpha_fi_diag(p->m, afi);
  fmfc_darko_alpha_m_diag(J, am);

  ck_near("alpha: alpha_fi is 1/m on every diagonal entry (x)", afi[0], 1.0 / p->m, 1e-6);
  ck("alpha: alpha_fi is ISOTROPIC (the same on all three axes)",
     afi[0] == afi[1] && afi[1] == afi[2],
     "the linear bracket's input gain is not (1/m) I3");
  ck_near("alpha: alpha_m is 1/Jxx on the aero x axis", am[0], 1.0 / p->Jxx, 1e-4);
  ck_near("alpha: alpha_m is 1/Jyy on the aero y axis", am[1], 1.0 / p->Jyy, 1e-4);
  ck_near("alpha: alpha_m is 1/Jzz on the aero z axis", am[2], 1.0 / p->Jzz, 1e-4);
  ck("alpha: alpha_m is ANISOTROPIC (Darko's inertias differ by 2.5x)",
     am[0] != am[1] && am[1] != am[2],
     "the three inertias came out equal, so J was probably not read");

  /* Through the CORE's own setter, so the det guard and the closed-form 3x3
   * cofactor inverse are what gets checked -- not just the diagonal we handed
   * in. alpha^-1 must be m*I and J respectively. */
  struct FmfcDarkoBracket lin, ang;
  fmfc_darko_bracket_init(&lin, TEST_TS);
  fmfc_darko_bracket_init(&ang, TEST_TS);
  fmfc_darko_bracket_set_alpha_diag(&lin, afi);
  fmfc_darko_bracket_set_alpha_diag(&ang, am);

  ck("alpha: the core accepted alpha_fi (not rejected as singular)", lin.mfc.alpha_ok, "det guard fired");
  ck("alpha: the core accepted alpha_m (not rejected as singular)",  ang.mfc.alpha_ok, "det guard fired");
  ck_near("alpha: inv(alpha_fi) is m on the diagonal", lin.mfc.alpha_inv[1][1], p->m, 1e-5);
  ck_near("alpha: inv(alpha_m) is Jxx on the aero x axis", ang.mfc.alpha_inv[0][0], p->Jxx, 1e-7);
  ck_near("alpha: inv(alpha_m) is Jyy on the aero y axis", ang.mfc.alpha_inv[1][1], p->Jyy, 1e-7);
  ck_near("alpha: inv(alpha_m) is Jzz on the aero z axis", ang.mfc.alpha_inv[2][2], p->Jzz, 1e-7);

  float worst = 0.f;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      if (i == j) { continue; }
      if (fabsf(ang.mfc.alpha[i][j]) > worst)     { worst = fabsf(ang.mfc.alpha[i][j]); }
      if (fabsf(ang.mfc.alpha_inv[i][j]) > worst) { worst = fabsf(ang.mfc.alpha_inv[i][j]); }
    }
  }
  ck_exact("alpha: alpha_m and its inverse are exactly diagonal", worst, 0.0);

  /* No conditioning question: unlike the HEOL horizontal channel's
   * anti-diagonal Jacobian, both of these are fixed positive diagonals. */
  ck("alpha: det(alpha_m) is positive and far from the guard",
     ang.mfc.alpha_det > 1000.f * MFC_MIMO_DET_MIN_ABS,
     "the moment bracket's alpha is near-singular, which it cannot be for a real inertia");
  ck("alpha: det(alpha_fi) is positive and far from the guard",
     lin.mfc.alpha_det > 1000.f * MFC_MIMO_DET_MIN_ABS,
     "the force bracket's alpha is near-singular, which it cannot be for a real mass");
}

/* ======================================================================== */
/* [4] the nominal (flat) inputs                                            */
/* ======================================================================== */

static void check_nominal_inputs(struct FlatnessDarkoParams *p)
{
  const float J[3] = { p->Jxx, p->Jyy, p->Jzz };

  /* Hover. NED z is down, so holding still needs an UPWARD (negative z) force. */
  const float a_hover[3] = { 0.f, 0.f, 0.f };
  float f_star[3];
  fmfc_darko_nominal_force(p->m, p->g, a_hover, f_star);
  ck_exact("nominal: f* has no horizontal component at hover (x)", f_star[0], 0.0);
  ck_exact("nominal: f* has no horizontal component at hover (y)", f_star[1], 0.0);
  ck_near("nominal: f* at hover is -m g (upward in NED)", f_star[2], -(double)p->m * p->g, 1e-5);

  /* A commanded acceleration. */
  const float a_ref[3] = { 1.5f, -0.75f, -2.0f };
  fmfc_darko_nominal_force(p->m, p->g, a_ref, f_star);
  ck_near("nominal: f*_x is m a_ref_x", f_star[0], (double)p->m * 1.5, 1e-5);
  ck_near("nominal: f*_z is m (a_ref_z - g)", f_star[2], (double)p->m * (-2.0 - p->g), 1e-4);

  /* THE PROPERTY THAT MAKES f* CORRECT: in the nominal plant, applying f*
   * reproduces a_ref exactly, so the residual the estimator is left to find is
   * IDENTICALLY ZERO. pdd = (1/m) f_i + g e_z. */
  for (int i = 0; i < 3; i++) {
    const double pdd = (double)f_star[i] / p->m + (i == 2 ? p->g : 0.0);
    char n[90];
    snprintf(n, sizeof n, "nominal: f* reproduces a_ref in the nominal plant (%c)", "xyz"[i]);
    ck_near(n, pdd, a_ref[i], 1e-4);
  }

  /* m* = J Omdot_ref, and it is ZERO at hover -- which is why the angular
   * feedforward is invisible in this stage and must still be written out. */
  const float omdot_hover[3] = { 0.f, 0.f, 0.f };
  float m_star[3];
  fmfc_darko_nominal_moment(J, omdot_hover, m_star);
  ck("nominal: m* is zero at hover", m_star[0] == 0.f && m_star[1] == 0.f && m_star[2] == 0.f,
     "a nonzero hover feedforward moment would bias the allocation");

  const float omdot_ref[3] = { 2.0f, -3.0f, 0.5f };
  fmfc_darko_nominal_moment(J, omdot_ref, m_star);
  ck_near("nominal: m*_x is Jxx Omdot_ref_x", m_star[0], 2.0 * p->Jxx, 1e-9);
  ck_near("nominal: m*_y is Jyy Omdot_ref_y", m_star[1], -3.0 * p->Jyy, 1e-9);
  for (int i = 0; i < 3; i++) {
    const double omdot = (double)m_star[i] / J[i];
    char n[90];
    snprintf(n, sizeof n, "nominal: m* reproduces Omdot_ref in the nominal plant (%c)", "xyz"[i]);
    ck_near(n, omdot, omdot_ref[i], 1e-5);
  }
  /* m* is EXACTLY J Omdot_ref -- purely linear in Omdot_ref, with no additive
   * gyroscopic or external-moment term. Doubling the reference must double the
   * feedforward; an Om x J Om term would break that and would double-count the
   * moment the paper already folds into the estimator. */
  float m_star2[3];
  const float omdot2[3] = { 2.f * omdot_ref[0], 2.f * omdot_ref[1], 2.f * omdot_ref[2] };
  fmfc_darko_nominal_moment(J, omdot2, m_star2);
  ck("nominal: m* is exactly linear in Omdot_ref (no gyroscopic term)",
     fabsf(m_star2[0] - 2.f * m_star[0]) < 1e-9f
     && fabsf(m_star2[1] - 2.f * m_star[1]) < 1e-9f
     && fabsf(m_star2[2] - 2.f * m_star[2]) < 1e-9f,
     "m* carries an additive term; the paper folds those into m_lpf/F_hat and adding one double-counts");
}

/* ======================================================================== */
/* [5] THE tsPD IDENTITY                                                    */
/* ======================================================================== */

/** With eps = -zeta_e and d(eps)/dt = Om_lpf - Om_ref, the decoupled core's
 * own feedback term IS eq. (tsPD) for kp = k_xi, kd = k_om. This is the whole
 * reason the MFC bracket can be dropped onto the FINDI attitude law without
 * changing the law, and it is asserted numerically against
 * findi_darko_tspd() so that it stays true. */
static void check_tspd_identity(struct FlatnessDarkoParams *p)
{
  const float J[3] = { p->Jxx, p->Jyy, p->Jzz };
  const float k_xi = 4.f, k_om = 6.f;

  struct FloatVect3 zeta_e = v3(0.13f, -0.07f, 0.21f);
  struct FloatVect3 om_ref = v3(0.f, 0.f, 0.f);
  struct FloatVect3 om_lpf = v3(0.5f, -0.9f, 0.3f);
  struct FloatVect3 omdot_c;
  findi_darko_tspd(k_xi, k_om, &zeta_e, &om_ref, &om_lpf, &omdot_c);

  struct FmfcDarkoBracket b;
  make_angular_bracket(&b, J, k_xi, k_om, false);   /* estimator held off */
  const float eps[3]   = { -zeta_e.x, -zeta_e.y, -zeta_e.z };
  const float ref[3]   = { 0.f, 0.f, 0.f };
  const float m_star[3] = { 0.f, 0.f, 0.f };
  const float d_eps[3] = { om_lpf.x - om_ref.x, om_lpf.y - om_ref.y, om_lpf.z - om_ref.z };
  fmfc_darko_bracket_set_derivative(&b, d_eps);
  bracket_step(&b, 1, eps, ref, m_star);

  ck("tsPD: F_hat is held at zero for this check", b.mfc.estimator[0] == 0.f,
     "the estimator is live, so the identity below is not isolating the PD");

  const double want[3] = { (double)J[0] * omdot_c.x,
                           (double)J[1] * omdot_c.y,
                           (double)J[2] * omdot_c.z };
  for (int i = 0; i < 3; i++) {
    char n[100];
    snprintf(n, sizeof n, "tsPD: the bracket reproduces J*findi_darko_tspd (%c)", "xyz"[i]);
    ck_near(n, b.command[i], want[i], 1e-6);
  }

  /* Om_ref enters with the OPPOSITE sign to Om_lpf. A transcription that
   * dropped Om_ref, or added it, passes the check above and fails here. */
  om_ref = v3(0.2f, 0.1f, -0.4f);
  findi_darko_tspd(k_xi, k_om, &zeta_e, &om_ref, &om_lpf, &omdot_c);
  struct FmfcDarkoBracket b2;
  make_angular_bracket(&b2, J, k_xi, k_om, false);
  const float d_eps2[3] = { om_lpf.x - om_ref.x, om_lpf.y - om_ref.y, om_lpf.z - om_ref.z };
  fmfc_darko_bracket_set_derivative(&b2, d_eps2);
  bracket_step(&b2, 1, eps, ref, m_star);
  ck_near("tsPD: the identity survives a nonzero Om_ref (y)", b2.command[1],
          (double)J[1] * omdot_c.y, 1e-6);

  /* DISCRIMINATING: feeding reference-minus-measured instead must break it.
   * If it did not, the derivative sign convention would be untested. */
  struct FmfcDarkoBracket b3;
  make_angular_bracket(&b3, J, k_xi, k_om, false);
  const float d_flipped[3] = { -d_eps[0], -d_eps[1], -d_eps[2] };
  fmfc_darko_bracket_set_derivative(&b3, d_flipped);
  bracket_step(&b3, 1, eps, ref, m_star);
  ck("tsPD: an inverted derivative sign breaks the identity",
     fabs((double)b3.command[1] - want[1]) > 1e-4,
     "the derivative sign convention is not actually being tested");

  /* And the nominal moment still adds outside. */
  struct FmfcDarkoBracket b4;
  make_angular_bracket(&b4, J, k_xi, k_om, false);
  const float m_star2[3] = { 0.004f, -0.002f, 0.001f };
  fmfc_darko_bracket_set_derivative(&b4, d_eps);
  findi_darko_tspd(k_xi, k_om, &zeta_e, &(struct FloatVect3){0.f,0.f,0.f}, &om_lpf, &omdot_c);
  bracket_step(&b4, 1, eps, ref, m_star2);
  ck_near("tsPD: m_c == m* + J*Omdot_c (the feedforward is outside)", b4.command[0],
          (double)m_star2[0] + (double)J[0] * omdot_c.x, 1e-6);

  /* Zero error, zero rate: the loop does nothing. */
  struct FmfcDarkoBracket b5;
  make_angular_bracket(&b5, J, k_xi, k_om, false);
  const float z3[3] = { 0.f, 0.f, 0.f };
  fmfc_darko_bracket_set_derivative(&b5, z3);
  bracket_step(&b5, 1, z3, z3, z3);
  ck_exact("tsPD: zero error and zero rate command no moment (x)", b5.command[0], 0.0);
  ck_exact("tsPD: zero error and zero rate command no moment (y)", b5.command[1], 0.0);
  ck_exact("tsPD: zero error and zero rate command no moment (z)", b5.command[2], 0.0);

  /* The second-order reading, and the gain that MOVED. Both are recorded here
   * so the delta is in the test output, not only in a comment. */
  const double wn   = sqrt((double)k_xi);
  const double zeta = k_om / (2.0 * wn);
  ck_near("tsPD: FMFC's (4, 6) reads as wn = 2 rad/s",   wn,   2.0, 1e-9);
  ck_near("tsPD: FMFC's (4, 6) reads as zeta = 1.5",     zeta, 1.5, 1e-9);
  const double wn_f   = sqrt(25.0);
  const double zeta_f = 7.0 / (2.0 * wn_f);
  ck_near("tsPD: findi_darko's MEASURED (25, 7) is wn = 5", wn_f,   5.0, 1e-9);
  ck_near("tsPD: findi_darko's MEASURED (25, 7) is zeta = 0.7", zeta_f, 0.7, 1e-9);
  printf("      NOTE the inner loop moved: FMFC wn %.2f zeta %.2f against the only\n"
         "      Darko point with evidence behind it, findi's wn %.2f zeta %.2f.\n"
         "      PLACEHOLDER GAINS. Report, do not tune.\n", wn, zeta, wn_f, zeta_f);
}

/* ======================================================================== */
/* [6] the clamp bounds the TOTAL, not the correction                       */
/* ======================================================================== */

static void check_clamp_on_total(const float J[3])
{
  const float ref[3] = { 0.f, 0.f, 0.f };
  const float d0[3]  = { 0.f, 0.f, 0.f };

  /* A large residual with a nonzero nominal: the TOTAL must land on the rail,
   * not the correction. If the clamp were on the correction alone the total
   * would come out at u_ff + u_max. */
  struct FmfcDarkoBracket b;
  make_angular_bracket(&b, J, 4.f, 6.f, false);
  for (int i = 0; i < 3; i++) { b.u_min[i] = -0.05f; b.u_max[i] = 0.05f; }
  const float eps[3]  = { -10.f, 0.f, 0.f };   /* drives the command hard positive */
  const float uff[3]  = {  0.02f, 0.f, 0.f };
  fmfc_darko_bracket_set_derivative(&b, d0);
  bracket_step(&b, 1, eps, ref, uff);
  ck_near("clamp: the TOTAL command lands on u_max, not u_ff + u_max",
          b.command[0], 0.05, 1e-6);

  /* And on the other rail. */
  struct FmfcDarkoBracket c;
  make_angular_bracket(&c, J, 4.f, 6.f, false);
  for (int i = 0; i < 3; i++) { c.u_min[i] = -0.05f; c.u_max[i] = 0.05f; }
  const float eps2[3] = { 10.f, 0.f, 0.f };
  fmfc_darko_bracket_set_derivative(&c, d0);
  bracket_step(&c, 1, eps2, ref, uff);
  ck_near("clamp: the TOTAL command lands on u_min on the other rail",
          c.command[0], -0.05, 1e-6);

  /* u_ff already outside the window: the window collapses onto the nearest
   * reachable edge rather than inverting (an inverted window passes anything). */
  struct FmfcDarkoBracket d;
  make_angular_bracket(&d, J, 4.f, 6.f, false);
  for (int i = 0; i < 3; i++) { d.u_min[i] = -0.05f; d.u_max[i] = 0.05f; }
  const float uff_out[3] = { 0.5f, 0.f, 0.f };   /* well past u_max */
  fmfc_darko_bracket_set_derivative(&d, d0);
  bracket_step(&d, 1, eps, ref, uff_out);
  ck("clamp: an out-of-range u_ff never produces an INVERTED window",
     d.mfc.u_min[0] <= d.mfc.u_max[0],
     "the correction window is inverted, so the clamp would pass anything");
  ck("clamp: the window forces the correction NEGATIVE to pull u_ff back in",
     d.mfc.u_max[0] < 0.f,
     "an out-of-range u_ff was not required to be corrected downwards");
  ck_near("clamp: the total then sits on the reachable edge", d.command[0], 0.05, 1e-6);
  /* NOTE the `lo > hi` collapse branch in fmfc_darko_bracket_run() (and in
   * heol_mimo.c, from which it is inherited) is UNREACHABLE while
   * u_min <= u_max, because lo > hi reduces to u_min > u_max. It is kept as a
   * guard against a caller that inverts its own bounds, and this check records
   * that the normal path already gets the answer right without it. */
}

/* ======================================================================== */
/* [7] estimator windows, hold times, and the MIMO asymmetry                */
/* ======================================================================== */

static void check_estimator(struct FlatnessDarkoParams *p)
{
  const float J[3] = { p->Jxx, p->Jyy, p->Jzz };

  /* The measured constants, asserted so a copy-paste between the two brackets
   * cannot go unnoticed. 500 against 20 is a 25:1 ratio and it is the point. */
  struct FmfcDarkoBracket lin, ang;
  make_linear_bracket(&lin, p->m, 1.8898f, 1.1093f, true);
  make_angular_bracket(&ang, J, 4.f, 6.f, true);
  ck_exact("estimator: the LINEAR window is FFilter_fi = 500",   lin.mfc.int_window, 500.0);
  ck_exact("estimator: the LINEAR hold time is 0.5 s",           lin.mfc.est_hold_time, 0.5);
  ck_exact("estimator: the ANGULAR window is FFilter_m = 20",    ang.mfc.int_window, 20.0);
  ck_exact("estimator: the ANGULAR hold time is 0.5 s",          ang.mfc.est_hold_time, 0.5);
  ck("estimator: the two windows are NOT the same (25:1)",
     lin.mfc.int_window != ang.mfc.int_window,
     "both brackets got one window, so one of them was copy-pasted");

  /* F_hat is pinned at exactly zero until the hold time elapses, then is free
   * to move. 512 Hz, so 0.5 s is step 256. */
  struct FmfcDarkoBracket b;
  make_angular_bracket(&b, J, 4.f, 6.f, true);
  const float eps[3] = { 0.05f, -0.02f, 0.01f };
  const float ref[3] = { 0.f, 0.f, 0.f };
  const float uff[3] = { 0.f, 0.f, 0.f };
  const float d0[3]  = { 0.f, 0.f, 0.f };
  fmfc_darko_bracket_set_derivative(&b, d0);
  for (int k = 1; k <= 200; k++) { bracket_step(&b, k, eps, ref, uff); }
  ck("estimator: F_hat is exactly zero before the hold time",
     b.mfc.estimator[0] == 0.f && b.mfc.estimator[1] == 0.f && b.mfc.estimator[2] == 0.f,
     "the estimator went live inside its hold window");
  for (int k = 201; k <= 400; k++) { bracket_step(&b, k, eps, ref, uff); }
  ck("estimator: F_hat is live after the hold time",
     b.mfc.estimator[0] != 0.f,
     "the estimator never went live, so the loop is running open on F_hat");

  /* THE MIMO ASYMMETRY: the numerator is per element, the denominator is
   * SCALAR AND SHARED. Drive three channels with residuals in a fixed ratio
   * and the estimates must come out in that same ratio -- which can only
   * happen if one denominator serves all three. */
  struct FmfcDarkoBracket r;
  make_angular_bracket(&r, J, 4.f, 6.f, true);
  const float eps_r[3] = { 0.01f, 0.02f, 0.04f };   /* 1 : 2 : 4 */
  fmfc_darko_bracket_set_derivative(&r, d0);
  for (int k = 1; k <= 400; k++) { bracket_step(&r, k, eps_r, ref, uff); }
  ck("estimator: the drive produced a nonzero estimate to compare",
     fabsf(r.mfc.estimator[0]) > 0.f, "no estimate to take a ratio of");
  ck_near("estimator: F_hat[1]/F_hat[0] == 2 (one shared denominator)",
          r.mfc.estimator[1] / r.mfc.estimator[0], 2.0, 1e-3);
  ck_near("estimator: F_hat[2]/F_hat[0] == 4 (one shared denominator)",
          r.mfc.estimator[2] / r.mfc.estimator[0], 4.0, 1e-3);
}

/* ======================================================================== */
/* [8] the 2-vector callers stay bit-identical                              */
/* ======================================================================== */

/** Run a fixed n = 2 sequence -- the HEOL horizontal channel's shape, with its
 * ANTI-DIAGONAL alpha -- and record the command trace. */
static void run_n2_trace(float out[40][2])
{
  struct MfcMimoParameters m;
  mfc_mimo_init(&m, TEST_TS, 2);
  m.kp = 4.f; m.kd = 6.f; m.ki = 0.f;
  m.int_window = 200.f;
  m.est_hold_time = 0.05f;
  m.decoupled = true;
  m.est_use_presat_command = false;
  /* The horizontal channel's alpha at level: a[0][0] is EXACTLY ZERO, which is
   * why the core uses a closed-form cofactor inverse and not float_mat_invert. */
  float a[MFC_MIMO_N_MAX][MFC_MIMO_N_MAX] = {{0.f}};
  a[0][0] =  0.f;    a[0][1] = -9.81f;
  a[1][0] =  9.81f;  a[1][1] =  0.f;
  mfc_mimo_set_alpha(&m, a);

  stub_sys_time = 0.0;
  mfc_mimo_reset(&m);
  for (int k = 1; k <= 40; k++) {
    stub_sys_time = (double)k * (double)TEST_TS;
    const float meas[2] = { 0.1f * sinf(0.3f * k), 0.05f * cosf(0.2f * k) };
    mfc_mimo_run(true, &m, meas);
    out[k - 1][0] = m.command[0][0];
    out[k - 1][1] = m.command[0][1];
  }
}

static void check_n2_unaffected(const float J[3])
{
  float alone[40][2], interleaved[40][2];

  run_n2_trace(alone);

  /* The same n = 2 channel again, with a live n = 3 bracket stepped between
   * every sample. Different struct, so this is really asserting that the core
   * carries no per-width shared state -- and that the closed-form 2x2 branch
   * is still taken. */
  struct FmfcDarkoBracket b;
  make_angular_bracket(&b, J, 4.f, 6.f, true);
  const float e3[3] = { 0.02f, -0.01f, 0.03f };
  const float r3[3] = { 0.f, 0.f, 0.f };
  const float u3[3] = { 0.f, 0.f, 0.f };
  const float d3[3] = { 0.f, 0.f, 0.f };
  fmfc_darko_bracket_set_derivative(&b, d3);
  for (int k = 1; k <= 40; k++) { bracket_step(&b, k, e3, r3, u3); }

  run_n2_trace(interleaved);

  ck("n=2: the 2-vector trace is BIT-IDENTICAL with a 3-vector channel present",
     memcmp(alone, interleaved, sizeof alone) == 0,
     "a 3-channel caller perturbed the 2-channel arithmetic");

  /* And the 2x2 inverse itself, against the closed form, at the alpha that
   * would break a naive Gauss-Jordan. */
  struct MfcMimoParameters m;
  mfc_mimo_init(&m, TEST_TS, 2);
  float a[MFC_MIMO_N_MAX][MFC_MIMO_N_MAX] = {{0.f}};
  a[0][1] = -9.81f; a[1][0] = 9.81f;
  mfc_mimo_set_alpha(&m, a);
  ck("n=2: an anti-diagonal alpha is still inverted (no pivot needed)", m.alpha_ok,
     "the 2x2 solve rejected the horizontal channel's own operating alpha");
  ck_near("n=2: inv([[0,-g],[g,0]])[0][1] == 1/g", m.alpha_inv[0][1], 1.0 / 9.81, 1e-6);
  ck_near("n=2: inv([[0,-g],[g,0]])[1][0] == -1/g", m.alpha_inv[1][0], -1.0 / 9.81, 1e-6);
  ck_exact("n=2: the width stayed 2", m.n, 2.0);
}

/* ======================================================================== */
/* [9] the hover chain, end to end                                          */
/* ======================================================================== */

/** f* -> linear bracket -> transform -> attitude error -> angular bracket ->
 * allocate, at a settled hover. Nothing to correct anywhere, so the chain must
 * reproduce the hover actuator command exactly. */
static void check_hover_chain(struct FlatnessDarkoParams *p)
{
  const float J[3] = { p->Jxx, p->Jyy, p->Jzz };
  const struct FloatVect3 v  = v3(0.f, 0.f, 0.f);
  const struct FloatVect3 by = v3(0.f, 1.f, 0.f);

  /* Linear bracket at trim: measurement == reference, a_ref = 0. Estimator
   * held off so the check is about the STRUCTURE, not about a converged
   * estimate. */
  struct FmfcDarkoBracket lin;
  make_linear_bracket(&lin, p->m, 1.8898f, 1.1093f, false);
  const float a_ref[3] = { 0.f, 0.f, 0.f };
  float f_star[3];
  fmfc_darko_nominal_force(p->m, p->g, a_ref, f_star);
  const float p_meas[3] = { 3.f, -2.f, -7.f };
  const float p_ref[3]  = { 3.f, -2.f, -7.f };
  const float d_eps[3]  = { 0.f, 0.f, 0.f };
  fmfc_darko_bracket_set_derivative(&lin, d_eps);
  bracket_step(&lin, 1, p_meas, p_ref, f_star);

  ck_near("chain: at trim the commanded force is exactly f* = -m g",
          lin.command[2], -(double)p->m * p->g, 1e-5);
  ck_exact("chain: no horizontal force is commanded at trim (x)", lin.command[0], 0.0);
  ck_exact("chain: no horizontal force is commanded at trim (y)", lin.command[1], 0.0);

  struct FloatVect3 fi_c = v3(lin.command[0], lin.command[1], lin.command[2]);
  findi_darko_limit_tilt(&fi_c, 0.5236f);
  ck_exact("chain: the tilt limiter leaves a pure hover demand alone", fi_c.z,
           (double)lin.command[2]);

  struct FlatnessDarkoForce o;
  const bool ok = flatness_darko_force_transform(p, &fi_c, 0.f, &v, 0.f, &by, &o);
  ck("chain: the transform succeeds at hover", ok, "roll reported undetermined at hover");
  ck_near("chain: HOVER-frame theta is 0", o.euler.theta, 0.0, 1e-6);
  ck_near("chain: AERO-frame theta is +pi/2", o.theta_aero, M_PI / 2., 1e-6);

  /* Measured attitude == commanded attitude: zero residual into the angular
   * bracket, so it must command exactly m* = 0. */
  struct FloatVect3 zeta_hover, zeta_aero;
  flatness_quad_att_error(&o.quat, &o.quat, &zeta_hover);
  findi_darko_hover_to_aero(&zeta_hover, &zeta_aero);
  ck_near("chain: the attitude error is zero at the commanded attitude",
          sqrt((double)(zeta_aero.x * zeta_aero.x + zeta_aero.y * zeta_aero.y
                        + zeta_aero.z * zeta_aero.z)), 0.0, 1e-6);

  struct FmfcDarkoBracket ang;
  make_angular_bracket(&ang, J, 4.f, 6.f, false);
  const float eps_a[3] = { -zeta_aero.x, -zeta_aero.y, -zeta_aero.z };
  const float ref_a[3] = { 0.f, 0.f, 0.f };
  const float m_star[3] = { 0.f, 0.f, 0.f };
  const float d_a[3] = { 0.f, 0.f, 0.f };     /* settled: no rate */
  fmfc_darko_bracket_set_derivative(&ang, d_a);
  bracket_step(&ang, 1, eps_a, ref_a, m_star);
  ck_near("chain: settled hover commands no moment",
          sqrt((double)(ang.command[0] * ang.command[0] + ang.command[1] * ang.command[1]
                        + ang.command[2] * ang.command[2])), 0.0, 1e-6);

  struct FloatVect3 m_c = v3(ang.command[0], ang.command[1], ang.command[2]);
  const struct FloatRMat R_i_b = transp(&o.rmat_aero);
  float u[4]; int8_t sat[4];
  const bool a_ok = flatness_darko_allocate(p, o.T, &m_c, &v, &R_i_b, u, sat);
  ck("chain: allocation succeeds at hover", a_ok, "allocator reported no flap authority");

  const float w_h = sqrtf((o.T * 0.5f) / p->cT);
  ck_near("chain: the loop reproduces the trim rotor speed", fabsf(u[0]), w_h, 1e-2);
  ck_near("chain: rotor 2 is emitted NEGATIVE", u[1], -w_h, 1e-2);
  ck_near("chain: the flaps stay at zero",
          sqrt((double)(u[2] * u[2] + u[3] * u[3])), 0.0, 1e-4);

  /* The two SI->command calibration constants, duplicated here because the
   * module cannot be compiled on the host. A change to either must be made in
   * both places; this comment is the only thing that says so. */
  const float c1 = findi_darko_w_to_cmd(u[0], 1300.0f);
  const float c2 = findi_darko_w_to_cmd(u[1], 1300.0f);
  printf("      hover commit: T = %.4f N, |w| = %.2f rad/s, motor cmds %.4f / %.4f\n",
         o.T, w_h, c1, c2);
  ck("chain: both motor commands land in (0.2, 0.8)",
     c1 > 0.2f && c1 < 0.8f && c2 > 0.2f && c2 < 0.8f,
     "hover trim is off the usable motor range");
}

/* ======================================================================== */
/* [10] the residual is fed in the AERO frame                               */
/* ======================================================================== */

/** The angular bracket runs entirely in the paper's AERO frame, and the frame
 * map inverts the axis correspondence a tailsitter reader expects: hover ROLL
 * is aero YAW and goes to differential THRUST. Fed in the wrong frame the
 * controller rolls when told to yaw -- a plausible-looking wrong answer, not a
 * crash. Driven here through the bracket and the REAL allocator. */
static void check_aero_frame_residual(struct FlatnessDarkoParams *p)
{
  const float J[3] = { p->Jxx, p->Jyy, p->Jzz };
  const struct FloatVect3 v = v3(0.f, 0.f, 0.f);

  struct FloatRMat R_h_i;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) { RMAT_ELMT(R_h_i, i, j) = (i == j) ? 1.f : 0.f; }
  }
  struct FloatRMat R_b_i;
  findi_darko_rmat_hover_to_aero(&R_h_i, &R_b_i);
  const struct FloatRMat R_i_b = transp(&R_b_i);
  const float T = p->m * p->g / (1.f - p->cDT);

  /* A pure HOVER-ROLL attitude error, converted to AERO and fed as -zeta_e. */
  struct FloatVect3 zeta_hover = v3(0.10f, 0.f, 0.f), zeta_aero;
  findi_darko_hover_to_aero(&zeta_hover, &zeta_aero);
  ck("frame: a hover-ROLL error lands on the AERO Z axis",
     zeta_aero.z == 0.10f && zeta_aero.x == 0.f && zeta_aero.y == 0.f,
     "the frame map is transposed; roll would come out as yaw");

  struct FmfcDarkoBracket b;
  make_angular_bracket(&b, J, 4.f, 6.f, false);
  const float eps[3] = { -zeta_aero.x, -zeta_aero.y, -zeta_aero.z };
  const float ref[3] = { 0.f, 0.f, 0.f };
  const float uff[3] = { 0.f, 0.f, 0.f };
  const float d0[3]  = { 0.f, 0.f, 0.f };
  fmfc_darko_bracket_set_derivative(&b, d0);
  bracket_step(&b, 1, eps, ref, uff);

  ck("frame: the moment command comes out on the AERO z axis",
     fabsf(b.command[2]) > 1e-4f && fabsf(b.command[0]) < 1e-9f
     && fabsf(b.command[1]) < 1e-9f,
     "a pure roll error produced a moment on the wrong aero axis");
  ck("frame: a positive hover-roll error commands a positive aero-z moment",
     b.command[2] > 0.f, "the attitude loop's sign is inverted in roll");

  struct FloatVect3 m_c = v3(b.command[0], b.command[1], b.command[2]);
  float u[4]; int8_t sat[4];
  flatness_darko_allocate(p, T, &m_c, &v, &R_i_b, u, sat);
  ck("frame: it reaches the actuators as DIFFERENTIAL THRUST",
     fabsf(u[0]) - fabsf(u[1]) > 1.f, "the rotors stayed matched on a roll demand");
  ck("frame: and the flaps stay near zero",
     fabsf(u[2]) < 0.05f && fabsf(u[3]) < 0.05f, "the flaps absorbed a roll demand");
  ck("frame: positive hover ROLL speeds up rotor 1 (the LEFT motor)",
     fabsf(u[0]) > fabsf(u[1]), "the roll sense is inverted");

  /* A pure HOVER-PITCH error: same axis in both frames, common-mode flap. */
  struct FloatVect3 q_hover = v3(0.f, 0.05f, 0.f), q_aero;
  findi_darko_hover_to_aero(&q_hover, &q_aero);
  struct FmfcDarkoBracket c;
  make_angular_bracket(&c, J, 4.f, 6.f, false);
  const float epsq[3] = { -q_aero.x, -q_aero.y, -q_aero.z };
  fmfc_darko_bracket_set_derivative(&c, d0);
  bracket_step(&c, 1, epsq, ref, uff);
  struct FloatVect3 mq = v3(c.command[0], c.command[1], c.command[2]);
  flatness_darko_allocate(p, T, &mq, &v, &R_i_b, u, sat);
  ck("frame: a hover-PITCH error comes out as COMMON-MODE flap",
     fabsf(u[2] + u[3]) > 1e-3f && fabsf(u[2] - u[3]) < 1e-3f,
     "pitch did not produce a symmetric flap pair");

  /* THE INERTIA IS ANISOTROPIC, so the same attitude error on two axes must
   * NOT give the same moment. Catches a J that was never read. */
  ck("frame: equal errors on x and z give different moments (J is read)",
     fabsf(fabsf(b.command[2]) - fabsf(c.command[1])) > 1e-6f,
     "the moment does not depend on the axis, so alpha_m is probably identity");
}

/* ======================================================================== */

int main(void)
{
  struct FlatnessDarkoParams p;
  flatness_darko_params_init(&p);
  const float J[3] = { p.Jxx, p.Jyy, p.Jzz };

  printf("Darko FMFC controller law -- analytic property checks\n");
  printf("INTEGRATION ONLY: the gains are placeholders and passing these checks\n"
         "says nothing about whether the loop flies.\n\n");

  printf("[1] the bracket: the HEOL invariant\n");
  check_bracket_structure(J);

  printf("\n[2] the estimator's command tap\n");
  check_presat_tap(J);

  printf("\n[3] the alpha matrices\n");
  check_alpha(&p);

  printf("\n[4] the nominal (flat) inputs\n");
  check_nominal_inputs(&p);

  printf("\n[5] the tsPD identity\n");
  check_tspd_identity(&p);

  printf("\n[6] the clamp bounds the TOTAL\n");
  check_clamp_on_total(J);

  printf("\n[7] estimator windows and the MIMO asymmetry\n");
  check_estimator(&p);

  printf("\n[8] the 2-vector callers stay bit-identical\n");
  check_n2_unaffected(J);

  printf("\n[9] the hover chain\n");
  check_hover_chain(&p);

  printf("\n[10] the residual is fed in the AERO frame\n");
  check_aero_frame_residual(&p);

  printf("\n%d checks run\n", g_checks);
  printf("RESULT: %s\n", g_fail ? "FAIL" : "all checks passed");
  return g_fail;
}
