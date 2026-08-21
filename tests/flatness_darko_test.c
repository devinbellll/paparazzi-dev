/*
 * Host harness: analytic property checks on the Darko flatness spine
 * (flatness_darko.c), compiled against the REAL pprz algebra rather than the
 * stubs the MIMO harness uses. Same fixture convention as
 * tests/flatness_quad_test.c.
 *
 * WHY PROPERTIES AND NOT A GOLDEN TRACE
 * -------------------------------------
 * The sim repo is the source of truth for these numerics, but generating a
 * trace from it needs MATLAB, which this environment does not have and will
 * not get. The quad spine hit the same wall and shipped analytic checks; this
 * does the same. Each check is a case where the correct answer is known in
 * closed form, chosen so the failure modes THIS transcription can actually
 * have are the ones that get caught.
 *
 * What each check discriminates:
 *   hover thrust     T(1-cDT) == m*g. The raw T is 1.9 % ABOVE weight because
 *                    propwash drag eats cDT of it. A check against m*g alone
 *                    would look 1.9 % wrong and invite a "fix".
 *   hover frame      theta == 0 in THIS tree's tailsitter hover frame, and
 *                    +pi/2 in the paper/sim aero frame. The single most likely
 *                    place for the port to go quietly wrong, and the one the
 *                    FlightGear render cannot see.
 *   cLT unreachable  0 vs 1000 must differ by EXACTLY zero. Guards the reason
 *                    the constant is 0 and not NaN.
 *   v = 0 benign     no NaN, no division; reduces to hover.
 *   quat round trip  quat2dcm(q) == R_b_i, catches a transposed rebuild.
 *   cmu sign IS READ flipping the field must invert the roll moment. Written
 *                    because in the sim repo this field was measured, stored,
 *                    and then read by no code for weeks.
 *   sign coverage    ASYMMETRIC, w1 != w2, banking. A residual check at a
 *                    symmetric trim cannot see the roll sign at all -- which is
 *                    exactly how the cmu sign survived.
 *   flap effect      allocator's dM/ddelta against the wrench model's, by
 *                    finite difference.
 *   round trip       allocate (T, m_c), push u back through the wrench, get
 *                    m_c. The strongest check available without traces, and it
 *                    exercises the sequential order directly.
 */

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "firmwares/rotorcraft/stabilization/flatness_darko.h"

static int g_fail = 0;

static void ck(const char *name, int ok, const char *detail)
{
  printf("  %-46s %s", name, ok ? "PASS\n" : "FAIL");
  if (!ok) { printf("   <- %s\n", detail); g_fail = 1; }
}

static void ck_near(const char *name, double got, double want, double tol)
{
  char d[160];
  int ok = fabs(got - want) <= tol;
  snprintf(d, sizeof d, "got %.9g, want %.9g (tol %g)", got, want, tol);
  ck(name, ok, d);
  if (ok) { printf("      measured: %.9g (want %.9g, tol %g)\n", got, want, tol); }
}

static void ck_exact(const char *name, double got, double want)
{
  char d[160];
  int ok = (got == want);
  snprintf(d, sizeof d, "got %.17g, want %.17g", got, want);
  ck(name, ok, d);
}

static struct FloatVect3 v3(float x, float y, float z)
{
  struct FloatVect3 v = { x, y, z }; return v;
}

/** Body -> NED, i.e. the transpose of the NED -> body rmat the transform
 * returns. Functions 2 and 3 want the AERO one. */
static struct FloatRMat i_b_of_b_i(const struct FloatRMat *R_b_i)
{
  struct FloatRMat R;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) { RMAT_ELMT(R, i, j) = RMAT_ELMT(*R_b_i, j, i); }
  }
  return R;
}

/* ======================================================================== */
/* [1] force transform                                                      */
/* ======================================================================== */

static void check_hover(struct FlatnessDarkoParams *p)
{
  struct FlatnessDarkoForce o;
  struct FloatVect3 fi = v3(0.f, 0.f, -p->m * p->g);
  struct FloatVect3 v  = v3(0.f, 0.f, 0.f);
  struct FloatVect3 by = v3(0.f, 1.f, 0.f);

  int ok = flatness_darko_force_transform(p, &fi, 0.f, &v, 0.f, &by, &o);
  ck("hover: transform valid", ok && o.status == FLATNESS_DARKO_OK, "errored");

  /* T is POSITIVE collective thrust, NOT the quad's signed body-z force. */
  ck("hover: T is positive", o.T > 0.f, "T <= 0");

  /* The raw T sits 1.9 % ABOVE weight ON PURPOSE: propwash drag eats cDT of
   * the rotor thrust, so the NET axial force is T(1-cDT). Checking T against
   * m*g directly would report a 1.9 % error that is not there. */
  ck_near("hover: raw T == 4.9196 N (not m*g)", o.T, 4.9196, 2e-4);
  ck_near("hover: T*(1-cDT) == m*g", o.T * (1.f - p->cDT), p->m * p->g, 1e-5);

  ck_near("hover: phi == 0", o.euler.phi, 0.0, 1e-6);
  ck_near("hover: psi == 0", o.euler.psi, 0.0, 1e-9);
}

/* THE frame check. In the sim repo Darko hover is theta = 90 deg; in THIS tree
 * the tailsitter hover frame makes theta = 0 hover. Both are asserted, so the
 * offset cannot silently drift and neither convention can be "corrected" into
 * the other without this failing. */
static void check_hover_frame(struct FlatnessDarkoParams *p)
{
  struct FlatnessDarkoForce o;
  struct FloatVect3 fi = v3(0.f, 0.f, -p->m * p->g);
  struct FloatVect3 v  = v3(0.f, 0.f, 0.f);
  struct FloatVect3 by = v3(0.f, 1.f, 0.f);
  flatness_darko_force_transform(p, &fi, 0.f, &v, 0.f, &by, &o);

  ck_near("frame: AERO theta == +90 deg (paper/sim)",
          o.theta_aero * 180.0 / M_PI, 90.0, 1e-4);
  ck_near("frame: HOVER theta == 0 deg (this tree)",
          o.euler.theta * 180.0 / M_PI, 0.0, 1e-4);
  ck_near("frame: hover quat is identity",
          fabs(o.quat.qi), 1.0, 1e-6);

  /* The AERO frame puts thrust along body +x: R_b_i maps world DOWN onto -x_b.
   * The HOVER frame puts it along body -z: R_h_i maps world down onto +z_h. */
  ck_near("frame: AERO R maps world-down to -x_b",
          RMAT_ELMT(o.rmat_aero, 0, 2), -1.0, 1e-6);
  ck_near("frame: HOVER R maps world-down to +z_h",
          RMAT_ELMT(o.rmat, 2, 2), 1.0, 1e-6);

  /* And the two differ by exactly the constant y rotation, at any state. */
  struct FloatVect3 fi2 = v3(1.3f, -0.8f, -5.6f);
  struct FloatVect3 v2  = v3(4.0f, 1.0f, -0.5f);
  flatness_darko_force_transform(p, &fi2, 0.4f, &v2, 0.05f, &by, &o);
  ck_near("frame: offset is exactly -pi/2 at a maneuvering state",
          o.theta_aero - o.euler.theta, M_PI / 2.0, 1e-6);
}

/* psi_ref must survive the inversion exactly -- the ZXY property. */
static void check_yaw_passthrough(struct FlatnessDarkoParams *p)
{
  struct FlatnessDarkoForce o;
  const float psi[] = { 0.f, 0.7f, -2.5f, 3.0f };
  struct FloatVect3 fi = v3(2.0f, -1.5f, -p->m * p->g);
  struct FloatVect3 v  = v3(3.0f, 0.5f, -1.0f);
  struct FloatVect3 by = v3(0.f, 1.f, 0.f);
  double worst = 0.0;
  int ok = 1;
  for (unsigned i = 0; i < sizeof psi / sizeof psi[0]; i++) {
    ok &= flatness_darko_force_transform(p, &fi, psi[i], &v, 0.02f, &by, &o);
    double e = fabs(o.euler.psi - psi[i]);
    if (e > worst) { worst = e; }
  }
  ck("yaw: transform valid at every psi", ok, "errored");
  ck_near("yaw: psi_ref preserved exactly", worst, 0.0, 1e-9);
}

/* cLT only ever multiplies sin(alphabar) = 0. Evaluating at 0 and 1000 must
 * differ by EXACTLY zero -- that is the licence to store 0 rather than NaN. */
static void check_clt_unreachable(struct FlatnessDarkoParams *p)
{
  struct FlatnessDarkoParams p1000 = *p;
  p1000.cLT = 1000.f;

  struct FlatnessDarkoForce a, b;
  struct FloatVect3 fi = v3(2.4f, -1.1f, -6.3f);
  struct FloatVect3 v  = v3(8.0f, 1.5f, -2.0f);
  struct FloatVect3 by = v3(0.f, 1.f, 0.f);

  flatness_darko_force_transform(p,      &fi, 0.6f, &v, 0.08f, &by, &a);
  flatness_darko_force_transform(&p1000, &fi, 0.6f, &v, 0.08f, &by, &b);

  ck_exact("cLT: T bit-identical",        a.T, b.T);
  ck_exact("cLT: phi bit-identical",      a.euler.phi, b.euler.phi);
  ck_exact("cLT: thetabar bit-identical", a.thetabar, b.thetabar);
  ck_exact("cLT: theta bit-identical",    a.euler.theta, b.euler.theta);
  ck_exact("cLT: quat qi bit-identical",  a.quat.qi, b.quat.qi);
  ck_exact("cLT: quat qx bit-identical",  a.quat.qx, b.quat.qx);
  ck_exact("cLT: quat qy bit-identical",  a.quat.qy, b.quat.qy);
  ck_exact("cLT: quat qz bit-identical",  a.quat.qz, b.quat.qz);

  /* And the NaN trap the constant exists to avoid. */
  ck("cLT: stored value is finite, not NaN", isfinite(p->cLT), "cLT is NaN/inf");
}

/* v = 0 is an ORDINARY point: every velocity term carries a factor of ||v||.
 * There is no airspeed singularity in this spine -- eqs (33) and (76) divide
 * by ||v|| and live in branches this spine does not implement. */
static void check_zero_velocity(struct FlatnessDarkoParams *p)
{
  struct FlatnessDarkoForce o;
  struct FloatVect3 fi = v3(0.8f, -0.4f, -5.0f);
  struct FloatVect3 v  = v3(0.f, 0.f, 0.f);
  struct FloatVect3 by = v3(0.f, 1.f, 0.f);

  int ok = flatness_darko_force_transform(p, &fi, 0.3f, &v, 0.1f, &by, &o);
  int finite = isfinite(o.T) && isfinite(o.euler.phi) && isfinite(o.euler.theta)
               && isfinite(o.quat.qi) && isfinite(o.quat.qx)
               && isfinite(o.quat.qy) && isfinite(o.quat.qz);
  ck("v=0: transform valid", ok, "errored");
  ck("v=0: no NaN anywhere in the output", finite, "NaN or inf present");

  /* And at zero force demand in the plane, it is exactly hover. */
  struct FloatVect3 fh = v3(0.f, 0.f, -p->m * p->g);
  flatness_darko_force_transform(p, &fh, 0.f, &v, 0.f, &by, &o);
  ck_near("v=0: reduces to hover", o.euler.theta, 0.0, 1e-6);
}

static void check_quat_round_trip(struct FlatnessDarkoParams *p)
{
  struct FlatnessDarkoForce o;
  struct FloatVect3 fi = v3(1.9f, -2.4f, -6.9f);
  struct FloatVect3 v  = v3(6.0f, -1.0f, 0.7f);
  struct FloatVect3 by = v3(0.f, 1.f, 0.f);
  flatness_darko_force_transform(p, &fi, -0.9f, &v, 0.06f, &by, &o);

  struct FloatRMat Ra, Rh;
  float_rmat_of_quat(&Ra, &o.quat_aero);
  float_rmat_of_quat(&Rh, &o.quat);

  double wa = 0.0, wh = 0.0;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      double ea = fabs(RMAT_ELMT(Ra, i, j) - RMAT_ELMT(o.rmat_aero, i, j));
      double eh = fabs(RMAT_ELMT(Rh, i, j) - RMAT_ELMT(o.rmat, i, j));
      if (ea > wa) { wa = ea; }
      if (eh > wh) { wh = eh; }
    }
  }
  /* Single precision throughout, so ~1e-15 is not available -- ~1e-6 is. */
  ck_near("quat: quat2dcm(q_aero) == R_b_i", wa, 0.0, 1e-6);
  ck_near("quat: quat2dcm(q_hover) == R_h_i", wh, 0.0, 1e-6);
}

/* The roll of eq (17) is genuinely undetermined when both betas vanish. There
 * is no arbitrary value that beats saying so. */
static void check_roll_undetermined(struct FlatnessDarkoParams *p)
{
  struct FlatnessDarkoForce o;
  struct FloatVect3 fi = v3(3.0f, 0.f, 0.f);   /* beta_x = beta_z = 0 at psi=0 */
  struct FloatVect3 v  = v3(0.f, 0.f, 0.f);
  struct FloatVect3 by = v3(0.f, 1.f, 0.f);
  int ok = flatness_darko_force_transform(p, &fi, 0.f, &v, 0.f, &by, &o);
  ck("roll: errors out when undetermined",
     !ok && o.status == FLATNESS_DARKO_ROLL_UNDETERMINED, "did not error");
}

/* ======================================================================== */
/* [2] wrench                                                               */
/* ======================================================================== */

/* THE cmu-sign check. The field is -1 on Darko and it must actually be READ.
 * Measured on the plant at (700, -500): -0.063360 N*m, against the paper
 * form's +0.063360. */
static void check_cmu_sign_is_read(struct FlatnessDarkoParams *p)
{
  struct FlatnessDarkoParams paper = *p;
  paper.cmu_sign_vs_paper = +1.0f;   /* the paper's sign, i.e. WRONG on Darko */

  const float w[2] = { 700.f, -500.f };   /* ASYMMETRIC: w1 != w2 */
  const float d[2] = { 0.f, 0.f };
  struct FloatVect3 v = v3(0.f, 0.f, 0.f);

  struct FloatRMat R; float_rmat_identity(&R);
  struct FloatVect3 f, m_darko, m_paper;

  flatness_darko_wrench(p,      w, d, &v, &R, &f, &m_darko);
  flatness_darko_wrench(&paper, w, d, &v, &R, &f, &m_paper);

  ck_near("cmu: plant value at (700,-500) is -0.063360",
          m_darko.x, -0.063360, 1e-6);
  ck_near("cmu: paper sign would give +0.063360",
          m_paper.x, +0.063360, 1e-6);
  ck("cmu: flipping the field CHANGES the moment (field is read)",
     fabsf(m_darko.x - m_paper.x) > 1e-6f, "field is not read anywhere");

  /* And the reason it survived so long: at a SYMMETRIC point it is invisible.
   * This is asserted, not assumed -- it documents why the check above must be
   * asymmetric. */
  const float ws[2] = { 600.f, -600.f };
  struct FloatVect3 ms_d, ms_p;
  flatness_darko_wrench(p,      ws, d, &v, &R, &f, &ms_d);
  flatness_darko_wrench(&paper, ws, d, &v, &R, &f, &ms_p);
  ck_near("cmu: symmetric trim cannot see the sign (by design)",
          fabs(ms_d.x - ms_p.x), 0.0, 1e-12);
}

/* The w2 sign lives at the ACTUATOR interface, and the wrench model cannot
 * police it: every rotor term is w_i^2, so the wrench is EVEN in w. Asserted
 * here so the next reader does not go looking for a residual check that can
 * catch a positive w2 -- there isn't one. The allocator's emission (see
 * "alloc: w2 is emitted NEGATIVE") is the only guard, and feeding both rotors
 * positive to the plant leaves a spurious roll moment: 44.6 rad/s^2 peak
 * measured, against 0.38 signed. */
static void check_wrench_is_even_in_w(struct FlatnessDarkoParams *p)
{
  const float d[2] = { 0.05f, -0.03f };
  struct FloatVect3 v = v3(5.f, 1.f, -0.5f);
  struct FloatRMat R; float_rmat_identity(&R);
  const float w_ok[2]  = { 700.f, -500.f };
  const float w_bad[2] = { 700.f,  500.f };
  struct FloatVect3 f1, m1, f2, m2;
  flatness_darko_wrench(p, w_ok,  d, &v, &R, &f1, &m1);
  flatness_darko_wrench(p, w_bad, d, &v, &R, &f2, &m2);
  ck("wrench: EVEN in w -- cannot detect a positive w2",
     m1.x == m2.x && m1.y == m2.y && m1.z == m2.z && f1.z == f2.z,
     "wrench is odd in w somewhere, revisit the sign story");
}

/* No lateral force, by construction, at any state. */
static void check_no_lateral_force(struct FlatnessDarkoParams *p)
{
  const float w[2] = { 700.f, -520.f };
  const float d[2] = { 0.12f, -0.07f };
  struct FloatVect3 v = v3(9.0f, 2.0f, -1.5f);
  struct FloatRMat R; float_rmat_identity(&R);
  struct FloatVect3 f, m;
  flatness_darko_wrench(p, w, d, &v, &R, &f, &m);
  /* R is identity here, so NED y IS body y. */
  ck_near("wrench: no lateral force component", f.y, 0.0, 1e-9);
}

/* cmuT is a STRUCTURAL zero: props at z = 0, purely axial thrust, no pitch
 * moment from the x offset. Measured at w = 400, 600, 800. */
static void check_cmut_structural_zero(struct FlatnessDarkoParams *p)
{
  const float d[2] = { 0.f, 0.f };
  struct FloatVect3 v = v3(0.f, 0.f, 0.f);
  struct FloatRMat R; float_rmat_identity(&R);
  struct FloatVect3 f, m;
  const float ws[3] = { 400.f, 600.f, 800.f };
  double worst = 0.0;
  for (int i = 0; i < 3; i++) {
    const float w[2] = { ws[i], -ws[i] };
    flatness_darko_wrench(p, w, d, &v, &R, &f, &m);
    if (fabs(m.y) > worst) { worst = fabs(m.y); }
  }
  ck_near("wrench: cmuT gives no pitch moment at 400/600/800", worst, 0.0, 1e-12);
}

/* ======================================================================== */
/* [3] allocation                                                           */
/* ======================================================================== */

/* THE strongest check available without traces. It exercises the sequential
 * order directly: yaw first onto differential thrust, flaps absorbing only the
 * remainder. A reordering into one linear solve does not close this. */
static void check_round_trip(struct FlatnessDarkoParams *p)
{
  struct FlatnessDarkoForce o;
  struct FloatVect3 fi = v3(1.6f, -1.2f, -5.9f);
  struct FloatVect3 v  = v3(7.0f, 1.2f, -0.9f);
  struct FloatVect3 by = v3(0.f, 1.f, 0.f);
  flatness_darko_force_transform(p, &fi, 0.35f, &v, 0.04f, &by, &o);

  struct FloatRMat R_i_b = i_b_of_b_i(&o.rmat_aero);

  /* ASYMMETRIC: a nonzero z moment forces w1 != w2, which is the only way the
   * roll sign is observable at all. */
  struct FloatVect3 m_c = v3(0.06f, -0.02f, 0.03f);

  float u[4]; int8_t sat[4];
  int ok = flatness_darko_allocate(p, o.T, &m_c, &v, &R_i_b, u, sat);
  ck("alloc: succeeds on a banking state", ok, "errored");
  ck("alloc: w2 is emitted NEGATIVE", u[1] < 0.f, "w2 came back positive");
  ck("alloc: w1 != w2 (asymmetric, roll sign observable)",
     fabsf(u[0] - fabsf(u[1])) > 1.f, "symmetric, check is blind");

  const float w[2] = { u[0], u[1] };
  const float d[2] = { u[2], u[3] };
  struct FloatVect3 f_back, m_back;
  flatness_darko_wrench(p, w, d, &v, &R_i_b, &f_back, &m_back);

  ck_near("round trip: m_c.x recovered", m_back.x, m_c.x, 1e-6);
  ck_near("round trip: m_c.y recovered", m_back.y, m_c.y, 1e-6);
  ck_near("round trip: m_c.z recovered", m_back.z, m_c.z, 1e-6);
}

/* The full chain at hover: transform -> allocate -> wrench must give back the
 * commanded inertial force, including the cDT bookkeeping. */
static void check_hover_chain(struct FlatnessDarkoParams *p)
{
  struct FlatnessDarkoForce o;
  struct FloatVect3 fi = v3(0.f, 0.f, -p->m * p->g);
  struct FloatVect3 v  = v3(0.f, 0.f, 0.f);
  struct FloatVect3 by = v3(0.f, 1.f, 0.f);
  flatness_darko_force_transform(p, &fi, 0.f, &v, 0.f, &by, &o);

  struct FloatRMat R_i_b = i_b_of_b_i(&o.rmat_aero);
  struct FloatVect3 m_c = v3(0.f, 0.f, 0.f);
  float u[4]; int8_t sat[4];
  flatness_darko_allocate(p, o.T, &m_c, &v, &R_i_b, u, sat);

  const float w[2] = { u[0], u[1] };
  const float d[2] = { u[2], u[3] };
  struct FloatVect3 f_back, m_back;
  flatness_darko_wrench(p, w, d, &v, &R_i_b, &f_back, &m_back);

  ck_near("hover chain: f_ned.z == -m*g", f_back.z, -p->m * p->g, 1e-5);
  ck_near("hover chain: f_ned.x == 0", f_back.x, 0.0, 1e-5);
  ck_near("hover chain: rotor speed in band",
          (fabs(u[0]) >= p->w_min && fabs(u[0]) <= p->w_max) ? 1.0 : 0.0, 1.0, 0.0);
  printf("      hover rotor speed |w| = %.4f rad/s, flaps %.3e / %.3e rad\n",
         fabs(u[0]), u[2], u[3]);
}

/* Finite-difference the wrench model's dM/ddelta and compare it against the
 * effectiveness the allocator inverts (eqs (39)(40)). */
static void check_flap_effectiveness(struct FlatnessDarkoParams *p)
{
  struct FloatVect3 v  = v3(6.0f, 1.0f, -0.5f);
  struct FloatRMat R; float_rmat_identity(&R);
  const float w[2] = { 640.f, -560.f };

  const float T1 = p->cT * w[0] * w[0];
  const float T2 = p->cT * w[1] * w[1];
  const float nv = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
  const float vax = v.x;   /* R identity, alpha0 = 0 */

  const float nu1 = -p->cLT_d * cosf(p->alphabar) * T1 - p->cLV_d * nv * vax;
  const float nu2 = -p->cLT_d * cosf(p->alphabar) * T2 - p->cLV_d * nv * vax;
  const float c0 = cosf(p->alpha0);
  /* The allocator's A, eq (39). */
  const float A[2][2] = { { -p->ldy * c0 * nu1,  p->ldy * c0 * nu2 },
                          {  p->ldx * nu1,       p->ldx * nu2      } };

  const float h = 1e-4f;
  double worst_rel = 0.0;
  for (int j = 0; j < 2; j++) {
    float dp[2] = { 0.f, 0.f }, dm[2] = { 0.f, 0.f };
    dp[j] =  h; dm[j] = -h;
    struct FloatVect3 f1, m1, f2, m2;
    flatness_darko_wrench(p, w, dp, &v, &R, &f1, &m1);
    flatness_darko_wrench(p, w, dm, &v, &R, &f2, &m2);
    const double fd[2] = { (m1.x - m2.x) / (2.0 * h),
                           (m1.y - m2.y) / (2.0 * h) };
    for (int i = 0; i < 2; i++) {
      double rel = fabs(fd[i] - A[i][j]) / fmax(fabs(A[i][j]), 1e-9);
      if (rel > worst_rel) { worst_rel = rel; }
    }
  }
  ck_near("flap: dM/ddelta matches the allocator's A (worst rel)",
          worst_rel, 0.0, 5e-3);
}

/* nu vanishes only if BOTH thrust and airspeed are zero. Then the flaps have
 * no flow and genuinely produce no moment: report it, do not divide. */
static void check_no_flap_authority(struct FlatnessDarkoParams *p)
{
  struct FloatVect3 v = v3(0.f, 0.f, 0.f);
  struct FloatVect3 m_c = v3(0.01f, 0.01f, 0.f);
  struct FloatRMat R; float_rmat_identity(&R);
  float u[4]; int8_t sat[4];
  int ok = flatness_darko_allocate(p, 0.f, &m_c, &v, &R, u, sat);
  ck("alloc: reports no flap authority at T=0, v=0", !ok, "did not report");
  ck("alloc: returns zero deflections, not NaN",
     u[2] == 0.f && u[3] == 0.f, "nonzero or NaN deflections");
  ck("alloc: flags both rotors low", sat[0] < 0 && sat[1] < 0, "no low flag");
}

/* A commanded rotor thrust below zero is unachievable on fixed-pitch props:
 * clamped at zero, and REPORTED. */
static void check_saturation_reported(struct FlatnessDarkoParams *p)
{
  struct FloatVect3 v = v3(5.f, 0.f, 0.f);
  struct FloatRMat R; float_rmat_identity(&R);
  float u[4]; int8_t sat[4];

  /* Huge yaw demand drives T2 negative. */
  struct FloatVect3 m_c = v3(0.f, 0.f, 5.0f);
  flatness_darko_allocate(p, 4.9f, &m_c, &v, &R, u, sat);
  ck("sat: negative rotor thrust clamps to w = 0", u[1] == 0.f, "not clamped");
  ck("sat: clamp is reported as SAT_LOW", sat[1] == FLATNESS_DARKO_SAT_LOW, "not flagged");

  /* Flap saturation is reported, NOT applied: the value must exceed d_max. */
  struct FloatVect3 m_big = v3(3.0f, 0.f, 0.f);
  flatness_darko_allocate(p, 4.9f, &m_big, &v, &R, u, sat);
  ck("sat: flap over d_max is REPORTED not applied",
     (fabsf(u[2]) > p->d_max) && sat[2] != FLATNESS_DARKO_SAT_NONE,
     "value was clipped or not flagged");
}

/* ======================================================================== */
/* [4] composition with the REUSED quad attitude error                      */
/* ======================================================================== */

static void check_composition(struct FlatnessDarkoParams *p)
{
  struct FlatnessDarkoForce o;
  struct FloatVect3 fi = v3(1.8f, -2.2f, -5.4f);
  struct FloatVect3 v  = v3(4.0f, 0.5f, -1.0f);
  struct FloatVect3 by = v3(0.f, 1.f, 0.f);
  flatness_darko_force_transform(p, &fi, -0.9f, &v, 0.03f, &by, &o);

  struct FloatVect3 z;
  flatness_quad_att_error(&o.quat, &o.quat, &z);
  ck_near("composition: commanded == measured gives zero error",
          sqrt(z.x * z.x + z.y * z.y + z.z * z.z), 0.0, 1e-6);
}

int main(void)
{
  struct FlatnessDarkoParams p;
  flatness_darko_params_init(&p);

  printf("Darko flatness spine -- analytic property checks\n\n");
  printf("[1] force transform\n");
  check_hover(&p);
  check_hover_frame(&p);
  check_yaw_passthrough(&p);
  check_clt_unreachable(&p);
  check_zero_velocity(&p);
  check_quat_round_trip(&p);
  check_roll_undetermined(&p);

  printf("\n[2] applied wrench\n");
  check_cmu_sign_is_read(&p);
  check_wrench_is_even_in_w(&p);
  check_no_lateral_force(&p);
  check_cmut_structural_zero(&p);

  printf("\n[3] allocation\n");
  check_round_trip(&p);
  check_hover_chain(&p);
  check_flap_effectiveness(&p);
  check_no_flap_authority(&p);
  check_saturation_reported(&p);

  printf("\n[4] composition\n");
  check_composition(&p);

  printf("\nRESULT: %s\n", g_fail ? "FAIL" : "all checks passed");
  return g_fail;
}
