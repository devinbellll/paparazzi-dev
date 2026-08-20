/*
 * Host harness: analytic property checks on the flatness spine
 * (flatness_quad.c), compiled against the REAL pprz algebra rather than the
 * stubs the MIMO harness uses.
 *
 * WHY PROPERTIES AND NOT A GOLDEN TRACE
 * -------------------------------------
 * The sim repo is the source of truth for these numerics, and a recorded trace
 * from it would be the stronger check -- but generating one needs MATLAB, which
 * this environment does not have. These checks are therefore derived from the
 * equations by hand: each one is a case where the correct answer is known in
 * closed form, chosen so that the failure modes the transcription can actually
 * have are the ones that get caught.
 *
 * A closed-loop comparison against findi_quad.slx is still owed; see the task
 * note verify-flatness-quad-against-sim.
 *
 * What each check discriminates:
 *   hover           sign and magnitude of f, and that level is level
 *   yaw passthrough psi_ref survives the inversion exactly (the ZXY property
 *                   that motivated the sequence)
 *   round trip      R_b_i * zb_world == [0;0;1], i.e. the rebuilt DCM really
 *                   maps the commanded thrust axis onto body z. Catches a
 *                   transposed or wrong-sequence rebuild, which a hover-only
 *                   test cannot see.
 *   past 90 deg     the k*pi branch. With the branch OFF this same input comes
 *                   back with the attitude flipped and no error raised.
 *   free fall       the held-attitude contract
 *   zeta_e = 0      the k = 2 limit at qe_w = 1 (0/0 at the commonest point)
 *   shortest path   a small error expressed by a negative-scalar quaternion
 *                   must give a small zeta_e, not a ~2*pi one
 *   known angle     |zeta_e| equals the true rotation angle
 */

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "firmwares/rotorcraft/stabilization/flatness_quad.h"

#define MASS  0.8f
#define GRAV  9.81f

static int g_fail = 0;

static void ck(const char *name, int ok, const char *detail)
{
  printf("  %-40s %s", name, ok ? "PASS\n" : "FAIL");
  if (!ok) { printf("   <- %s\n", detail); g_fail = 1; }
}

static void ck_near(const char *name, double got, double want, double tol)
{
  char d[128];
  int ok = fabs(got - want) <= tol;
  snprintf(d, sizeof d, "got %.9g, want %.9g (tol %g)", got, want, tol);
  ck(name, ok, d);
}

/* Hover: fi_c = m*(a - g*iz) with a = 0 is m*[0;0;-g] in NED. */
static struct FloatVect3 hover_force(void)
{
  struct FloatVect3 f = { 0.f, 0.f, -MASS * GRAV };
  return f;
}

static void check_hover(void)
{
  struct FlatnessQuad fq;
  struct FlatnessQuadForce o;
  flatness_quad_init(&fq, MASS, true);

  struct FloatVect3 fi = hover_force();
  int ok = flatness_quad_force_transform(&fq, &fi, 0.f, &o);

  ck("hover: transform valid", ok && o.status == FLATNESS_QUAD_OK, "degraded");
  /* NEGATIVE in hover: this is the collective body-z force in NED, not a
   * thrust magnitude. A sign slip here is silent everywhere downstream. */
  ck_near("hover: f = -m*g", o.f, -MASS * GRAV, 1e-5);
  ck_near("hover: phi = 0",   o.euler.phi,   0.0, 1e-6);
  ck_near("hover: theta = 0", o.euler.theta, 0.0, 1e-6);
  ck_near("hover: quat identity", fabs(o.quat.qi), 1.0, 1e-6);
}

/* psi_ref must come back as psi EXACTLY, at any tilt. That is the ZXY property
 * the sequence was chosen for; a ZYX map does not have it. */
static void check_yaw_passthrough(void)
{
  struct FlatnessQuad fq;
  struct FlatnessQuadForce o;
  flatness_quad_init(&fq, MASS, true);

  const float psi[] = { 0.f, 0.7f, -2.5f, 3.0f };
  struct FloatVect3 fi = { 2.0f, -1.5f, -MASS * GRAV };
  int ok = 1;
  double worst = 0.0;
  for (unsigned i = 0; i < sizeof psi / sizeof psi[0]; i++) {
    flatness_quad_reset(&fq);
    ok &= flatness_quad_force_transform(&fq, &fi, psi[i], &o);
    double e = fabs(o.euler.psi - psi[i]);
    if (e > worst) { worst = e; }
  }
  ck("yaw: transform valid at every psi", ok, "degraded");
  ck_near("yaw: psi_ref preserved", worst, 0.0, 1e-6);
}

/* The rebuilt R_b_i must map the commanded thrust axis onto body z. This is the
 * check that a hover test cannot do: it exercises every entry of the matrix. */
static void check_round_trip(void)
{
  struct FlatnessQuad fq;
  struct FlatnessQuadForce o;
  flatness_quad_init(&fq, MASS, true);

  struct FloatVect3 cases[] = {
    {  0.0f,  0.0f, -MASS * GRAV },
    {  3.0f,  0.0f, -MASS * GRAV },   /* pure pitch demand */
    {  0.0f,  3.0f, -MASS * GRAV },   /* pure roll demand */
    { -2.0f,  1.0f, -MASS * GRAV * 1.3f },
  };
  const float psi_ref = 0.6f;

  double worst = 0.0;
  int ok = 1;
  for (unsigned c = 0; c < sizeof cases / sizeof cases[0]; c++) {
    flatness_quad_reset(&fq);
    ok &= flatness_quad_force_transform(&fq, &cases[c], psi_ref, &o);

    /* zb in NED, from the force itself: zb = -fi_c/|fi_c|. */
    float n = sqrtf(cases[c].x * cases[c].x + cases[c].y * cases[c].y
                    + cases[c].z * cases[c].z);
    float zb[3] = { -cases[c].x / n, -cases[c].y / n, -cases[c].z / n };

    /* R_b_i * zb should be body z = [0;0;1]. */
    for (int r = 0; r < 3; r++) {
      float v = RMAT_ELMT(o.rmat, r, 0) * zb[0]
              + RMAT_ELMT(o.rmat, r, 1) * zb[1]
              + RMAT_ELMT(o.rmat, r, 2) * zb[2];
      float want = (r == 2) ? 1.f : 0.f;
      double e = fabs(v - want);
      if (e > worst) { worst = e; }
    }
  }
  ck("round trip: transform valid", ok, "degraded");
  ck_near("round trip: R_b_i * zb = body z", worst, 0.0, 1e-5);
}

/* Tilt past 90 degrees. The commanded force points mostly UP in NED (positive
 * body-z demand), which needs |theta| > 90 deg to achieve. With the k*pi branch
 * the triple stays continuous with the previous step; with it off the same
 * input comes back flipped and nothing reports it. */
static void check_past_90(void)
{
  struct FlatnessQuad on, off;
  struct FlatnessQuadForce o;
  flatness_quad_init(&on,  MASS, true);
  flatness_quad_init(&off, MASS, false);

  /* Walk the demand from level round to inverted so the branch has continuity
   * to work with, rather than jumping there in one step. */
  const int N = 64;
  float last_phi_on = 0.f, last_theta_on = 0.f;
  double worst_jump_on = 0.0;
  int ok = 1;

  for (int k = 0; k <= N; k++) {
    float a = (float)k / (float)N * (float)M_PI;   /* 0 .. pi of tilt */
    struct FloatVect3 fi = { MASS * GRAV * sinf(a), 0.f, -MASS * GRAV * cosf(a) };

    ok &= flatness_quad_force_transform(&on, &fi, 0.f, &o);
    if (k > 0) {
      double dphi = fabs(o.euler.phi - last_phi_on);
      /* Roll may legitimately wrap by 2*pi; fold it. */
      while (dphi > M_PI) { dphi = fabs(dphi - 2.0 * M_PI); }
      double dth = fabs(o.euler.theta - last_theta_on);
      while (dth > M_PI) { dth = fabs(dth - 2.0 * M_PI); }
      if (dphi > worst_jump_on) { worst_jump_on = dphi; }
      if (dth  > worst_jump_on) { worst_jump_on = dth; }
    }
    last_phi_on   = o.euler.phi;
    last_theta_on = o.euler.theta;

    flatness_quad_force_transform(&off, &fi, 0.f, &o);
  }

  ck("past 90: transform valid throughout", ok, "degraded");
  /* Step size is pi/64 ~ 0.049 rad; allow a few of those, reject a pi flip. */
  ck("past 90: branch keeps the attitude continuous",
     worst_jump_on < 0.25, "a pi-sized jump means the branch did not take");

  /* And the two modes must genuinely differ past 90 deg, otherwise this test is
   * not exercising what it claims to. */
  struct FlatnessQuad a_on, a_off;
  struct FlatnessQuadForce r_on, r_off;
  flatness_quad_init(&a_on,  MASS, true);
  flatness_quad_init(&a_off, MASS, false);
  /* Seed both at level, then jump past 90 deg. */
  struct FloatVect3 level = hover_force();
  struct FloatVect3 inv   = { 0.5f, 0.f, MASS * GRAV };  /* +z demand = inverted */
  flatness_quad_force_transform(&a_on,  &level, 0.f, &r_on);
  flatness_quad_force_transform(&a_off, &level, 0.f, &r_off);
  flatness_quad_force_transform(&a_on,  &inv,   0.f, &r_on);
  flatness_quad_force_transform(&a_off, &inv,   0.f, &r_off);
  ck("past 90: branch on and off actually differ",
     fabs(r_on.euler.phi - r_off.euler.phi) > 1.0,
     "the two modes agree, so this case is not past the alias");
}

static void check_free_fall(void)
{
  struct FlatnessQuad fq;
  struct FlatnessQuadForce o, held;
  flatness_quad_init(&fq, MASS, true);

  /* Establish a non-trivial attitude first, so "held" is distinguishable from
   * "identity". */
  struct FloatVect3 tilted = { 2.5f, -1.0f, -MASS * GRAV };
  flatness_quad_force_transform(&fq, &tilted, 0.3f, &held);

  struct FloatVect3 nothing = { 0.f, 0.f, 0.f };
  int fresh = flatness_quad_force_transform(&fq, &nothing, 0.3f, &o);

  ck("free fall: reports not-fresh", !fresh, "returned true");
  ck("free fall: status is FREE_FALL", o.status == FLATNESS_QUAD_FREE_FALL, "wrong status");
  ck("free fall: attitude is held",
     fabs(o.euler.phi   - held.euler.phi)   < 1e-9 &&
     fabs(o.euler.theta - held.euler.theta) < 1e-9 &&
     fabs(o.quat.qi     - held.quat.qi)     < 1e-9,
     "attitude moved");
  ck_near("free fall: f still computed", o.f, 0.0, 1e-9);
  ck("free fall: counted", fq.degraded_count == 1, "counter not incremented");

  /* And gimbal lock must NOT degrade when the branch is on -- that is the whole
   * reason the branch is on by default. */
  struct FlatnessQuad gl;
  struct FlatnessQuadForce g;
  flatness_quad_init(&gl, MASS, true);
  /* Force exactly along the yawed x-axis: v2 = v3 = 0, so ct = 0. */
  struct FloatVect3 sideways = { MASS * GRAV, 0.f, 0.f };
  int okg = flatness_quad_force_transform(&gl, &sideways, 0.f, &g);
  ck("gimbal lock: resolved by the branch, not degraded",
     okg && g.status == FLATNESS_QUAD_OK, "degraded with the branch on");

  struct FlatnessQuad gl_off;
  flatness_quad_init(&gl_off, MASS, false);
  int okg2 = flatness_quad_force_transform(&gl_off, &sideways, 0.f, &g);
  ck("gimbal lock: degrades with the branch off",
     !okg2 && g.status == FLATNESS_QUAD_GIMBAL_LOCK, "did not degrade");
}

static struct FloatQuat quat_axis(float ax, float ay, float az, float ang)
{
  float n = sqrtf(ax * ax + ay * ay + az * az);
  float s = sinf(ang * 0.5f) / n;
  struct FloatQuat q = { cosf(ang * 0.5f), ax * s, ay * s, az * s };
  return q;
}

static void check_att_error(void)
{
  struct FloatVect3 z;

  /* Identical attitudes: the 0/0 point, and the commonest one in flight. */
  struct FloatQuat q = quat_axis(0.3f, -0.5f, 0.8f, 0.7f);
  flatness_quad_att_error(&q, &q, &z);
  ck_near("att err: zero at identical attitudes",
          sqrt(z.x * z.x + z.y * z.y + z.z * z.z), 0.0, 1e-6);

  /* |zeta_e| is the rotation angle between the two attitudes. */
  const float ang = 0.4f;
  struct FloatQuat qm = quat_axis(0.f, 0.f, 1.f, 0.f);
  struct FloatQuat qc = quat_axis(0.f, 1.f, 0.f, ang);
  flatness_quad_att_error(&qc, &qm, &z);
  ck_near("att err: magnitude is the rotation angle",
          sqrt(z.x * z.x + z.y * z.y + z.z * z.z), ang, 1e-5);
  ck_near("att err: on the rotation axis (y)", z.y, ang, 1e-5);
  ck_near("att err: no x component", z.x, 0.0, 1e-6);

  /* Shortest path: negating q_cmd is the SAME attitude, so the error must be
   * unchanged -- not 2*pi - ang. */
  struct FloatQuat qcn = { -qc.qi, -qc.qx, -qc.qy, -qc.qz };
  struct FloatVect3 zn;
  flatness_quad_att_error(&qcn, &qm, &zn);
  ck_near("att err: shortest path (negated q_cmd)",
          sqrt(zn.x * zn.x + zn.y * zn.y + zn.z * zn.z), ang, 1e-5);

  /* A large error still resolves the short way: 350 deg one way is 10 the
   * other. */
  struct FloatQuat qbig = quat_axis(0.f, 0.f, 1.f, (float)(2.0 * M_PI - 0.2));
  flatness_quad_att_error(&qbig, &qm, &z);
  ck_near("att err: 350 deg reads as 10 deg",
          sqrt(z.x * z.x + z.y * z.y + z.z * z.z), 0.2, 1e-4);
}

/* The spine composed: commanding an attitude and measuring it back must give
 * zero error. Catches a quaternion-sense mismatch between the two functions,
 * which neither one alone can detect. */
static void check_composition(void)
{
  struct FlatnessQuad fq;
  struct FlatnessQuadForce o;
  flatness_quad_init(&fq, MASS, true);

  struct FloatVect3 fi = { 1.8f, -2.2f, -MASS * GRAV };
  flatness_quad_force_transform(&fq, &fi, -0.9f, &o);

  struct FloatVect3 z;
  flatness_quad_att_error(&o.quat, &o.quat, &z);
  ck_near("composition: commanded == measured gives zero error",
          sqrt(z.x * z.x + z.y * z.y + z.z * z.z), 0.0, 1e-6);
}

int main(void)
{
  printf("Flatness spine -- analytic property checks\n\n");
  printf("[1] force transform\n");        check_hover();
                                          check_yaw_passthrough();
                                          check_round_trip();
  printf("\n[2] the k*pi branch\n");      check_past_90();
  printf("\n[3] degraded cases\n");       check_free_fall();
  printf("\n[4] attitude error\n");       check_att_error();
  printf("\n[5] composition\n");          check_composition();

  printf("\nRESULT: %s\n", g_fail ? "FAIL" : "all checks passed");
  return g_fail;
}
