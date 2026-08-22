/*
 * Host harness: analytic property checks on the Darko FINDI controller law
 * (oneloop_findi_darko_law.h) and its composition with the stage-1 spine
 * (flatness_darko.c). Same fixture convention as tests/flatness_darko_test.c.
 *
 * WHY PROPERTIES AND NOT A GOLDEN TRACE
 * -------------------------------------
 * There is no MATLAB in this environment and there will not be one, so there
 * are no golden traces for the controller either. The checks below were
 * decided UP FRONT, as the bring-up ladder's rung 1, rather than discovered
 * mid-port: each is a case where the correct answer is known in closed form,
 * chosen so the failure modes THIS transcription can actually have are the
 * ones that get caught.
 *
 * And offline agreement is NOT evidence the loop flies. The HEOL MIMO port was
 * bit-exact against its traces and diverged the moment the loop engaged. These
 * checks buy one thing: that a divergence in SITL is a control problem and not
 * a transcription problem.
 *
 * What each check discriminates:
 *   frame map        aero<->hover round trip, and the SPECIFIC permutation.
 *                    A sign or an axis swap here rotates roll into yaw on a
 *                    tailsitter: a plausible-looking wrong answer, not a crash.
 *   rmat map         findi_darko_rmat_hover_to_aero == Ry(+pi/2) R_h_i, built
 *                    independently from the spine's own force transform, which
 *                    emits BOTH frames for exactly this cross-check.
 *   axis corresp.    a pure HOVER-ROLL demand must come out as differential
 *                    THRUST with the flaps near zero, and a pure HOVER-YAW
 *                    demand as differential FLAP with the rotors equal. This
 *                    is the check that the "yaw onto differential thrust" of
 *                    eq (37) was read in the right frame.
 *   tsPD             the single-PD form, its second-order reading
 *                    (wn = sqrt(k_xi), 2 zeta wn = k_om), and that Om_lpf and
 *                    not the raw rate is what it differences.
 *   increment        omdot_c == omdot_lpf must give m_c == m_lpf EXACTLY. The
 *                    defining property of an incremental law: no error, no
 *                    change to the command.
 *   actuator maps    |w| (w2 is emitted negative and an ESC takes no sign),
 *                    clamping, and the hover trim landing mid-range.
 *   elevon sign      the pprz mapping's mirror, checked against the SITL
 *                    plant's OWN measured elevon coefficients. Both plant axes
 *                    must agree under ONE sign choice; that is what makes the
 *                    mirror more than a guess.
 *   hover chain      transform -> tsPD -> increment -> allocate at a hover
 *                    trim closes: zero attitude error and a settled wrench
 *                    reproduce the hover actuator command.
 */

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "firmwares/rotorcraft/stabilization/flatness_darko.h"
#include "firmwares/rotorcraft/oneloop/oneloop_findi_darko_law.h"

static int g_fail = 0;

static void ck(const char *name, int ok, const char *detail)
{
  printf("  %-52s %s", name, ok ? "PASS\n" : "FAIL");
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

/** Body -> NED, i.e. the transpose of the NED -> body rmat. */
static struct FloatRMat transp(const struct FloatRMat *R)
{
  struct FloatRMat T;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) { RMAT_ELMT(T, i, j) = RMAT_ELMT(*R, j, i); }
  }
  return T;
}

/* ======================================================================== */
/* [1] frames                                                               */
/* ======================================================================== */

static void check_frame_map(void)
{
  struct FloatVect3 h = v3(1.f, 2.f, 3.f);
  struct FloatVect3 a, back;

  findi_darko_hover_to_aero(&h, &a);
  ck_exact("frame: hover(1,2,3) -> aero.x == -3", a.x, -3.0);
  ck_exact("frame: hover(1,2,3) -> aero.y == +2", a.y,  2.0);
  ck_exact("frame: hover(1,2,3) -> aero.z == +1", a.z,  1.0);

  findi_darko_aero_to_hover(&a, &back);
  ck("frame: aero->hover inverts hover->aero exactly",
     back.x == h.x && back.y == h.y && back.z == h.z, "round trip lost a component");

  /* Alias safety: the .c never calls these in place, but a future caller
   * might, and a naive implementation would corrupt itself. */
  struct FloatVect3 s = v3(1.f, 2.f, 3.f);
  findi_darko_hover_to_aero(&s, &s);
  ck("frame: in-place conversion is safe",
     s.x == -3.f && s.y == 2.f && s.z == 1.f, "in-place call corrupted the vector");
}

/** The rmat map, checked against the spine's OWN two frames. The force
 * transform emits rmat (hover) and rmat_aero for exactly this purpose, and it
 * builds them from two independent Ry() calls -- so agreement here is a real
 * cross-check and not a tautology. */
static void check_rmat_map(struct FlatnessDarkoParams *p)
{
  struct FlatnessDarkoForce o;
  struct FloatVect3 fi = v3(1.1f, -0.7f, -5.2f);
  struct FloatVect3 v  = v3(0.9f, 0.2f, -0.4f);
  struct FloatVect3 by = v3(0.f, 1.f, 0.f);
  flatness_darko_force_transform(p, &fi, 0.4f, &v, 0.02f, &by, &o);

  struct FloatRMat R_b_i;
  findi_darko_rmat_hover_to_aero(&o.rmat, &R_b_i);

  float worst = 0.f;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      const float d = fabsf(RMAT_ELMT(R_b_i, i, j) - RMAT_ELMT(o.rmat_aero, i, j));
      if (d > worst) { worst = d; }
    }
  }
  ck_near("rmat: hover->aero reproduces the spine's rmat_aero", worst, 0.0, 1e-6);
}

/** Hover attitude in the HOVER frame is theta = 0, and in the AERO frame the
 * SAME attitude is theta = +pi/2. The single most likely place for a
 * tailsitter port to go quietly wrong. */
static void check_hover_frame_through_the_map(struct FlatnessDarkoParams *p)
{
  struct FlatnessDarkoForce o;
  struct FloatVect3 fi = v3(0.f, 0.f, -p->m * p->g);
  struct FloatVect3 v  = v3(0.f, 0.f, 0.f);
  struct FloatVect3 by = v3(0.f, 1.f, 0.f);
  flatness_darko_force_transform(p, &fi, 0.f, &v, 0.f, &by, &o);

  ck_near("hover: HOVER-frame theta == 0",        o.euler.theta, 0.0,       1e-6);
  ck_near("hover: AERO-frame theta == +pi/2",     o.theta_aero,  M_PI / 2., 1e-6);

  /* The hover body z axis points DOWN in NED (thrust along body -z pulls up).
   * R_h_i maps NED -> body, so row 2 of it is b_z in NED. */
  ck_near("hover: hover b_z is +down in NED", RMAT_ELMT(o.rmat, 2, 2), 1.0, 1e-6);
  /* The aero body x axis points UP in NED (thrust along body +x). */
  ck_near("hover: aero b_x is -down in NED", RMAT_ELMT(o.rmat_aero, 0, 2), -1.0, 1e-6);
}

/* ======================================================================== */
/* [2] the attitude law                                                     */
/* ======================================================================== */

static void check_tspd(void)
{
  const float k_xi = 25.f, k_om = 7.f;
  struct FloatVect3 zero = v3(0.f, 0.f, 0.f);
  struct FloatVect3 out;

  findi_darko_tspd(k_xi, k_om, &zero, &zero, &zero, &out);
  ck("tsPD: zero error and zero rate command nothing",
     out.x == 0.f && out.y == 0.f && out.z == 0.f, "nonzero at the trim point");

  /* Proportional term alone. */
  struct FloatVect3 z = v3(0.1f, -0.2f, 0.3f);
  findi_darko_tspd(k_xi, k_om, &z, &zero, &zero, &out);
  ck_near("tsPD: P term is k_xi * zeta_e", out.y, -0.2 * 25.0, 1e-5);

  /* Damping term alone, and it differences Om_lpf -- NOT the raw rate. If the
   * quad's wiring were copied (raw rate into the loop) this check still
   * passes; what it pins is the SIGN and the gain, which a cascade
   * transcription gets wrong by a factor of k_att. */
  struct FloatVect3 om = v3(0.f, 1.f, 0.f);
  findi_darko_tspd(k_xi, k_om, &zero, &zero, &om, &out);
  ck_near("tsPD: D term is -k_om * Om_lpf", out.y, -7.0, 1e-5);

  /* Om_ref enters with the opposite sign to Om_lpf. */
  findi_darko_tspd(k_xi, k_om, &zero, &om, &om, &out);
  ck_exact("tsPD: Om_ref == Om_lpf cancels exactly", out.y, 0.0);

  /* The second-order reading the measured gains were swept against. */
  const double wn = sqrt((double)k_xi);
  const double zeta = k_om / (2.0 * wn);
  ck_near("tsPD: (25, 7) reads as wn = 5 rad/s",  wn,   5.0, 1e-9);
  ck_near("tsPD: (25, 7) reads as zeta = 0.7",    zeta, 0.7, 1e-9);
}

static void check_increment(void)
{
  const float J[3] = { FLATNESS_DARKO_JXX, FLATNESS_DARKO_JYY, FLATNESS_DARKO_JZZ };
  struct FloatVect3 omdot = v3(1.3f, -2.1f, 0.7f);
  struct FloatVect3 m_lpf = v3(0.011f, -0.004f, 0.002f);
  struct FloatVect3 m_c;

  /* THE defining property: no angular-acceleration error, no change to the
   * moment command. An implementation that dropped m_lpf, or that added a
   * spurious m_ext term, fails here and nowhere else at trim. */
  findi_darko_moment_cmd(J, &omdot, &omdot, &m_lpf, &m_c);
  ck_exact("increment: omdot_c == omdot_lpf gives m_c == m_lpf (x)", m_c.x, m_lpf.x);
  ck_exact("increment: omdot_c == omdot_lpf gives m_c == m_lpf (y)", m_c.y, m_lpf.y);
  ck_exact("increment: omdot_c == omdot_lpf gives m_c == m_lpf (z)", m_c.z, m_lpf.z);

  struct FloatVect3 omdot_c = v3(2.3f, -2.1f, 0.7f);
  findi_darko_moment_cmd(J, &omdot_c, &omdot, &m_lpf, &m_c);
  ck_near("increment: 1 rad/s^2 of error costs Jxx N*m",
          m_c.x - m_lpf.x, FLATNESS_DARKO_JXX, 1e-9);
}

/* ======================================================================== */
/* [3] axis correspondence -- the frame map against the real allocator       */
/* ======================================================================== */

/** A pure HOVER-ROLL demand is aero YAW, which eq (37) sends to differential
 * thrust. A pure HOVER-YAW demand is -aero ROLL, which the 2x2 flap solve
 * takes. If the frame map were transposed these two swap, and both still
 * "work" -- the aircraft just rolls when told to yaw. */
static void check_axis_correspondence(struct FlatnessDarkoParams *p)
{
  const struct FloatVect3 v = v3(0.f, 0.f, 0.f);
  /* Hover attitude: R_h_i = I, so R_i_b (aero body -> NED) is Ry(pi/2)^T. */
  struct FloatRMat R_h_i;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) { RMAT_ELMT(R_h_i, i, j) = (i == j) ? 1.f : 0.f; }
  }
  struct FloatRMat R_b_i;
  findi_darko_rmat_hover_to_aero(&R_h_i, &R_b_i);
  const struct FloatRMat R_i_b = transp(&R_b_i);

  const float T = p->m * p->g / (1.f - p->cDT);
  float u[4]; int8_t sat[4];

  /* Pure hover ROLL. */
  struct FloatVect3 m_h = v3(0.02f, 0.f, 0.f), m_a;
  findi_darko_hover_to_aero(&m_h, &m_a);
  ck("axis: hover ROLL maps to aero YAW", m_a.z == 0.02f && m_a.x == 0.f && m_a.y == 0.f,
     "roll did not land on the aero z axis");
  flatness_darko_allocate(p, T, &m_a, &v, &R_i_b, u, sat);
  ck("axis: hover ROLL comes out as differential THRUST",
     fabsf(u[0]) - fabsf(u[1]) > 1.f, "rotors stayed matched");
  ck("axis: hover ROLL leaves the flaps near zero",
     fabsf(u[2]) < 0.05f && fabsf(u[3]) < 0.05f, "flaps absorbed a roll demand");
  /* Positive hover roll must come from the LEFT rotor being the faster one --
   * rotor 1. This is the sense darko.xml's flight-proven stock G1_ROLL
   * {0, 0, -15, +15} for [ele_l, ele_r, RM, LM] reports. */
  ck("axis: positive hover ROLL speeds up rotor 1 (LEFT)",
     fabsf(u[0]) > fabsf(u[1]), "the roll sense is inverted");

  /* Pure hover YAW. */
  struct FloatVect3 y_h = v3(0.f, 0.f, 0.01f), y_a;
  findi_darko_hover_to_aero(&y_h, &y_a);
  ck("axis: hover YAW maps to -aero ROLL", y_a.x == -0.01f && y_a.z == 0.f,
     "yaw did not land on the negated aero x axis");
  flatness_darko_allocate(p, T, &y_a, &v, &R_i_b, u, sat);
  ck("axis: hover YAW comes out as DIFFERENTIAL flap",
     fabsf(u[2] - u[3]) > 1e-3f && fabsf(u[2] + u[3]) < 1e-3f,
     "yaw did not produce an antisymmetric flap pair");
  ck("axis: hover YAW leaves the rotors matched",
     fabsf(fabsf(u[0]) - fabsf(u[1])) < 1.f, "rotors absorbed a yaw demand");

  /* Pure PITCH: same axis in both frames, common-mode flap. */
  struct FloatVect3 q_h = v3(0.f, 0.005f, 0.f), q_a;
  findi_darko_hover_to_aero(&q_h, &q_a);
  ck_near("axis: PITCH is the same axis in both frames", q_a.y, 0.005, 1e-9);
  flatness_darko_allocate(p, T, &q_a, &v, &R_i_b, u, sat);
  ck("axis: PITCH comes out as COMMON-MODE flap",
     fabsf(u[2] + u[3]) > 1e-3f && fabsf(u[2] - u[3]) < 1e-3f,
     "pitch did not produce a symmetric flap pair");
}

/* ======================================================================== */
/* [4] actuator -> command maps                                             */
/* ======================================================================== */

/* The two calibration constants oneloop_findi_darko.c ships. Duplicated here
 * rather than included, because the .c cannot be compiled on the host -- so a
 * change to either must be made in both places, and this comment is the only
 * thing that says so. */
#define TEST_W_FULL_CMD 1300.0f
#define TEST_D_FULL_CMD 0.638f

static void check_actuator_maps(struct FlatnessDarkoParams *p)
{
  /* w2 IS EMITTED NEGATIVE by the allocator -- the plant signs the rotor SPEED,
   * not the torque. An ESC takes no sign, and passing it through would command
   * a stopped right rotor at hover. */
  ck_near("act: w_to_cmd uses |w| (w2 is negative)",
          findi_darko_w_to_cmd(-650.f, TEST_W_FULL_CMD), 0.5, 1e-6);
  ck_exact("act: w_to_cmd clamps above full", findi_darko_w_to_cmd(5000.f, TEST_W_FULL_CMD), 1.0);
  ck_exact("act: w_to_cmd is 0 at 0",         findi_darko_w_to_cmd(0.f, TEST_W_FULL_CMD), 0.0);

  ck_near("act: d_to_cmd keeps the sign",
          findi_darko_d_to_cmd(-0.319f, TEST_D_FULL_CMD), -0.5, 1e-5);
  ck_exact("act: d_to_cmd clamps at +1", findi_darko_d_to_cmd(9.f, TEST_D_FULL_CMD), 1.0);
  ck_exact("act: d_to_cmd clamps at -1", findi_darko_d_to_cmd(-9.f, TEST_D_FULL_CMD), -1.0);

  /* Hover trim must land in the usable middle of the motor range. A wrong
   * W_FULL_CMD by an order of magnitude shows up here and not in flight,
   * where it looks like a thrust gain problem. */
  const float T_hover = p->m * p->g / (1.f - p->cDT);
  const float w_hover = sqrtf((T_hover * 0.5f) / p->cT);
  const float c_hover = findi_darko_w_to_cmd(w_hover, TEST_W_FULL_CMD);
  printf("      hover: T = %.4f N total, |w| = %.2f rad/s, motor cmd = %.4f\n",
         T_hover, w_hover, c_hover);
  ck("act: hover motor command is in (0.2, 0.8)",
     c_hover > 0.2f && c_hover < 0.8f, "hover trim is not in the usable range");
}

/* ======================================================================== */
/* [5] elevon sign, against the SITL plant's own coefficients               */
/* ======================================================================== */

/* Measured off conf/simulator/jsbsim/aircraft/Systems/aerodynamics_cyfoam.xml,
 * converted FT*LBS -> N*m (x 1.35582), per unit of normalised elevon command:
 *
 *   pitch = -0.0417 e_l + 0.0417 e_r      ->  +/- 0.056538 N*m
 *   yaw   = -0.35  (e_l + e_r)            ->    -0.474537 N*m each
 *
 * These are the SIM's numbers, not Darko's. The check is not that the
 * MAGNITUDES agree -- they do not, and the calibration constants exist because
 * of that -- but that ONE sign choice in the pprz mapping gives the model and
 * the plant the same sign on BOTH axes. Either sign flipped and one of the two
 * inverts, which is what makes this more than a guess. */
#define PLANT_ELE_PITCH 0.056538f
#define PLANT_ELE_YAW   (-0.474537f)

/* The mapping oneloop_findi_darko.c commits: the elevons are mounted mirrored,
 * so equal-sign pprz is equal-and-opposite PHYSICAL deflection. */
#define MAP_D1_SIGN (+1.0f)
#define MAP_D2_SIGN (-1.0f)

static void plant_moment_hover(float d1, float d2, float *pitch, float *yaw)
{
  const float e_l = MAP_D1_SIGN * d1 / TEST_D_FULL_CMD;
  const float e_r = MAP_D2_SIGN * d2 / TEST_D_FULL_CMD;
  *pitch = PLANT_ELE_PITCH * (e_r - e_l);
  *yaw   = PLANT_ELE_YAW   * (e_l + e_r);
}

/** The model's HOVER-frame flap moment, straight out of the spine's wrench at
 * a hover trim, with the rotors held at the trim speed so only the flaps move. */
static void model_moment_hover(struct FlatnessDarkoParams *p, float d1, float d2,
                               float *pitch, float *yaw)
{
  struct FloatRMat R_h_i;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) { RMAT_ELMT(R_h_i, i, j) = (i == j) ? 1.f : 0.f; }
  }
  struct FloatRMat R_b_i;
  findi_darko_rmat_hover_to_aero(&R_h_i, &R_b_i);
  const struct FloatRMat R_i_b = transp(&R_b_i);

  const float T_hover = p->m * p->g / (1.f - p->cDT);
  const float w_h = sqrtf((T_hover * 0.5f) / p->cT);
  const float w[2] = { w_h, -w_h };
  const float d[2] = { d1, d2 };
  const struct FloatVect3 v = v3(0.f, 0.f, 0.f);

  struct FloatVect3 f_ned, m_aero, m_hover;
  flatness_darko_wrench(p, w, d, &v, &R_i_b, &f_ned, &m_aero);
  findi_darko_aero_to_hover(&m_aero, &m_hover);
  *pitch = m_hover.y;
  *yaw   = m_hover.z;
}

static void check_elevon_sign(struct FlatnessDarkoParams *p)
{
  float mp, my, pp, py;

  /* Common-mode deflection: PITCH. */
  model_moment_hover(p, 0.1f, 0.1f, &mp, &my);
  plant_moment_hover(0.1f, 0.1f, &pp, &py);
  printf("      common-mode d = +0.1 rad: model pitch %+0.6f, plant pitch %+0.6f N*m\n", mp, pp);
  ck("elevon: common-mode flap gives PITCH in the model", fabsf(mp) > 1e-4f && fabsf(my) < 1e-6f,
     "common-mode flap did not come out as pure pitch");
  ck("elevon: model and plant agree on the PITCH sign", (mp > 0.f) == (pp > 0.f),
     "the mapping inverts pitch in the sim plant");

  /* Differential deflection: YAW. */
  model_moment_hover(p, 0.1f, -0.1f, &mp, &my);
  plant_moment_hover(0.1f, -0.1f, &pp, &py);
  printf("      differential d = +/-0.1 rad: model yaw %+0.6f, plant yaw %+0.6f N*m\n", my, py);
  ck("elevon: differential flap gives YAW in the model", fabsf(my) > 1e-4f && fabsf(mp) < 1e-6f,
     "differential flap did not come out as pure yaw");
  ck("elevon: model and plant agree on the YAW sign", (my > 0.f) == (py > 0.f),
     "the mapping inverts yaw in the sim plant");

  /* THE point of the check: one sign choice, both axes. Flipping D2 must break
   * exactly one of them -- if it broke both or neither the check is vacuous. */
  const float e_l =  0.1f / TEST_D_FULL_CMD;
  const float e_r = +0.1f / TEST_D_FULL_CMD;      /* D2_SIGN flipped to +1 */
  const float pitch_flipped = PLANT_ELE_PITCH * (e_r - e_l);
  ck("elevon: flipping the D2 sign kills the pitch agreement",
     fabsf(pitch_flipped) < 1e-9f || (pitch_flipped > 0.f) != (mp > 0.f),
     "the check does not discriminate the sign it claims to");
}

/* ======================================================================== */
/* [6] the hover chain, end to end                                          */
/* ======================================================================== */

/** transform -> attitude error -> tsPD -> increment -> allocate, at a settled
 * hover. Zero attitude error and a settled wrench must reproduce the hover
 * actuator command: the loop does nothing when there is nothing to do. */
static void check_hover_chain(struct FlatnessDarkoParams *p)
{
  const struct FloatVect3 v = v3(0.f, 0.f, 0.f);
  const struct FloatVect3 by = v3(0.f, 1.f, 0.f);
  struct FloatVect3 fi = v3(0.f, 0.f, -p->m * p->g);

  struct FlatnessDarkoForce o;
  const bool ok = flatness_darko_force_transform(p, &fi, 0.f, &v, 0.f, &by, &o);
  ck("chain: the transform succeeds at hover", ok, "roll reported undetermined at hover");

  /* Measured attitude == commanded attitude: zero error. */
  struct FloatVect3 zeta_hover, zeta_aero;
  flatness_quad_att_error(&o.quat, &o.quat, &zeta_hover);
  findi_darko_hover_to_aero(&zeta_hover, &zeta_aero);
  ck_near("chain: attitude error is zero at the commanded attitude",
          sqrt((double)(zeta_aero.x * zeta_aero.x + zeta_aero.y * zeta_aero.y
                        + zeta_aero.z * zeta_aero.z)), 0.0, 1e-6);

  /* Settled: no rate, no angular acceleration. */
  const struct FloatVect3 zero = v3(0.f, 0.f, 0.f);
  struct FloatVect3 omdot_c;
  findi_darko_tspd(25.f, 7.f, &zeta_aero, &zero, &zero, &omdot_c);
  ck_near("chain: settled hover commands no angular acceleration",
          sqrt((double)(omdot_c.x * omdot_c.x + omdot_c.y * omdot_c.y
                        + omdot_c.z * omdot_c.z)), 0.0, 1e-5);

  /* m_lpf: the wrench the trim actuators are producing. */
  const struct FloatRMat R_i_b = transp(&o.rmat_aero);
  const float w_h = sqrtf((o.T * 0.5f) / p->cT);
  const float w[2] = { w_h, -w_h };
  const float d[2] = { 0.f, 0.f };
  struct FloatVect3 f_ned, m_lpf;
  flatness_darko_wrench(p, w, d, &v, &R_i_b, &f_ned, &m_lpf);
  ck_near("chain: the trim wrench lifts exactly m*g", f_ned.z, -(double)(p->m * p->g), 1e-4);

  const float J[3] = { p->Jxx, p->Jyy, p->Jzz };
  struct FloatVect3 m_c;
  findi_darko_moment_cmd(J, &omdot_c, &zero, &m_lpf, &m_c);

  float u[4]; int8_t sat[4];
  const bool a_ok = flatness_darko_allocate(p, o.T, &m_c, &v, &R_i_b, u, sat);
  ck("chain: allocation succeeds at hover", a_ok, "allocator reported no flap authority");
  ck_near("chain: the loop reproduces the trim rotor speed", fabsf(u[0]), w_h, 1e-2);
  ck_near("chain: rotor 2 is emitted negative",             u[1],       -w_h, 1e-2);
  ck_near("chain: the flaps stay at zero",   sqrt((double)(u[2] * u[2] + u[3] * u[3])), 0.0, 1e-4);

  const float c1 = findi_darko_w_to_cmd(u[0], TEST_W_FULL_CMD);
  const float c2 = findi_darko_w_to_cmd(u[1], TEST_W_FULL_CMD);
  printf("      hover commit: motor cmds %.4f / %.4f, flaps %.4f / %.4f rad\n",
         c1, c2, u[2], u[3]);
  ck("chain: both motor commands land in (0.2, 0.8)",
     c1 > 0.2f && c1 < 0.8f && c2 > 0.2f && c2 < 0.8f, "hover trim off the usable range");
}

/* ======================================================================== */
/* [7] tilt limit                                                           */
/* ======================================================================== */

static void check_limit_tilt(void)
{
  const float bank = 0.5236f;   /* 30 deg */

  /* Inside the envelope: untouched. */
  struct FloatVect3 f = v3(0.5f, 0.f, -5.f);
  findi_darko_limit_tilt(&f, bank);
  ck_exact("tilt: a force inside the envelope is untouched", f.x, 0.5);

  /* Outside: horizontal scaled, VERTICAL PRESERVED. Giving up the vertical
   * demand at the bank limit is the last thing you want. */
  f = v3(10.f, 0.f, -5.f);
  findi_darko_limit_tilt(&f, bank);
  ck_near("tilt: horizontal is clamped to -fz tan(bank)", f.x, 5.0 * tan(0.5236), 1e-4);
  ck_exact("tilt: the vertical demand is preserved", f.z, -5.0);

  /* Direction preserved, not just magnitude. */
  f = v3(6.f, 8.f, -5.f);
  findi_darko_limit_tilt(&f, bank);
  ck_near("tilt: the horizontal DIRECTION is preserved", f.y / f.x, 8.0 / 6.0, 1e-5);

  /* fz >= 0 is left to the transform's T >= 0 branch. */
  f = v3(3.f, 0.f, 1.f);
  findi_darko_limit_tilt(&f, bank);
  ck_exact("tilt: a downward force is left alone", f.x, 3.0);
}

int main(void)
{
  struct FlatnessDarkoParams p;
  flatness_darko_params_init(&p);

  printf("Darko FINDI controller law -- analytic property checks\n\n");
  printf("[1] frames\n");
  check_frame_map();
  check_rmat_map(&p);
  check_hover_frame_through_the_map(&p);

  printf("\n[2] the attitude law\n");
  check_tspd();
  check_increment();

  printf("\n[3] axis correspondence\n");
  check_axis_correspondence(&p);

  printf("\n[4] actuator maps\n");
  check_actuator_maps(&p);

  printf("\n[5] elevon sign vs the SITL plant\n");
  check_elevon_sign(&p);

  printf("\n[6] the hover chain\n");
  check_hover_chain(&p);

  printf("\n[7] tilt limit\n");
  check_limit_tilt();

  printf("\nRESULT: %s\n", g_fail ? "FAIL" : "all checks passed");
  return g_fail;
}
