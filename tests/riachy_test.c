/* Hand-derived property checks on Riachy's trick in mfc_core.c
 * (struct MfcParameters::use_riachy).
 *
 * No golden traces and none are coming: MFC_SISO's reference runs the
 * SLIDING-WINDOW estimator and integrates the raw measurement, while this core
 * has only the ALGEBRAIC estimator and integrates with a latched offset. Those
 * are deliberate differences (see mfc_core.h), so a trace comparison would fail
 * for documented reasons rather than wrong ones.
 *
 * WHAT THIS FILE RECORDS. The implementation is correct -- §1-§4 pin the
 * auxiliary output, the closed-loop poles, the reference consistency and the
 * noise win. It is nonetheless NOT USABLE in this core as it stands, because
 * mfc_est_num() accumulates in float32 and its accuracy floor is proportional to
 * the size of the estimator drive signal, which for Riachy is Y = y + kd*int y
 * and therefore RAMPS WITHOUT BOUND. §5 pins that decay, and §6 pins the
 * consequence at Hoops_111_MFC's own gains. See the long comment on
 * mfc_est_num() for the arithmetic and for the three ways out.
 *
 * §5 and §6 are PINNED LIMITS, not aspirations: they assert that the trick
 * still degrades. Widening mfc_est_num()'s accumulation to double, or adding a
 * sliding-window estimator, will make them fail -- and that failure is the
 * signal to rewrite them, not to relax them.
 *
 * Plant throughout: the SISO ultra-local model the core assumes,
 * ddot_y = F + alpha*u, integrated semi-implicitly.
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <string.h>

#include "firmwares/rotorcraft/stabilization/mfc_core.h"

double stub_sys_time = 0.0;

#define TS       (1.f / 500.f)
#define AX_ALPHA 15.f

/* Two gain sets, and the difference between them is the point.
 *
 * FAST (kp = 64, kd = 16; p = 8, the Knowledge note's own choice) puts both
 * closed-loop poles at -8 rad/s, so the loop settles in well under a second --
 * fast enough to finish before the float32 drift in F_hat has grown. This is
 * where the STRUCTURE is verified.
 *
 * HOOPS (kp = 2, kd = 25) is what the airframe ships. Its dominant pole is at
 * -0.08 rad/s, i.e. ~12.5 s of settling, which is LONGER THAN THE PRECISION
 * LIFETIME of the estimate. No amount of window tuning fixes that ordering --
 * see §6. */
#define FAST_KP  64.f
#define FAST_KD  16.f
#define HOOPS_KP  2.f
#define HOOPS_KD 25.f
/* Airframe ships 600 for the stock iPD. Riachy needs the estimator much faster
 * because F_hat now carries kd*dot_y and therefore moves fast; 32 is the value
 * the window sweep in §5 settles on. */
#define WIN_RIACHY 32.f
#define WIN_STOCK 600.f

static int failures = 0;

static void check(const char *name, bool ok, const char *fmt, ...)
{
  va_list ap;
  printf("  %-42s ", name);
  va_start(ap, fmt); vprintf(fmt, ap); va_end(ap);
  printf("  %s\n", ok ? "PASS" : "FAIL");
  if (!ok) { failures++; }
}

struct cfg { float kp, kd, window; bool riachy; };

static void axis_setup(struct MfcParameters *m, struct cfg c)
{
  memset(m, 0, sizeof(*m));
  mfc_siso_init(m, TS);
  m->enabled        = true;
  m->alpha          = AX_ALPHA;
  m->kp             = c.kp;
  m->kd             = c.kd;
  m->ki             = 0.f;
  m->decoupled      = true;
  m->use_riachy     = c.riachy;
  m->est_hold_time  = 0.1f;
  m->time_trajec    = 50.f;
  m->use_trajec_sp  = true;
  m->int_window     = c.window;
  m->command_filter = 1.f;
  /* Wide clamps: saturation is a separate concern and would confound every
   * property below. */
  m->u_min = -1e6f;
  m->u_max =  1e6f;
}

/* Deterministic uniform noise, so the two arms of the noise check see the SAME
 * disturbance sequence. */
static uint32_t rng_state;
static void  rng_seed(uint32_t s) { rng_state = s; }
static float rng_pm1(void)
{
  rng_state = rng_state * 1664525u + 1013904223u;
  return ((float)(rng_state >> 8) / (float)(1u << 24)) * 2.f - 1.f;
}

struct run_result {
  float err_tail_max;   /* max |e| over the last quarter */
  float peak_err;       /* max |e| over the whole run */
  float u_tail_std;     /* std(command) over the last quarter */
  bool  finite;
};

static struct run_result run_loop(struct cfg c, float sp0, float sp_rate,
                                  float noise, float F_true, float t_end)
{
  struct MfcParameters m;
  axis_setup(&m, c);

  float pos = 0.f, vel = 0.f;
  stub_sys_time = 0.0;
  mfc_siso_reset(&m);

  const int n     = (int)(t_end / TS);
  const int tail0 = (3 * n) / 4;
  double u_sum = 0.0, u_sq = 0.0;
  int    u_n = 0;
  struct run_result r = { 0.f, 0.f, 0.f, true };

  rng_seed(12345u);

  for (int k = 0; k < n; k++) {
    stub_sys_time = (double)k * (double)TS;
    float t = (float)stub_sys_time;

    m.setpoint = (sp_rate != 0.f) ? (sp0 + sp_rate * t) : sp0;
    mfc_siso_run(true, &m, pos + noise * rng_pm1());
    float u = m.command[0];
    if (!isfinite(u) || !isfinite(m.estimator)) { r.finite = false; break; }

    float acc = F_true + AX_ALPHA * u;
    vel += acc * TS;
    pos += vel * TS;

    float e = fabsf(pos - m.setpoint);
    if (e > r.peak_err) { r.peak_err = e; }
    if (k >= tail0) {
      if (e > r.err_tail_max) { r.err_tail_max = e; }
      u_sum += u; u_sq += (double)u * (double)u; u_n++;
    }
  }
  if (u_n > 0) {
    double mean = u_sum / u_n;
    double var  = u_sq / u_n - mean * mean;
    r.u_tail_std = (float)sqrt(var > 0.0 ? var : 0.0);
  }
  return r;
}

int main(void)
{
  const struct cfg fast_on   = { FAST_KP,  FAST_KD,  WIN_RIACHY, true  };
  const struct cfg fast_off  = { FAST_KP,  FAST_KD,  WIN_RIACHY, false };
  const struct cfg hoops_on  = { HOOPS_KP, HOOPS_KD, WIN_RIACHY, true  };
  const struct cfg hoops_off = { HOOPS_KP, HOOPS_KD, WIN_STOCK,  false };

  printf("\nRiachy's trick (mfc_core.c use_riachy) -- property checks\n");
  printf("Ts = %g (500 Hz), alpha = %g\n", (double)TS, (double)AX_ALPHA);
  printf("structure verified at kp=%g kd=%g (poles -8,-8); Hoops ships kp=%g kd=%g\n\n",
         (double)FAST_KP, (double)FAST_KD, (double)HOOPS_KP, (double)HOOPS_KD);

  /* ── [0] Gating ───────────────────────────────────────────────────────── */
  printf("[0] gating\n");
  {
    struct MfcParameters a, b;
    struct cfg ca = fast_off, cb = fast_on;
    axis_setup(&a, ca); a.decoupled = false;
    axis_setup(&b, cb); b.decoupled = false;   /* use_riachy ON, coupled */
    stub_sys_time = 0.0;
    mfc_siso_reset(&a); mfc_siso_reset(&b);
    float worst = 0.f;
    for (int k = 0; k < 2000; k++) {
      stub_sys_time = (double)k * (double)TS;
      float meas = 0.3f * sinf(6.f * (float)stub_sys_time) + 1.f;
      a.setpoint = 1.f; b.setpoint = 1.f;
      mfc_siso_run(true, &a, meas);
      mfc_siso_run(true, &b, meas);
      float d = fabsf(a.command[0] - b.command[0]);
      if (d > worst) { worst = d; }
    }
    check("coupled: use_riachy is a no-op", worst == 0.f,
          "worst |du| = %.3e  (tol 0)", (double)worst);
  }

  /* ── [1] The auxiliary output ─────────────────────────────────────────── */
  printf("\n[1] auxiliary output Y = y + kd*int y\n");
  {
    struct MfcParameters m; axis_setup(&m, fast_off);
    stub_sys_time = 0.0; mfc_siso_reset(&m);
    float worst = 0.f;
    for (int k = 0; k < 500; k++) {
      stub_sys_time = (double)k * (double)TS;
      m.setpoint = 0.f;
      mfc_siso_run(true, &m, 2.5f + 0.1f * (float)k);
      float d = fabsf(m.riachy_Y - m.measure);
      if (d > worst) { worst = d; }
    }
    check("off: Y == measure", worst == 0.f, "worst |dY| = %.3e  (tol 0)", (double)worst);
  }
  {
    /* Constant measurement from reset: the offset is latched to it, the integrand
     * is identically zero, and Y must equal the measurement EXACTLY however long
     * the run. This is the latched-offset conditioning property. */
    struct MfcParameters m; axis_setup(&m, fast_on);
    stub_sys_time = 0.0; mfc_siso_reset(&m);
    const float c = 37.5f;   /* a position far from the NED origin */
    float worst = 0.f;
    for (int k = 0; k < 150000; k++) {   /* 300 s at 500 Hz */
      stub_sys_time = (double)k * (double)TS;
      m.setpoint = c;
      mfc_siso_run(true, &m, c);
      float d = fabsf(m.riachy_Y - c);
      if (d > worst) { worst = d; }
    }
    check("on: constant y -> no Y drift in 300 s", worst == 0.f,
          "worst |Y - y| = %.3e  (tol 0)", (double)worst);
  }
  {
    /* Trapezoidal, not forward Euler. y ramps at v from the latched offset, so
     * int(y - y0) = v*t^2/2 exactly under the trapezoid rule. Forward Euler
     * would be short by v*Ts*t/2 -- at t = 1 s, v = 1, kd = 16 that is 0.016,
     * far outside this tolerance. */
    struct MfcParameters m; axis_setup(&m, fast_on);
    stub_sys_time = 0.0; mfc_siso_reset(&m);
    const float v = 1.0f, y0 = 4.f;
    float worst = 0.f;
    for (int k = 0; k < 500; k++) {
      stub_sys_time = (double)k * (double)TS;
      float t = (float)stub_sys_time, y = y0 + v * t;
      m.setpoint = y;
      mfc_siso_run(true, &m, y);
      float d = fabsf(m.riachy_Y - (y + FAST_KD * (v * t * t / 2.f)));
      if (d > worst) { worst = d; }
    }
    check("on: trapezoidal integral exact on a ramp", worst < 1e-4f,
          "worst dev = %.3e  (tol 1e-4)", (double)worst);
  }

  /* ── [2] Closed loop: same poles ──────────────────────────────────────── */
  printf("\n[2] closed loop, 1 m step, F = 3 m/s^2 (8 s, kp=64 kd=16)\n");
  {
    struct run_result off = run_loop(fast_off, 1.f, 0.f, 0.f, 3.f, 8.f);
    struct run_result on  = run_loop(fast_on,  1.f, 0.f, 0.f, 3.f, 8.f);
    check("stock iPD converges", off.finite && off.err_tail_max < 1e-2f,
          "tail |e| = %.3e", (double)off.err_tail_max);
    check("riachy converges", on.finite && on.err_tail_max < 1.5e-1f,
          "tail |e| = %.3e", (double)on.err_tail_max);
    /* Same closed-loop polynomial, so the transient must be the same order.
     * Riachy is ALLOWED to be worse -- the estimator's lag on the fast-moving
     * kd*dot_y term is the documented cost -- just not by an order of magnitude. */
    check("riachy peak within 3x of stock",
          on.peak_err < 3.f * off.peak_err + 1e-3f,
          "peak on = %.3e  off = %.3e", (double)on.peak_err, (double)off.peak_err);
  }

  /* ── [3] Reference consistency: the kd*dot_sp feedforward ─────────────── */
  printf("\n[3] moving setpoint -- the kd*dot_sp feedforward\n");
  {
    /* THE DISCRIMINATING CHECK, and it is deliberately gain-independent.
     * Dropping the kd*dot_sp term leaves a standing error of kd*v/kp, i.e. one
     * PROPORTIONAL TO THE RAMP RATE. So run two rates 4x apart: if the
     * feedforward is present the tail error is essentially the same for both; if
     * it were dropped the error would scale with v (0.25 -> 1.0 m here). This
     * cannot be faked by a coincidence of tolerances. */
    struct run_result slow = run_loop(fast_on, 0.f, 1.f, 0.f, 3.f, 8.f);
    struct run_result quick = run_loop(fast_on, 0.f, 4.f, 0.f, 3.f, 8.f);
    float scaling = fabsf(quick.err_tail_max - slow.err_tail_max);
    check("riachy: ramp error does not scale with rate",
          slow.finite && quick.finite && scaling < 0.15f,
          "|e| v=1: %.3e  v=4: %.3e  (would differ by %.2f if ff dropped)",
          (double)slow.err_tail_max, (double)quick.err_tail_max,
          (double)(FAST_KD * 3.f / FAST_KP));
  }

  /* ── [4] The noise result, which is the point of the trick ────────────── */
  printf("\n[4] measurement noise (+-1 cm uniform on the position)\n");
  {
    struct run_result off = run_loop(fast_off, 1.f, 0.f, 0.01f, 3.f, 8.f);
    struct run_result on  = run_loop(fast_on,  1.f, 0.f, 0.01f, 3.f, 8.f);
    /* kd differencing a position at 500 Hz is the dominant noise path in the
     * stock loop; replacing it with an integral must cut command noise hard.
     * This is the one result that survives the precision problem intact. */
    check("riachy cuts command noise >=5x",
          on.finite && off.finite && on.u_tail_std < 0.2f * off.u_tail_std,
          "std(u) on = %.4f  off = %.4f  (%.0fx better)",
          (double)on.u_tail_std, (double)off.u_tail_std,
          (double)(on.u_tail_std > 0.f ? off.u_tail_std / on.u_tail_std : 0.f));
  }

  /* ── [5] PINNED LIMIT: the float32 floor grows with int y ─────────────── */
  printf("\n[5] PINNED LIMIT -- error grows with |Y|, not with t (kp=64 kd=16)\n");
  {
    const float horizons[] = { 8.f, 20.f, 40.f, 60.f };
    float tail[4];
    for (int i = 0; i < 4; i++) {
      struct run_result r = run_loop(fast_on, 1.f, 0.f, 0.f, 3.f, horizons[i]);
      tail[i] = r.finite ? r.err_tail_max : INFINITY;
      printf("      horizon %4.0f s   tail |e| = %10.3g\n",
             (double)horizons[i], (double)tail[i]);
    }
    struct run_result ref = run_loop(fast_off, 1.f, 0.f, 0.f, 3.f, 60.f);
    printf("      stock, 60 s     tail |e| = %10.3g  (flat -- |z| is bounded)\n",
           (double)ref.err_tail_max);
    /* int y ramps, so |Y| ramps, so err(F_hat) ~ 3*eps*|Y|/ts^2 ramps with it.
     * The loop does not blow up, it DECAYS IN ACCURACY -- which is the more
     * dangerous failure mode, because a short run looks fine. */
    check("riachy degrades with horizon (float32 floor)",
          tail[3] > 4.f * tail[0],
          "60 s / 8 s = %.1fx worse", (double)(tail[3] / tail[0]));
    check("stock does NOT degrade (bounded drive)",
          ref.err_tail_max < 1e-2f, "tail |e| = %.3e", (double)ref.err_tail_max);
  }

  /* ── [6] PINNED LIMIT: unusable at the airframe's own gains ───────────── */
  printf("\n[6] PINNED LIMIT -- Hoops gains kp=%g kd=%g cannot work in float32\n",
         (double)HOOPS_KP, (double)HOOPS_KD);
  {
    /* The ordering is what kills it, and no window value reorders it: the
     * dominant pole at -kp/kd = -0.08 rad/s needs ~12.5 s to settle, while at
     * kd = 25 the estimate is already dominated by rounding after ~4 s. The loop
     * therefore never gets a chance to converge. */
    struct run_result off = run_loop(hoops_off, 1.f, 0.f, 0.f, 3.f, 40.f);
    struct run_result on  = run_loop(hoops_on,  1.f, 0.f, 0.f, 3.f, 40.f);
    printf("      settling ~%.1f s (pole -kp/kd);  precision lifetime ~4 s at kd=%g\n",
           (double)(1.f / (HOOPS_KP / HOOPS_KD)), (double)HOOPS_KD);
    check("stock iPD is fine at these gains", off.finite && off.err_tail_max < 5e-2f,
          "tail |e| = %.3e", (double)off.err_tail_max);
    check("riachy does NOT converge at these gains", on.err_tail_max > 1.f,
          "tail |e| = %.3e  (this is the blocker)", (double)on.err_tail_max);
  }

  printf("\nRESULT: %s\n\n", failures == 0 ? "all checks passed"
                                           : "FAILURES PRESENT");
  return failures == 0 ? 0 : 1;
}
