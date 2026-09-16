# HEOL MIMO Horizontal Channel (Matrix Alpha Port)

Plan for replacing the two independent scalar horizontal HEOL channels with a
single $2\times2$ MIMO channel whose input gain is the acceleration-to-tilt
Jacobian.

Prerequisite: the flat-nominal-inputs work must have landed first, so that
`T_ref`, `phi_ref`, `theta_ref` are onboard and the shared input-sensitivity
block exists. See `Knowledge/Plans/HEOL Guidance Against the Canonical Spec
(Flat Nominal Inputs).md`, which this plan continues.

## Why this document exists

The MATLAB reference implementation lives in the `MFC_SISO` sim repo and the
controller specification lives in the thesis vault. **Neither is mounted in this
repo.** Everything needed to do the port is therefore transcribed here:
the algorithms, the tuning constants, the plant models, and the exact conditions
that generated the reference traces. You should not need to see either repo.

What did cross the boundary, copied in from the root tier:

```
tests/golden/mimo_diag.csv        601 samples, 11 columns
tests/golden/mimo_cross.csv       601 samples, 11 columns
tests/golden/2nd_decoupled_alg.csv  601 samples, 6 columns  (SISO, for the reduction check)
```

These are the authoritative reference. Do not regenerate or "fix" them here —
they are outputs of the sim repo, and a mismatch means the port is wrong, not
the trace.

## What is actually being changed

**This is not a scalar-gain-to-matrix swap.** The input gain is
$\partial(\ddot x, \ddot y)/\partial(\phi,\theta)$, so the $2\times2$ solve *is*
the acceleration-to-attitude inversion. Consequences:

- The channel's output becomes **attitude increments** $\Delta(\phi,\theta)$,
  summed onto `phi_ref`/`theta_ref` and saturated at max bank.
- `accel_to_att_sp()` leaves the feedback path **entirely**. It was standing in
  for this inversion. Keeping both would put a linearized inversion and an exact
  one in series.
- Today's structure — two scalar gains then `accel_to_att_sp()` — is an ad-hoc
  factorization of exactly this matrix. That is why the earlier attempt to
  "diagonalize alpha" was closed as a wrong premise rather than completed.

## The input gain

With this repo's negative-thrust convention ($T = $ `thrust_ref` $< 0$, hover
$= -mg$), and $c_\bullet, s_\bullet$ meaning $\cos, \sin$:

$$\boldsymbol\alpha_{xy} = \frac{T}{m}\begin{bmatrix} s_\psi c_\phi - c_\psi s_\theta s_\phi & c_\psi c_\theta c_\phi \\ -c_\psi c_\phi - s_\psi s_\theta s_\phi & s_\psi c_\theta c_\phi \end{bmatrix}$$

evaluated on **reference** quantities $(T^{ref}, \phi^{ref}, \theta^{ref},
\psi^{ref})$, never measured ones. It comes from the shared input-sensitivity
block built by the previous stage, which should already be computing and
publishing it to telemetry with no control consumer — this task connects it.

Numbers computed against all 751 rows of the demo trajectory:

- At level attitude with $\psi = 0$: $\begin{bmatrix}0 & -9.81\\ +9.81 & 0\end{bmatrix}$ — **anti-diagonal**. $\ddot x$ responds to $\theta$, $\ddot y$ to $\phi$, diagonal entries zero. The matrix rotates as $\psi$ sweeps to 45°, and the diagonal entries become nonzero.
- $|\det|$ ranges **51.5 … 179.7**; condition number **1.000 … 1.245**. Essentially perfectly conditioned over the whole trajectory. **Use a closed-form $2\times2$ inverse.** No pseudo-inverse, no regularization, no solver library — and no need for a singularity guard beyond a plain determinant check for safety.
- The constants it replaces are `GX_ALPHA = GY_ALPHA = 15/m = 18.75`.

## The reference algorithms, transcribed

Both MATLAB blocks are thin wrappers. The real content is small.

### Estimator — `mfc_fhat_alg2_decoupled_mimo_block`

Estimates $F$ in $\ddot{\mathbf y} = \mathbf F + \boldsymbol\alpha \mathbf u$
with $\mathbf y, \mathbf u, \mathbf F$ being $n\times1$ and
$\boldsymbol\alpha$ an $n \times n$ matrix. It calls the *unchanged* scalar
estimator function with `a_fold = b_fold = 0` — that function was already
vector-safe.

**The one structural property that matters for the port**, and the thing most
likely to be got wrong:

> The numerator is **per-element** ($n\times1$). The denominator is $t^2$ —
> **scalar and shared** across the whole vector. One integration window, one
> IIR smoother pair, serves all channels.

State, with sizes:

```
z_km1, z_km2               n x 1
num_filt_km1, num_filt_km2 n x 1
den_filt_km1, den_filt_km2 1 x 1   <-- scalar, shared
```

`den_filt_*` being scalar is directly observable in the golden traces: they
carry a single `valid` column, not one per channel.

**Correspondence to what this repo already has.** `mfc_core.c` already
implements this exact recursion for the scalar case. The delta is narrow:

| `mfc_core.c` today | MIMO version |
|---|---|
| `z[0..2]`, `estimator_num[0..2]` scalar | become 2-vectors |
| `estimator_den[0..2]` scalar | **stays scalar** |
| `int_window` / IIR on num and den | one window; num filter runs per element, den filter once |
| `alpha * d2u` scalar product | `alpha @ d2u` matrix–vector product |
| `F_k = num/den` | `F_k[i] = num[i]/den`, same scalar `den` |
| hold until `time > est_hold_time` | unchanged, one shared clock |

Do not reimplement the recursion from scratch. Factor the existing one.

### Command law — `mfc_command_mimo_block`

$$\boldsymbol\alpha\,\mathbf u = -\widehat{\mathbf F} + \mathbf f_f - \mathbf f_b \quad\Longrightarrow\quad \mathbf u = \boldsymbol\alpha^{-1}\left(-\widehat{\mathbf F} + \mathbf f_f - \mathbf f_b\right)$$

Stateless. The feedback is **subtracted**; the error is $\mathbf e = \mathbf y -
\mathbf y_{sp}$. With a decoupled estimator the feedback is an explicit
per-channel PD on that error. Saturation, where enabled, is element-wise and has
**no anti-windup**.

**The same $\boldsymbol\alpha$ matrix must be fed to both the estimator and the
command law.** Stated explicitly in the reference block's own documentation.
This is easy to get silently wrong once they are separate translation units —
the loop still looks plausible and is subtly wrong. Compute
$\boldsymbol\alpha$ once per tick and pass it to both; never recompute it
independently in two places.

## Replaying the golden traces

The traces are closed-loop. To reproduce them in C you need the driving
conditions, all transcribed below.

**Shared tuning** (identical to the existing 2nd-order SISO golden variants):

```
Ts        = 0.01        WFilter   = 10        FFilter = 10
hold_time = 0.1         Ki        = 0
tau_design = 0.05  ->  p = (1/tau_design)/5 = 4
Kp = p^2 = 16           Kd = 2*p = 8
t   = 0 : 0.01 : 6      (601 samples)
ref = unit step at t >= 0.2, same reference on every channel
n   = 2
```

**Variant `mimo_diag`** — diagonal gain, two identical channels:

```
alpha = eye(2)
plant: g = [9.81; 9.81]   d = [0.20; 0.20]   b = [1.0; 1.0]   tau = [0.05; 0.05]
```

**Variant `mimo_cross`** — off-diagonal gain, two different plants:

```
alpha = [ 1.00  0.35
         -0.20  1.20 ]
plant: g = [9.81; 9.81]   d = [0.20; 0.35]   b = [1.0; 1.4]   tau = [0.05; 0.08]
```

**Plant integration**, per channel, forward Euler:

```
u_act += (u - u_act) / tau * Ts        // actuator lag
ddy    = -g - d * dy + b * u_act       // gravity + drag + input
dy    += ddy * Ts
y     += dy  * Ts
```

The plants are physically **decoupled** — any cross-channel coupling in
`mimo_cross` comes only from $\boldsymbol\alpha$, inside the estimator and the
command law. That is deliberate: it isolates exactly the thing being ported.

**Per-tick loop order**, which matters because the unit delay is part of the
contract:

1. `u_prev = u[k-1]` (the loop's explicit unit delay)
2. reference smoother → `sp_filt`, `ddot_sp`; then `err = y - sp_filt`
3. estimator: driven by `y` (decoupled), with `u_prev` and the matrix `alpha`
4. feedback: `fb = Kd*dot_err + Kp*err + Ki*int_err`, per channel
5. command: `ff = ddot_sp`; `u_raw = alpha \ (-F_hat + ff - fb)`; `u = u_raw`
   (no output filter, no saturation in these traces)

**Trace format**, `[N x (5n+1)]` with `n = 2`:

```
u1, u2, F_hat1, F_hat2, sp_filt1, sp_filt2, err1, err2, u_raw1, u_raw2, valid
```

and the SISO comparison trace is `u, F_hat, sp_filt, err, u_raw, valid`.

## Verification ladder

Climb it in order. Do not skip to SITL.

1. **`mimo_diag` self-consistency.** Reproduce `tests/golden/mimo_diag.csv`.
2. **The reduction check — the one that actually proves the port.** With
   `alpha = eye(2)` and identical channels, the matrix solve and the
   `alpha*d2u` product must degenerate to the scalar path, so channel 1 and
   channel 2 of `mimo_diag` must each reproduce the `u` column of
   `tests/golden/2nd_decoupled_alg.csv`. This anchors the port against a trace
   that predates the MIMO code entirely. Use a **tolerance, not bit-exactness**
   — the matrix solve takes a different if mathematically equivalent path than
   the scalar code, and demanding bit-equality here produces a false failure.
3. **`mimo_cross`.** Off-diagonal gain, different plants. This is what pins the
   per-element numerator against the shared scalar denominator; `mimo_diag`
   alone cannot distinguish them.
4. **Host harness against the firmware loop**, same technique the earlier HEOL
   library work used — stub the two dependencies and run inside
   `paparazzi-build:latest`. Check $\boldsymbol\alpha_{xy}$ and
   $\Delta\mathbf u$ per tick.
5. **SITL, last.**

## On the SITL step: there is no baseline

The tracking-error numbers in
`Knowledge/Sessions/2026-08-14-heol-guidance-fixes.md` cover a 1.5 s window and
are **not** a stability baseline — longer interactive runs on that same build
diverge into sustained horizontal oscillation.

So fly long enough to see settling behaviour, not just the trajectory pass, and
record that as the first real baseline for this variant.

Note what this task is replacing: the oscillating loop itself. Two outcomes,
both informative — if the oscillation **goes away**, the scalar-gain
factorization was implicated; if it **persists**, the problem is not in
horizontal guidance and the search moves to the attitude loop and the allocator.
Record which, either way. That is the most valuable single output of the SITL
step, more than any tracking number.

## Airframe changes

In the `GUIDANCE_HEOL` section of
`conf/airframes/ENAC/quadrotor/anton_heol.xml`: retire `GX_ALPHA`/`GY_ALPHA`
(the horizontal input gain is now geometric, not tuned) and collapse the two
per-axis estimator windows into one shared window, matching the single shared
denominator.

## Invariant to preserve

Add a `heol_mimo` wrapper mirroring `heol.h` for the 2-vector case, and keep the
invariant that file exists to protect: the embedded loop is run **only** from
the wrapper, and driven **only** with the tracking residual `epsilon` — never
the raw setpoint, never the total command. Breaking it silently converts HEOL
back into an ordinary setpoint tracker.

This is settled, not merely conventional: the estimator's delayed-command input
is the correction $\delta\mathbf u$ only, taken before the nominal input is
added and before saturation. The specification's realization figure draws it
otherwise and is known to be wrong on that point.
