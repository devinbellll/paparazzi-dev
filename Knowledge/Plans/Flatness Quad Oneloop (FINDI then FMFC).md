# Flatness Quad Oneloop (FINDI then FMFC)

Plan for bringing the two **flatness-based quadrotor controllers** into this
repo as a *oneloop* module: one tick that runs position feedback, the exact
flatness inversion, the attitude loop and the allocation with no
guidance→stabilization seam in the middle.

Two variants, staged: `findi_quad` first (no estimator), then `fmfc_quad` on
the verified spine.

## Why this document exists

The controller specification lives in the thesis vault
(`Documents/ENAC-report/canonical/`) and the MATLAB reference implementation
lives in the `Generic_Quad` sim repo. **Neither is mounted in this repo.**
Everything needed is therefore transcribed here: the equations, the algorithms
including their guard branches, the sign conventions, the parameter bindings,
and the four questions you must not answer on your own.

The specification is author-written and both items are `status: draft` — i.e.
visible and usable, but not yet signed off. Where this plan and the code
disagree, raise it; do not "fix" the spec by inference.

What crosses the boundary as data: reference CSVs under `tests/golden/`,
copied in from the root tier (see *Reference traces*). They are authoritative
outputs of the sim repo. A mismatch means the port is wrong, not the trace.

## Why a oneloop, and not guidance + stabilization

This is the whole point of the architecture, so get it right before writing
any code.

In both flatness variants the commanded inertial force $f_i^c$ and the yaw
reference $\psi^{ref}$ are inverted **together** into a collective body-z force
$f$ *and* a full commanded attitude $q^{cmd}$. One transform, one instant, two
outputs that are two halves of the same answer.

Paparazzi's normal path forces that answer through a `StabilizationSetpoint` +
`ThrustSetpoint` pair handed from the guidance layer to the stabilizer. That
puts a decoupling seam exactly where the physics couples: the attitude arrives
as Euler angles (re-parameterised, and in the wrong sequence — see *ZXY*
below), the force arrives separately and scaled differently, and the exact
inversion gets replaced by the approximate small-angle one.

So: **one module owns position → flatness transform → attitude → allocation**,
and passes the commanded *quaternion* internally. `accel_to_att_sp()` has no
counterpart in this architecture — the flatness transform *is* the inversion.
This is the same argument `heol_mimo.h`'s header already makes for the 2×2
case, applied to the full 6-DOF one.

The existing `oneloop_mfc.c` already establishes the packaging pattern: under
`#ifndef ONELOOP_MFC_NOT_STANDALONE` (`oneloop_mfc.c:1565-1671`) it defines
`stabilization_attitude_enter()`, `guidance_h_run_*()` and `guidance_v_run_*()`
itself. Follow that shape. Do **not** restructure `oneloop_mfc.c` — it stays
flying as the channel-wise MFC reference; the new work is a new module.

## Notation and conventions (fix these first)

| symbol | meaning | firmware source |
|---|---|---|
| $p, v$ | NED position, velocity | `stateGetPositionNed_f()`, `stateGetSpeedNed_f()` |
| $\tilde a$ | filtered, gravity-compensated inertial accel | see eq. (A) |
| $R_{be}$ | NED → body DCM; $R_{eb} = R_{be}^\top$ | `stateGetNedToBodyRMat_f()` |
| $q$ | measured attitude, **scalar first**, NED → body | `stateGetNedToBodyQuat_f()` |
| $f_i^c$ | commanded inertial force [N] | — |
| $f$ | collective body-z force [N], **negative in hover** | — |
| $m_c$ | commanded body moment [N·m] | — |
| $\omega$ | body rates | `stateGetBodyRates_f()` |
| $G_1$ | effectiveness, rows `[Mx; My; Mz; Fz]` | airframe XML — **see Q1** |

Three sign/convention traps, each of which has cost a session somewhere in
this workspace:

1. **The two variants use opposite error senses.** FINDI forms
   *reference − measurement*; FMFC's linear loop forms *measurement −
   reference* (the MFC convention), while its angular cascade forms *command −
   measurement*. This is recorded as wired in both canon items, not as a
   typo. Getting it backwards inverts the feedback and the loop diverges.
2. **The flatness map is ZXY (3-1-2)**; the plant and most of Paparazzi
   integrate ZYX. That is exactly why the attitude error is carried as a
   **quaternion**, not an Euler difference. Never convert the commanded
   attitude to Euler angles and difference it against a measured Euler triple.
   `float_quat_of_eulers_yxz()` exists in this tree and is **not** the right
   tool here — the transform emits a quaternion directly, use it.
3. **$f$ is negative in hover** (NED, the vehicle pushes along $-z_b$). The
   allocator's fourth row is $F_z$, not thrust magnitude.

## Reference traces

Copied into `tests/golden/` from the root tier:

```
tests/golden/flat_force.csv           quad_flatness_force_transform sweep
tests/golden/att_err.csv              quad_indi_attitude sweep
tests/golden/alloc.csv                quad_alloc sweep, real force_G1
tests/golden/findi_quad_closed.csv    one closed-loop findi_quad run
```

The repo already has a golden-test harness — `tests/mimo_golden_test.c`,
`tests/run_mimo_golden.sh`, `tests/stubs/` — built for the MIMO port. Extend
it; do not invent a second fixture convention.

Two rows in each transform sweep matter more than the rest:

- `flat_force.csv` contains a **tilt past 90°**. That row is what catches a
  ZXY/ZYX confusion, and it is where `use_by_branch` would change the answer
  (it is **off**, matching the source default — so the expected output there
  is the `cos θ ≥ 0` alias).
- `att_err.csv` contains a **near-zero attitude error** row and a row needing
  the **shortest-path negation**. The first catches a missing `k = 2` limit
  (a `0/0` at the most common operating point, not a rare one); the second
  catches taking the 350° way round a 10° error.

---

# Part 1 — The flatness spine (stage 1)

Three pure functions, no controller state. Build them as
`sw/airborne/firmwares/rotorcraft/stabilization/flatness_quad.{c,h}` and verify
against the CSVs **before** any loop is wired.

## 1.1 Flatness force transform

Transcribed from `Generic_Quad/functions/quad_flatness_force_transform.m`.
Inputs: $f_i^c$ (3×1, N) and $\psi^{ref}$ (rad). Outputs: $f$ (scalar, N),
the ZXY Euler triple, $R$ (body→NED) and $q^{cmd}$ (scalar first, NED→body).

```
tvec = fi_c / m
nt   = norm(tvec)
                                        -- if nt < 1e-6: FREE FALL, see Q2
f  = -m * nt
zb = -tvec / nt

v1 =  cos(psi_ref)*zb[0] + sin(psi_ref)*zb[1]
v2 = -sin(psi_ref)*zb[0] + cos(psi_ref)*zb[1]
v3 =  zb[2]

ct  = hypot(v2, v3)          -- = |cos(theta)|
sgn = 1                      -- the unconditional cos(theta) >= 0 alias
phi = atan2(-v2, v3)
                                        -- if ct < 1e-8: GIMBAL LOCK, see Q2
theta = atan2(v1, sgn*ct)
```

Then rebuild the DCM **explicitly** rather than reusing the extraction, so the
round trip is literal — $R_{bi} = R_y(\theta) R_x(\phi) R_z(\psi)$, passive:

```
R_b_i = [ ct*cps - st*sp*sps,   ct*sps + st*sp*cps,  -st*cp
         -cp*sps,               cp*cps,               sp
          st*cps + ct*sp*sps,   st*sps - ct*sp*cps,   ct*cp ]

R     = R_b_i^T                          -- body -> NED
euler = [phi, theta, psi_ref]
q_cmd = float_quat_of_rmat(R_b_i)        -- NED -> body, scalar first
```

`use_by_branch` (the paper's $k\pi$ continuity branch, needing the previously
commanded body-y) is **off** in both quad models and is **out of scope here**.
Leave a documented hook, implement nothing. It only matters for trajectories
whose $\theta$ winds past 90°, which no firmware trajectory currently does.

Why the unconditional alias is safe under ZXY, transcribed because it is a
real result and not obvious: under ZXY the $\cos\theta \ge 0$ alias *is* the
$\psi$-preserving one at every tilt — $\phi$ runs the full circle and takes
the sign of the flip, so $\psi^{ref}$ comes back exactly and the triple is
continuous through the inversion. That is **not** true of the ZYX version this
replaced, which needed a $k = \mathrm{sign}(v_3)$ branch. Do not "improve" it
back into a branch.

## 1.2 Quaternion attitude error

Transcribed from `Generic_Quad/functions/quad_indi_attitude.m`. Inputs
$q^{cmd}$, $q$ (both unit, scalar first, NED→body); output $\zeta_e$ (3×1).

```
qi = conj(q)                              -- unit, so inverse == conjugate
qe = qi (Hamilton-product) q_cmd          -- scalar first
qe = qe / |qe|                            -- renormalize if |qe| > 0
if qe[0] < 0: qe = -qe                    -- shortest path
qw = clamp(qe[0], -1, 1)
k  = (1 - |qw|) < zeta_tol ? 2 : 2*acos(qw)/sqrt(1 - qw^2)
zeta_e = k * qe[1:3]
```

`zeta_tol = 1e-8`. The `k = 2` branch is the exact limit of the expression at
$q_w = 1$; it is the hover operating point, not an edge case.

## 1.3 Allocation

Transcribed from `Generic_Quad/functions/quad_alloc.m`:

$$ u = G_1^{-1}\begin{bmatrix} m_c \\ f \end{bmatrix},\qquad
   \omega^{sp} = \texttt{MAX\_PPRZ}\cdot \mathrm{sat}_{[0,1]}(u) $$

$G_1$ is **square (4×4) and well conditioned** for the standard X-quad mix, so
this is an *exact inverse*, not a pseudo-inverse and not WLS — there is no
redundancy to distribute and no active set to solve. `quad_alloc` itself does
not saturate: it solves and reports feasibility, and the clamp is a separate
downstream stage (`u_sat`). Keep that split; it is what makes an infeasible
wrench visible instead of silently absorbed.

This differs from `oneloop_mfc.c`'s allocator (WLS or `g1g2_pseudo_inv` over
`MFC_OUTPUTS` in pprz-scaled units). **Do not reuse that path.** See Q1 before
writing a line of it.

---

# Part 2 — `oneloop_findi` (stage 2)

The FINDI oneloop. No ultra-local model and no $\widehat F$ estimator — that is
the one architectural break from the MFC family. The lumped disturbance is
rejected incrementally: an acceleration increment becomes a wrench increment
through the nominal inertia, added to the wrench the actuators are currently
applying.

Per-tick order, exactly:

**(A) Acceleration feedback signal**

$$ \tilde a = H(z)\Bigl(R_{eb}\,a_{acc} + [0,0,g]^\top\Bigr) $$

$H(z)$ is the shared Butterworth. See **Q4** on which accelerometer signal and
filter to use — this repo already has `stateGetAccelBody_f()` /
`stateGetAccelNed_f()` and Butterworth filters instantiated in
`oneloop_mfc.c:787-805`.

**(B) Applied reactions measurement**

$$ \begin{bmatrix} m^{prev} \\ F_z^{prev}\end{bmatrix} = G_1 \cdot H(z)\,u_{meas},
   \qquad f_i^{prev} = R_{be}\,[0,0,F_z^{prev}]^\top $$

$m^{prev}$ is rows 1–3, $F_z^{prev}$ row 4. $u_{meas}$ is the measured actuator
state — this repo already reconstructs it in `get_actuator_state()`
(`oneloop_mfc.c:974`), with RPM feedback when available and a first-order
actuator model otherwise. Reuse that mechanism.

Note $G_1$ appears **twice** in this controller: here, reconstructing the
applied wrench, and in the allocation inverse. Same matrix both times.

**(C) Outer PD cascade** — errors are *reference − measurement*:

$$ e_p = p^{ref} - p,\quad e_v = v^{ref} - v,\quad e_a = a^{ref} - \tilde a $$
$$ a_{fb} = R_{eb} R_{be}\bigl(k_x^2 e_p + 2\zeta_x k_x e_v + k_a e_a\bigr),
   \qquad a_c = a^{ref} + a_{fb} $$

Each gain is a scaled identity. $R_{eb}R_{be} = I$ — the two rotations compose
to the identity, recorded as wired in the Simulink source and **not**
simplified there. In C, write the product out or write the identity, but say in
a comment which you did and why, so the next reader does not think a rotation
was lost.

**(D) Linear increment**

$$ f_i^c = f_i^{prev} + m\,(a_c - \tilde a) $$

**(E) Flatness force transform** (§1.1) → $f$, $q^{cmd}$.
**(F) Attitude error** (§1.2) → $\zeta_e$.

**(G) Attitude and rate PD**

$$ \omega_c = k_{att}\,\zeta_e + \omega^{ref},\qquad
   \dot\omega_c = k_{rate}(\omega_c - \omega) + \dot\omega^{ref} $$

$\omega^{ref}$ and $\dot\omega^{ref}$ come from `guidance_flat_nominal.{c,h}`,
which **already latches the complete flat-map nominal set** onboard —
`thrust_ref`, `phi/theta_ref`, `p/q/r_ref`, `pdot/qdot/rdot_ref`,
`Mx/My/Mz_ref` — from the generated trajectory header, in one timestamped
zero-order-hold struct compiled by `guidance_rotorcraft`, deliberately
variant-agnostic. Nothing consumed the rates or moments before this task. They
were verified against the source table when they landed (`7f8fe7c97`).

One recorded defect in the reference to **not** reproduce: the sim's rate loop
port is named `pqr_lpf` but is wired to the *raw* `pqr_meas`. Only the angular
*acceleration* path is filtered. Use the raw body rate here and comment that
the name in the model is wrong, not the wiring.

**(H) Angular increment**

$$ m_c = m^{prev} + I\,(\dot\omega_c - \dot\omega_{lpf}) $$

with $I = \mathrm{diag}(\texttt{airframe.I})$ and $\dot\omega_{lpf}$ the
filtered discrete derivative of the measured body rate —
`oneloop_mfc.c:796-798` already computes exactly this as
`angular_acceleration[]`.

**(I) Allocation** (§1.3) → motor commands.

## Wiring

- New `sw/airborne/firmwares/rotorcraft/oneloop/oneloop_findi.{c,h}`.
- New `conf/modules/oneloop_findi.xml` with the settings panel — mirror
  `conf/modules/oneloop_mfc.xml` for structure, not for content.
- New airframe `conf/airframes/ENAC/quadrotor/anton_findi.xml`, registered in
  `conf_enac.xml`. Start from `anton_oneloop.xml` / `anton_mfc.xml`.
- Gains start at the Simulink values: `kx`, `zeta_x`, `ka`, `k_att`, `k_rate`.
  **None of these loops has a flight-tested baseline.** Do not present a gain
  set as tuned.

---

# Part 3 — FMFC (stages 3–4)

`fmfc_quad` keeps the whole spine of Part 2 — applied reactions, flatness
transform, attitude error, allocation — and replaces the two *increments*
(D) and (H) with MIMO HEOL/MFC brackets.

## 3.1 `mfc_core_mimo` must become 3-vector (stage 3)

`MFC_MIMO_N` is hard-coded to **2** in `mfc_core_mimo.h:82`. Both FMFC
brackets are 3-vector. Generalize the existing core to a compile-time `N`;
do **not** add a second copy — the MIMO estimator's whole structural point is
that *the numerator is per-element while the denominator is scalar and
shared*, and two transcriptions of that will drift.

Regression for this stage: the existing `tests/golden/mimo_diag.csv` and
`mimo_cross.csv` must still reproduce **bit-identically** through the
2-vector callers (`heol_mimo`, the horizontal channel). If they move at all,
the generalization changed behaviour and is wrong.

## 3.2 The two brackets (stage 4)

Both use the decoupled algebraic estimator with a **live matrix $\alpha$**,
grounded feedforward ($f_f = 0$), and the nominal command added *outside* the
correction. The command law is the scalar division replaced by a linear solve:

$$ \delta u[k] = \alpha[k]^{-1}\bigl(-\widehat F[k] + f_f[k] - f_b[k]\bigr) $$

**Linear bracket.** Errors *measurement − reference* here:

$$ e_p = p - p^{ref},\quad e_v = v - v^{ref},\quad e_a = \tilde a - a^{ref} $$
$$ a_c = R_{eb}R_{be}\bigl(k_x^2 e_p + 2\zeta_x k_x e_v + k_a e_a\bigr) $$
$$ f_i^\star = m\bigl(a^{ref} + [0,0,-g]^\top\bigr),\quad
   \alpha_{fi} = \tfrac{1}{m}I,\quad z_k = e_p[k],\quad
   u^{prev} = f_i^{prev} - f_i^\star $$
$$ f_i^c = f_i^\star + m\bigl(-\widehat F_{fi}[k] - a_c[k]\bigr) $$

**Angular bracket.** Cascade error sense is *command − measurement*:

$$ \omega_c = k_{att}\zeta_e + \omega^{ref},\qquad
   \dot\omega_c = k_{rate}(\omega_c - \omega) $$
$$ m^\star = I\,\dot\omega^{ref},\quad \alpha_m = I^{-1},\quad
   z_k = -H(z)\,\zeta_e[k],\quad u^{prev} = m^{prev} - m^\star $$
$$ m_c = m^\star + I\bigl(-\widehat F_m[k] + \dot\omega_c[k]\bigr) $$

Note the angular cascade here has **no $\dot\omega^{ref}$ feedforward term
inside $\dot\omega_c$** — unlike FINDI's (G), where it is present. The
feedforward enters through $m^\star$ instead. That difference is real; do not
harmonize the two.

**Both $\alpha$ matrices are constant and diagonal.** This is the single most
important difference from the HEOL horizontal channel, where $\alpha$ is the
live acceleration-to-tilt Jacobian. Here the flatness map sits **outside** the
loop, downstream of it, so no Jacobian is inverted inside the estimator and
there is no conditioning or singularity question. Consequences:

- `heol_input_sensitivity.{c,h}` has **nothing to do** in this controller.
- **Do not inherit `HXY_INTEGRATION_WINDOW = 500`**, or any other tuning
  constant, from the HEOL horizontal channel. That channel diverged in SITL on
  its last flown run (see `Knowledge/Sessions/2026-08-15-heol-mimo-port.md`),
  and its window was believed to be an unjustified inheritance. **Correction
  (2026-08-20): it is not.** `heol_quad_params.m` sets 500 deliberately on both
  guidance channels because they are position loops carrying GPS noise, against
  20 on the attitude channels. The estimator parameter that WAS wrong on that
  channel is `est_hold_time`, 0.1 against a reference 0.8. Take the windows from
  the reference rather than re-deriving them.
- **`est_use_presat_command` defaults to the correction's *pre*-saturation
  value now** — `cfede3078` flipped the `mfc_core_mimo` default together with
  `HXY_EST_PRESAT`, `GZ_EST_PRESAT` and both module-XML defines, after the
  module-level define was found shadowing the core default. Set it explicitly
  on every channel you create and say which you chose; do not rely on the
  default being what you expect, which is precisely how that defect hid.

The FMFC estimator windows and holds bind as: linear `FFilter_fi` /
`F_hold_time_fi`; angular, the same pair on its own channel.

## 3.3 The HEOL invariant still holds

Reuse `heol.h`/`heol_mimo.h`'s settled invariant, which is not merely
conventional: **the estimator's delayed-command lane carries the correction
$\delta u$ alone** — taken before the nominal input is added and before
saturation. Breaking it silently converts HEOL back into an ordinary setpoint
tracker. See **Q3** for the one part of this that is *not* settled here.

---

# Questions you must not answer on your own

Route these to the author. Each is a design decision or an unresolved
discrepancy, not a transcription gap.

**Q1 — Units, and which $G_1$.** The canon is SI throughout: $f$ in N, $m_c$ in
N·m, $I$ in kg·m². `quad_alloc`'s own docstring says $u$ comes out "in whatever
units $G_1$ is expressed in", and for the sim's `generateQuad` that is the
Paparazzi actuator command, nominally `0..MAX_PPRZ`. This repo's `oneloop_mfc`
path uses `Bwls` and `MFC_G_SCALING` on a differently scaled effectiveness
matrix. **Establish what the firmware airframe's $G_1$ actually maps, from the
XML, before writing the allocator.** Do not assume the two agree because both
are called G1. The SI-units port (`c0f92cd` and the matching submodule commit)
moved Anton's guidance onto an SI scheme; check whether the effectiveness
matrix came with it.

**Q2 — Free fall and gimbal lock.** The MATLAB transform *raises an error* at
$|f_i^c|/m < 10^{-6}$ ("the trajectory is effectively in free fall and the
attitude is undefined") and, with `use_by_branch` off, at $ct < 10^{-8}$.
Firmware cannot throw. It needs a defined degraded behaviour. Proposal to put
to the author: hold the previous commanded attitude, keep the collective force
at its computed value, set a status flag, and log it — but *do not* pick this
unilaterally, and above all do not silently clamp the denominator, which turns
an undefined attitude into a confident wrong one.

**Q3 — The missing unit delay.** `fmfc-quad-controller.md` records, from the
saved `.slx`, that inside both HEOL blocks the `Memory` on the estimator's
`u_prev` is `Commented = through` — i.e. **there is no unit delay on the
estimator's previous applied command**, unlike every other MFC variant in the
workspace. Three other blocks are bypassed the same way (the estimator-drive
`LPF`, so the drive is the *raw* position error; and `Saturation2`, so no
clamp). The canon note flags the missing `Memory` as "the one worth checking".
State clearly in your implementation which you did, and ask.

**Q4 — Which acceleration signal.** $\tilde a$ is specified as
$H(z)(R_{eb}a_{acc} + [0,0,g]^\top)$ — accelerometer rotated to inertial,
gravity-compensated, Butterworth-filtered. Confirm the firmware's available
signal matches in **frame** (body vs NED), in **sign** (NED down-positive), and
in **filter** (which cutoff, which order) before wiring it. This signal appears
in the error term *and*, in FINDI, in the increment itself, so an error here
shows up twice with different gains.

---

# Verification ladder

Do not skip rungs, and do not treat an early rung as evidence about a later
one. This is not generic advice: the HEOL MIMO port passed rungs 0–4 with a
**bit-exact** result on the decisive check and then failed SITL outright.

0. **Existing MIMO traces still pass** after the stage-3 generalization
   (bit-identical, not "close").
1. **Stage-1 transforms vs `flat_force.csv` / `att_err.csv` / `alloc.csv`**,
   to float precision. The >90°-tilt row and the near-zero-error row are the
   two that actually discriminate.
2. **Closed-loop FINDI vs `findi_quad_closed.csv`**, signal by signal through
   (A)–(I). A disagreement that first appears at (D) is a linear-increment
   bug; one that first appears at (E) is a convention bug.
3. **Compile**: `./pprz.sh build ANTON_FINDI nps` and `./pprz.sh build
   ANTON_FINDI ap`. A produced `.elf` is the only automated correctness check
   this repo has.
4. **SITL, one rung at a time**: conventions and units → rate loop → attitude →
   vertical → horizontal → `Flat_Traj_Demo` trajectory → NPS noise sources back
   on one at a time. Report the no-noise and with-noise result as a **pair**.
   When a rung fails, the fault is between it and the rung below.

Diagnostic modifications follow the repo rule: **restore or promote, no third
option**, before any gain or metric is recorded.

Write a session note to `Knowledge/Sessions/<YYYY-MM-DD>-<topic>.md` per
launch, including which rung each failure was found on and what fixed it —
that is usually worth more than the gains.
