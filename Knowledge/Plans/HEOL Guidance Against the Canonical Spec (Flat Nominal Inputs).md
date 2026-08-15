# HEOL Guidance Against the Canonical Spec (Flat Nominal Inputs)

Plan for bringing the guidance-side HEOL loops in line with the canonical
controller definition, using the flat-map nominal inputs now carried in
`sw/airborne/modules/nav/flat_traj_demo_data.h`.

Scope is the **outer loop only** — the planar $x$–$y$ and vertical $z$ position
channels. The roll/pitch MIMO and yaw attitude channels are separate later work
and are not touched here.

## Provenance, and how much to trust this file

The specification lives outside this repo, at
`Documents/ENAC-report/canonical/heol-quad-controller.tex` in the thesis vault,
which is **not mounted in this repo's sandbox**. That is why the load-bearing
equations are transcribed below rather than referenced.

Two caveats on that transcription:

- The `.tex` is the source of truth. Where it and this file disagree, it wins,
  and the disagreement is a bug in this file — **with one recorded exception**,
  the estimator's delayed-command lane, where the figure is known to be wrong
  and a correction is pending. See D2 and open question 1.
- It currently has **no `.md` sibling**, so the canon format's `status:` field
  (`draft` vs `confirmed`) is not recorded for it. Treat it as unsettled: the
  author has not signed it off. Do not treat anything below as a licence to
  change control structure without asking.

## The specification, transcribed

Ultra-local model, tracking error, and the per-channel law:

$$\mathbf y^{(\nu)}(t) = \mathbf F(t) + \boldsymbol\alpha(t)\,\mathbf u(t)$$
$$\mathbf e[k] = \mathbf y[k] - \mathbf y^\star[k]$$
$$\widehat{\mathbf F}[k] = \mathcal E\!\left(\mathbf y, \mathbf e, \mathbf u[k-1], \boldsymbol\alpha, t\right)$$
$$\mathbf f_b[k] = \mathrm{PD}(z)\,\mathbf e[k]$$
$$\delta\mathbf u[k] = \boldsymbol\alpha[k]^{-1}\left(-\widehat{\mathbf F}[k] + \mathbf f_f[k] - \mathbf f_b[k]\right)$$
$$\mathbf u[k] = \mathrm{sat}\!\left(\mathbf u^\star[k] + \delta\mathbf u[k]\right)$$

Channel specializations for the two loops in scope:

| block | $\mathbf y$ | $\mathbf u$ | $\boldsymbol\alpha$ |
|---|---|---|---|
| Planar HEOL position controller | $[x, y]^\mathsf{T}$ | $[\phi, \theta]^\mathsf{T}$ | $\boldsymbol\alpha_{xy}$ |
| Vertical HEOL position controller | $z$ | $T$ | $\alpha_z$ |

The input sensitivities are the translational-acceleration Jacobian linearized
about the reference — **not** tuned constants:

$$\boldsymbol\alpha_{xy} = \frac{\partial(\ddot x, \ddot y)}{\partial(\phi,\theta)} = \frac{T}{m}\begin{bmatrix} s_\psi c_\phi - c_\psi s_\theta s_\phi & c_\psi c_\theta c_\phi \\ -c_\psi c_\phi - s_\psi s_\theta s_\phi & s_\psi c_\theta c_\phi \end{bmatrix}$$

$$\alpha_z = \frac{\partial \ddot z}{\partial T} = \frac{c_\theta c_\phi}{m}$$

Critically, and stated explicitly in the spec's realization diagram: the
sensitivity is scheduled from the **reference**, $\boldsymbol\alpha[k] =
\boldsymbol\alpha(\mathbf y^\star[k])$, evaluated at $(T^{ref}, \phi^{ref},
\theta^{ref}, \psi^{ref})$. Never from the measured state.

## What the architecture figure requires structurally

The spec's architecture diagram is not just a picture of the equations — it
dictates where the sensitivity is computed, and that shapes this task.

**The Input-Sensitivity Transformation is a single shared block.** It takes
$(T^{ref}, \phi^{ref}, \theta^{ref}, \psi^{ref})$ — four reference quantities,
nothing measured — and emits **both** $\boldsymbol\alpha_{xy}$ and $\alpha_z$,
fanning them out to the planar and vertical controllers respectively. It is not
per-channel code that happens to compute similar trig twice.

That is the single most important structural point for this task, because it
determines what to build now versus later:

- Everything the block consumes is already being plumbed by this task. There is
  no additional data dependency.
- **Build the block whole, emitting both outputs, even though only $\alpha_z$
  has a consumer this stage.** Wire $\alpha_z$ into the vertical channel;
  compute $\boldsymbol\alpha_{xy}$ and publish it to telemetry only.
- If instead $\alpha_z$ gets inlined as `cosf(theta)*cosf(phi)/m` inside
  `guidance_heol_vert()`, the MIMO stage has to refactor it back out — and the
  refactor lands in the same commit as the control-structure change, which is
  the worst place for it.

Publishing $\boldsymbol\alpha_{xy}$ to telemetry before it is load-bearing also
means the convention question below gets settled against real logged numbers
while it is still harmless.

Two further structural facts from the figure, for orientation:

- The vertical channel's output $T^{sp}$ goes **straight to control
  allocation**. It does not pass through any attitude conversion.
- The planar channel outputs $\phi^{sp}, \theta^{sp}$ **directly** — because
  $\boldsymbol\alpha_{xy}^{-1}$ *is* the acceleration-to-attitude inversion.
  There is no `accel_to_att_sp()` in the specified architecture; that function
  is standing in for the $\boldsymbol\alpha_{xy}$ inversion and the MIMO stage
  deletes it. Do not entangle further with it here.
- Yaw is an **independent SISO loop**, not cascaded from either position
  channel. It takes $\psi^{ref}$ and $M_z^{ref}$ directly.

## The sensitivity values, computed against the real trajectory

All figures below are computed from the 751 rows of
`sw/airborne/modules/nav/flat_traj_demo_data.h` with $m = 0.8$.

### $\alpha_z$ — safe, and 5–6× off today

$\alpha_z = c_\theta c_\phi/m$ ranges **1.0097 … 1.2500** over the trajectory
(1.2500 at level attitude). The constant it replaces, `GZ_ALPHA = 5/m = 6.25`,
is therefore **5.00× to 6.19× too large**, and since $\alpha$ enters inverted
that is a 5–6× increase in correction authority. This is the D3 warning, now
quantified across the whole trajectory rather than at a single point.

### $\boldsymbol\alpha_{xy}$ — anti-diagonal and well conditioned

At level attitude with $\psi = 0$, and with this repo's negative-thrust
convention:

$$\boldsymbol\alpha_{xy} = \begin{bmatrix} 0 & -9.81 \\ +9.81 & 0\end{bmatrix}$$

**Anti-diagonal.** $\ddot x$ responds to $\theta$, $\ddot y$ responds to $\phi$,
and the diagonal entries are zero. The current firmware's two independent scalar
gains (`GX_ALPHA = GY_ALPHA = 15/m = 18.75`) followed by `accel_to_att_sp()` is
an ad-hoc factorization of exactly this matrix — which is why the attempt to
"diagonalize alpha" was abandoned as a wrong premise. As $\psi$ rotates through
45° over the trajectory the matrix rotates with it, and the diagonal entries
become nonzero.

Sign warning: as with $\alpha_z$, the whole matrix flips sign with the thrust
convention. The values above use $T = $ `thrust_ref` $< 0$ (this repo). The spec
derives with $T > 0$ and would give the transpose-sign matrix. Treat
$\boldsymbol\alpha_{xy}$ and $\alpha_z$ under **one** consistent convention —
mixing them inverts one loop and not the other.

Invertibility, which matters because the MIMO law solves against this matrix:
$|\det|$ ranges **51.5 … 179.7** and the condition number **1.000 … 1.245** over
the trajectory. Essentially perfectly conditioned, no near-singular point. A
closed-form $2\times2$ inverse is safe; no pseudo-inverse or regularization is
warranted.

## Symbol → identifier map

| spec | table field | firmware |
|---|---|---|
| $T^{ref}$ | `thrust_ref` | new; today `z_ff` is computed, see D4 |
| $\phi^{ref}, \theta^{ref}$ | `phi_ref`, `theta_ref` | new |
| $\psi^{ref}$ | from `heading` (offset) + trigger heading | `gh->sp.heading` |
| $\mathbf u^\star$ (vertical) | `thrust_ref` | `heol_gz.u_ff` |
| $\mathbf u^\star$ (planar) | `phi_ref`, `theta_ref` | not used until the MIMO work |
| $\alpha_z$ | derived | `heol_gz.mfc.alpha`, constant `GUIDANCE_HEOL_GZ_ALPHA` today |
| $\mathbf e$ | — | `heol->epsilon` |
| $\delta\mathbf u$ | — | `heol->mfc.command[0]` |
| $\mathbf u$ | — | `heol->command` |

## Sign convention — read this before touching $\alpha_z$

The spec writes the vertical equation of motion as $m\ddot z = mg - c_\theta
c_\phi T$, i.e. **$T$ positive**, thrust reducing $\ddot z$ in a $z$-down frame.
At hover that gives $T = +mg$.

This firmware, and the generated table, use the opposite sign: `thrust_ref` is
$-7.848\,\mathrm{N}$ at hover, which is $-mg$ for $m = 0.8$. So here the
equation of motion is $m\ddot z = mg + c_\theta c_\phi T$, and therefore

$$\alpha_z^{\text{(this repo)}} = +\frac{c_\theta c_\phi}{m}$$

**positive**, which is consistent with the existing positive `GZ_ALPHA` and with
the existing `z_ff = MASS * (zdd_ref - 9.81)`. Keep the sign positive.

Separately, the `.tex` carries an author's note flagging that its own
$\delta T$ coefficient in the full sensitivity matrix ($-c_\theta c_\phi/m$)
disagrees in sign with its $\alpha_z$ ($+c_\theta c_\phi/m$), left unreconciled
pending a decision on the report's $z$-axis convention. **Do not treat the
paragraph above as resolving that.** It resolves only what this firmware should
do under its own established convention; the report's internal consistency is
the author's question and is listed as open at the bottom of this file.

## Discrepancies between spec, Simulink, and this firmware

Five, in descending order of how much they matter for this task.

### D1 — Saturation is on the wrong quantity, and the recent feedforward change made that matter

Spec: $\mathbf u[k] = \mathrm{sat}(\mathbf u^\star[k] + \delta\mathbf u[k])$ —
the clamp is on the **total** command.

Firmware: `mfc_core.c` clamps $\delta u$ against `u_min`/`u_max` (and freezes
the integral in the same sample), then `heol.c` adds `u_ff` *afterwards* with no
further clamp. The total is unbounded by construction.

The existing code compensates by pre-offsetting the vertical clamp — the comment
in `guidance_heol.c` says as much, that `u_fb`'s clamp is offset by the hover
`u_ff` so the sum still respects the physical range. **That approximation was
valid only while `u_ff` was a constant.** It no longer is: the earlier fix
turned on `FLAT_TRAJ_DEMO_ORDER 4`, so `u_ff` now varies over the trajectory,
and a fixed offset no longer bounds the sum correctly.

**Fix this here.** Move the clamp to the total command in `heol_run()`, and drop
the hover offset from the gz limits so they express the real physical range
again. Keep the integral-freeze behaviour tied to whichever clamp actually
bites. Note the interaction: the anti-windup freeze currently keys off the
$\delta u$ clamp, so moving the clamp means moving what the freeze observes.

### D2 — What the estimator sees as $\mathbf u[k-1]$: settled, it is $\delta u$

The spec's realization diagram taps the delayed command **after** the summing
junction and after the saturation, making $\mathbf u[k-1]$ the total applied
command. **That is wrong, and is an error in the figure.** Author's decision,
2026-08-14: the estimator's delayed-command input is the **correction only**,
$\delta\mathbf u[k-1]$ — taken before the nominal input is added and before the
saturation.

So the existing code is already correct and needs no change here. This firmware
feeds `command[1]`, the correction; `heol.h` states that as a required
invariant, that the estimator must see only `u_fb` or the structure silently
degrades into an ordinary setpoint tracker. **Leave that invariant intact** —
and note it now has authority behind it, not just precedent. The Simulink model
matches: a `Delay`/`Memory` on the pre-saturation $\delta u$, while its
saturation sits on the total.

Note this is *not* in tension with D1. The two taps are deliberately different
points: saturation acts on $\mathbf u^\star + \delta\mathbf u$, while the
estimator is fed $\delta\mathbf u$ from before both the sum and the clamp. That
combination is exactly the Simulink structure.

Two consequences worth being deliberate about, both intended:

- **The estimator is blind to $\mathbf u^\star$.** $\widehat{\mathbf F}$ is the
  lumped dynamics of the *tracking error* about the flatness nominal, not of the
  plant. This is what makes the architecture HEOL rather than plain MFC, and it
  is why the estimator is driven by $\mathbf e$ rather than $\mathbf y$.
- **The estimator is blind to the saturation.** When the total command clamps,
  the plant does not respond as $\alpha\,\delta u$ predicts, and
  $\widehat{\mathbf F}$ absorbs the shortfall as though it were a disturbance.
  With the loops running PD-only (`KI = 0`) there is no integrator to wind up,
  so this is bounded — but it is the reason the clamp in D1 needs to be applied
  where the physical limit actually is, rather than approximated by an offset.

The one thing to fix is the spec figure, which is an author edit to the canon
item and not something to change from inside this repo — listed as open below.

### D3 — $\alpha_z$ as a design parameter vs. the true Jacobian: a 5× loop-gain change

Today `GZ_ALPHA = 5. / MODEL_MASS` $= 6.25$. The spec's $\alpha_z = c_\theta
c_\phi/m$ evaluates over this trajectory to a range of **1.0097 … 1.2500**
(computed from the generated table; $1.25$ at level).

Since $\alpha$ enters as $\alpha^{-1}$ multiplying the whole correction,
swapping the constant for the Jacobian multiplies the correction by **5×** at
level attitude. That is not a refinement, it is a substantial loop-gain
increase — and it lands on a variant whose horizontal loops already show
sustained oscillation in longer interactive runs, with a gain set that was
itself only just re-derived.

**Stage it.** Land the plumbing first with `GZ_ALPHA` still constant and verify
the nominal inputs arrive correctly. Then flip $\alpha_z$ to the live Jacobian
as a separate, individually revertible commit, and expect to retune the vertical
PD alongside it. Treat "$\alpha$ live" and "PD gains" as one coupled change, not
two independent ones — this is the same class of mistake as reconciling a gain
convention without re-verifying the closed loop.

### D4 — The vertical feedforward is a small-angle approximation; the table's is exact

Today `z_ff = MASS * (zdd_ref - 9.81)` — the $z$ component of the required
force. The spec's $\mathbf u^\star$ for the vertical channel is $T^{ref}$, the
**collective** thrust along body $-z$, which the flat map computes exactly.

These agree only at zero tilt. Checked against the generated table:

| | `thrust_ref` | `m*(az-g)` | error | tilt |
|---|---|---|---|---|
| sample 0 | −7.8480 N | −7.8480 N | 0.00 % | 0.00° |
| sample 543 | −6.4086 N | −5.1767 N | **19.22 %** | 36.19° |

A ~19 % feedforward error at the most aggressive point of the maneuver, which
the feedback loop is currently absorbing silently.

**Fix.** Source $u^\star_z$ from `thrust_ref` and delete the computed
approximation. It needs no unit conversion — the table's sign and units already
match this repo's convention.

### D5 — $\mathbf f_f$ inside the inversion is unused; leave it that way

The spec's correction law carries an $\mathbf f_f[k]$ term *inside* the
$\alpha^{-1}(\cdot)$ inversion, distinct from the nominal input $\mathbf u^\star$
that is summed outside it. Both existing implementations ground it — Simulink
literally, and here `dot_dot_setpoint_trajec` collapses to zero because
`heol_init()` pins the setpoint at zero.

Leave it grounded. Do not invent a value for it. Noted only so it is not
mistaken for a missing feature during review.

## Implementation order

1. **Decide the plumbing route and write down why.** The nominal inputs are not
   derivatives and must not ride the Taylor extrapolation in
   `sw/airborne/firmwares/rotorcraft/guidance/guidance_h_ref.c` /
   `guidance_v_ref.c`, which propagates pos/vel/accel from held jerk and snap.
   Extrapolating $T^{ref}$ or $\phi^{ref}$ that way is meaningless. Preferred:
   extend the flat-ref struct with a nominal-input group that is held or
   linearly interpolated per tick. Alternative: route them from
   `nav_flat_traj` straight to `guidance_heol`, at the cost of a second
   independent path for trajectory data.
2. **Consume the new fields** in `sw/airborne/modules/nav/nav_flat_traj.c`. The
   struct now comes from the generated header — if a field is wrong, fix the
   generator in the sim repo, not the output. `FLAT_TRAJ_DEMO_ORDER` gates only
   the kinematic derivatives; it must not gate the reference columns.
3. **D4:** replace the computed `z_ff` with `thrust_ref`.
4. **D1:** move saturation to the total command; drop the hover offset from the
   gz limits; re-point the integral freeze.
5. **Add a per-tick $\alpha$ setter to `mfc_core`** — the MIMO work needs it
   too, so build it as a clean interface rather than a special case.
6. **D3, separate commit:** $\alpha_z \leftarrow c_\theta c_\phi / m$ from
   *reference* attitude, with the vertical PD retuned alongside.
7. **Retire `filt_thrust`.** Once $T^{ref}$ is the source, the pinning of
   `heol_thrust_physical` to nominal hover throttle can go. That pinning was
   deliberate — using the commanded thrust is singular as it approaches zero —
   and $T^{ref}$ preserves the guarantee properly, since a reference thrust is
   never zero on a sane trajectory. The filter then has no consumer; remove it
   rather than leaving it computed and discarded.
8. **Check the handoff step** seen in earlier SITL: `sp_traj_z` briefly steps to
   −4.7105 where the flight plan leaves `Flat_Traj_Demo` for `Standby`. It
   reproduces with the feedforward disabled, so it is pre-existing; the
   suspicion is `guidance_heol_vert()` being called unconditionally for
   `guidance_v_run_pos()`, i.e. for altitude hold as well as the trajectory
   path. This task is in that code — confirm or dismiss it.

## Out of scope

$\boldsymbol\alpha_{xy}$ and the planar MIMO restructure; the roll/pitch and yaw
attitude channels (still `stabilization type="mfc"`); control allocation. Step 5
exists to make the MIMO work cheap, not to start it.

## Verification

- Log $T^{ref}, \phi^{ref}, \theta^{ref}$ and confirm they match the sim repo's
  `thrust_ref`/`phi_ref`/`theta_ref` on the same trajectory. Both sides now read
  the same generated table, so a mismatch is a plumbing bug, not a modelling
  difference.
- After step 3, confirm the vertical feedforward diverges from the old
  `m*(az-g)` exactly where the table says it should — ~19 % near sample 543,
  zero at sample 0. If it does not, the wrong field is being read.
- After step 4, drive the vertical channel into saturation deliberately and
  confirm the *total* command is what clamps, and that the integral freezes.
- After step 6, $\alpha_z$ should vary with reference tilt across roughly
  1.01–1.25, not sit constant.
- Do **not** use the tracking-error table in
  `Knowledge/Sessions/2026-08-14-heol-guidance-fixes.md` as a pass/fail bar. It
  covers a 1.5 s window and is not a stability baseline; longer runs oscillate.
  Judge this task on whether the nominal inputs arrive correctly and the
  saturation behaves, not on closed-loop performance.

## Open questions for the author — do not resolve unilaterally

1. **Canon fix pending (D2).** The realization figure in
   `heol-quad-controller.tex` draws the estimator's delayed-command lane from
   after the summing junction and saturation. It should come from
   $\delta\mathbf u$, before both. Decided 2026-08-14; the figure has not been
   corrected yet. Until it is, the figure and this plan disagree, and **this
   plan is right on that one point** — the reverse of the precedence rule stated
   at the top of this file.
2. The $z$-sign note the `.tex` raises against itself, pending the report's
   $z$-axis convention. Independent of what this firmware does internally.
3. Whether $\mathbf f_f$ is intended to stay grounded permanently, or is a
   placeholder for something not yet built.
