# Kickoff — ANTON_MFC SITL: the position loop, seeded from the working Simulink case

**For:** Claude Code, scoped to `paparazzi_dev`. Scope and verification
contract; the worker owns the implementation plan.

**Tracks:** `ThAIsis/Tasks/tune-mfc-quad-sitl.md`.

**Spun out of** `Generic_Quad/Knowledge/Tasks/kickoff-mfc-quad-firmware-parity.md`
stage 1, which halted 2026-08-17. Its result is
`Generic_Quad/Knowledge/RETRO-2026-08-17-mfc-quad-firmware-form.md` and the
effort `Generic_Quad/Data/2026-08-17_mfc_quad_firmware_form`.

**Cross-repo results are embedded below** because a `paparazzi_dev`-scoped mount
cannot see `Generic_Quad` — the mirror image of the constraint that produced the
parity kickoff. Every Simulink number here was **measured on 2026-08-17** from
`models/controllers/mfc_quad.slx` at `cfg.controller = "MFC"`, `Ts = 0.002 s`,
against ground truth. Where a firmware number appears it is marked with where it
came from, and most of them are **not verified** — see §1.

---

> [!note] Root, 2026-08-17 — placed here from the vault tier, and reconciled
> This kickoff was drafted by the Generic_Quad worker, which **could not see this
> repo**. Two firmware sources it could not read are in *your* repo — read them
> before working the hypotheses below; they already resolve most of §1:
>
> - **`Knowledge/15 - Simulink MFC Quad ↔ ANTON_MFC Firmware Correspondence.md`**
>   (2026-08-17) — **verified** ANTON_MFC values read from `anton_mfc.xml`. It
>   supersedes the "unverified" firmware column in §1. Start there; do not
>   re-derive what it already has. `anton_mfc.xml` stays the final authority.
> - **`Knowledge/Sessions/2026-08-17-anton-mfc-sitl-tuning.md`** — the session that
>   reached rung 2 and raised `TIME_TRAJECTORY` 50→250.
>
> **Two hypothesis corrections from those verified values (both firmware and
> Simulink run at 500 Hz, so the sample-count windows compare directly):**
>
> 1. **H2 is right for z, inverted for horizontal.** Firmware
>    `GX/GY_INTEGRATION_WINDOW = 600` (1.2 s), not the "5" the deleted comments
>    claimed — that is **6× longer** than Simulink's 100 (0.2 s), so the
>    horizontal estimator is *not* memory-starved; if anything it is heavily
>    smoothed. But firmware `GZ_INTEGRATION_WINDOW = 4` (8 ms) against Simulink's
>    50 (100 ms) **is** ~12× shorter — so H2 stands for the **z** channel and is
>    the leading verified explanation for "z noisy and choppy". Split H2 by axis.
> 2. **H5 is partly verified true.** Firmware runs an output EMA where Simulink
>    runs none: `GZ_COMMAND_FILTER = 8`, and since 2026-08-07 attitude
>    `COMMAND_FILTER = 10`; `GX_COMMAND_FILTER = 1` matches Simulink. So z and
>    attitude command bandwidth differ between the sides; horizontal does not.
>
> `alpha` is confirmed (roll/pitch `2/I`, yaw `3/I`, gz `5/m`, gx `15/m`). H1
> (does the estimator see the *saturated* command) is untouched by this and is
> still the first thing to check — it is one line of `mfc_core.c`.

## The inversion this task exists to exploit

**Simulink MFC flies position. SITL does not.** The recent SITL session
(`Knowledge/Sessions/2026-08-17-anton-mfc-sitl-tuning.md`) reached attitude only,
rung 2, and left XY in a large limit cycle and the z command noisy and choppy.
The Simulink loop with the same controller family holds position to 2.5 cm in
hover with every noise source on.

So this is not a tuning problem to search. **There is a working reference, and
the job is to find which structural difference accounts for the gap.** The
parity note's "horizontal is the problem child" was written from this side; it
does not describe the Simulink loop.

## The Simulink baseline, measured

`quad_minsnap` — 1 m N, 1 m E, 1 m climb, yaw 0 → 45°, complete by t ≈ 3.5 s,
hover hold to 10 s. Errors against ground truth.

| case | pos peak | pos rms | pos rms, hover | φ/θ/ψ rms | xy cmd on rail (hover) |
|---|---|---|---|---|---|
| noise off | 0.497 m | 0.158 m | 0.0172 m | 13.3 / 6.76 / 1.46° | 0 % |
| noise on | 0.790 m | 0.225 m | 0.0248 m | 7.35 / 25.1 / 1.40° | 73 % |
| noise on, decorrelated GPS seed | 0.759 m | 0.225 m | 0.0346 m | 18.4 / 18.4 / 1.43° | 92 % |

**Read the last column before the others.** The Simulink loop holds position,
but its bank command is saturated for three quarters of the noisy run *including
in hover*, where the position error is 2.5 cm. It is not tracking a clean
command; it is chattering between rails and being low-passed into a small mean
bank by the attitude loop and the airframe.

**So "Simulink works" is a weaker statement than it looks, and that is good news
here** — it means SITL does not have to find a clean command to match the
reference. It has to find out why the same chatter that Simulink absorbs turns
into a limit cycle on SITL.

Two cautions on using this baseline as a target:

- The Simulink GPS noise is **one realisation shared by all three axes**
  (`WhiteNoise_A3`, scalar `seed = 4` on a 3-wide port, `VectorParams1D` on;
  measured identical across x/y/z to 2.2e-16, σ = 0.0503 m). Decorrelating it
  raises the rail fraction 73 % → 92 %. **The committed Simulink noise is
  optimistic.** Do not treat its rail figure as a bar to clear.
- The Simulink runs are noise-only. Both process-disturbance switches are
  grounded, and that rung is independently known to fail on the sibling HEOL
  controller.

## The Simulink configuration, exactly

Everything below is read from the six `mfc_siso_core` blocks. This is the
reference structure; §2 is the list of places firmware may differ.

| | φ, θ, ψ | thrust / z | pos x, y |
|---|---|---|---|
| structure | **coupled** (error-driven estimator, poles folded) | decoupled (measurement-driven, explicit iPD) | decoupled |
| model order | 2 | 2 | 2 |
| estimator | algebraic, growing window | same | same |
| `est_filter_window` | 5 samples | 50 | **100** |
| `est_hold_time` | **0** (literal) | 0.01 s (literal) | 0.01 s (literal) |
| `ref_filter_window` | 50 samples | 50 | 50 |
| `command_filter` (output EMA) | 1 = **off** | 1 = off | 1 = off |
| `Ki` | 0 | 0 | 0 |
| saturation | on | on | on, ±2.684 N = 20° bank |

Applied coefficients of `s² + Kd·s + Kp`, after the 2026-08-17 refactor that put
them in firmware form (the transform used to live in the block dialogs):

| axis | applied Kp | applied Kd |
|---|---|---|
| φ, θ | 4 | 12 |
| ψ | 4 | 6 |
| thrust | 16 | 5.6 |
| pos x, y | 75 | 150 |

On horizontal the stored gains carry an `alpha_pos_x` factor
(`Kp = alpha_pos_x·kp_pos_x²`) that the attitude and vertical dialogs do not.
This is a **param-file representation choice — not a control difference, and not a
special cancellation.** Every MFC axis forms `u = (−F̂ + ff − fb)/alpha`, so the
feedback `fb = Kp·err + Kd·derr` is divided by alpha on *every* axis alike; that
is universal MFC, nothing particular to horizontal. So do **not** treat the
post-alpha command gains (75/18.75 = 4, 150/18.75 = 8) as the coefficients to
line up against firmware. The horizontal **fb coefficients are 75 and 150** —
same status as roll's 4 and 12 — and firmware's `GX_KP`/`GX_KD` are its own fb
coefficients; match definition *and* alpha on both sides before comparing (that
reconciliation is §1's job, not asserted here). The `8` used in H3 is the
post-alpha command gain in N per m/s, which is the right quantity *there* for a
demand-vs-rail figure — a different use from a gain comparison.

`alpha`: φ/θ = 147.06 (`1/I_xx`), ψ = 73.53, thrust = 1.25 (`1/m`), pos = 18.75
(`15/m`), with `m` = 0.8 kg, `I` = [0.0068 0.0068 0.0136]. Every axis divides its
feedback by its own alpha — universal MFC, not a per-axis difference.

## 1. First, verify the firmware numbers — most of them are not

The parity work deleted a set of per-line firmware cross-reference comments from
`scripts/controllers/mfc_quad_params.m` **because several were provably wrong**
(they claimed `GX_KP = 0.8` and `GZ_KP = 4`). The values below are what those
comments claimed. They are recorded here so you know what to check, **not as
firmware facts**. `conf/airframes/ENAC/quadrotor/anton_mfc.xml` is the authority.

| claimed by the deleted comments | status |
|---|---|
| `ROLL_INTEGRATION_WINDOW = 5`, `PITCH` = 5, `YAW` = 5 | **unverified** |
| `GZ_INTEGRATION_WINDOW = 4` | **unverified — and see §3** |
| `GX_INTEGRATION_WINDOW = 5`, `GY` = 5 | **unverified — and see §3** |
| `ROLL_TIME_TRAJECTORY = 50` (etc.) | consistent with the SITL session's 50 → 250 |
| `GZ_TIME_TRAJECTORY = 4`, `GX_TIME_TRAJECTORY = 10` | **unverified** |
| `ROLL_COMMAND_FILTER = 1`, `GX_COMMAND_FILTER = 1`, `GZ_COMMAND_FILTER = 8` | **unverified — and see §5** |
| `ROLL_ALPHA = 2`, `PITCH_ALPHA = 2`, `YAW_ALPHA = 3`, `GZ_ALPHA = 5`, `GX_ALPHA = 10` | **unverified**; all differ from Simulink |

Only four firmware values are corroborated, and only because the parity note
asserted them and the Simulink applied values were independently computed:
`ROLL_PROPORTIONAL_GAIN = 4` (**agrees** with Simulink's applied 4),
`ROLL_DERIVATIVE_GAIN = 6` (**Simulink applies 12** — a real 2× difference on
roll and pitch), `GZ_KP = 16` (**agrees**), and `GX_KP = 2` — a firmware fb
coefficient, **not** comparable to a Simulink "effective 4"; the Simulink
horizontal fb coefficient is 75, and reconciling the two needs each side's alpha
matched (see the alpha note under "The Simulink configuration, exactly").

**And the unit trap, which invalidates every window comparison if skipped:**
`est_filter_window` and `ref_filter_window` are in **samples**, not seconds. The
smoother is `mfc_iir_smoother`, a critically damped second-order IIR with a
repeated pole at `z = W/(W+1)` — time constant ≈ W samples. Simulink runs at
`Ts = 0.002 s` (500 Hz), so its windows are 10 ms (attitude), 100 ms (z), 200 ms
(horizontal). **Establish the ANTON_MFC loop rate before comparing any window
number to any other.** If the rates differ, the integers are not comparable and
the milliseconds are.

## 2. The hypotheses, cheapest and highest-yield first

Take these in order. Each is a structural difference between a loop that flies
and one that does not, and each is checkable by reading `mfc_core.c` before any
SITL run.

### H1 — does the estimator see the SATURATED command?

In Simulink it does. `mfc_siso.step` ends with `state.u_km1 = u`, where `u` is
the command **after** the output EMA and **after** the clamp, and that is what
is fed back into the F estimator on the next sample.

This is load-bearing precisely because the horizontal command is on its rail
73–92 % of the time. If `mfc_core.c` feeds the estimator the pre-saturation
model-inversion output, then whenever the command is limited the estimator is
attributing to the plant a command the plant never received, F̂ drifts to cover
the difference, and the command is driven further into the rail. **That is a
limit-cycle generator, and it matches the reported XY symptom.**

This exact defect was found and documented on the sibling HEOL controller
(`Generic_Quad/Knowledge/RETRO-2026-08-17-heol-quad.md`, "the command blocks are
fed the unsaturated command"), so it is a known failure mode in this family and
not a speculative one.

**Check first. It is a read of one line of C.**

### H2 — is the position estimator window far too short?

Simulink's horizontal `est_filter_window` is **100 samples** and its z window is
**50**. The deleted comments claim firmware runs **5** and **4**. If that is
true and the rates are comparable, firmware's position estimator has 20× less
memory than the one that works.

The consequence is already measured, on the sibling controller and on the same
estimator function: `RETRO-2026-08-17-heol-quad.md` showed a position-driven
channel with a 5-sample window sitting at **390 m/s² rms** of F̂ noise with its
command on the clamp 97 % of the run, and a 500-sample window bringing the same
signal to **0.37 m/s²**. Its conclusion was that the channel had been grouped
with the attitude axes and sized for attitude noise, when its drive signal is a
position in metres carrying GPS noise.

For comparison, the F̂ actually measured in the working Simulink loop
(t > 1 s, noise on): **7.5 m/s² rms on x and y, 11.3 on z, 11.7 on φ**.

**"z command is noisy and choppy" is the exact symptom a too-short window
produces on a channel that is named for thrust but driven by altitude.** Check
what drives the GZ channel in `guidance_mfc.c` — if it is a position in metres,
it wants a position-sized window regardless of what it outputs.

### H3 — the derivative term, and whether anything filters it

Simulink's `mfc_siso.feedback` forms `dot_err = (err - err_km1)/Ts` — a **raw
backward difference, unfiltered**. `mfc_siso_core` exposes no derivative filter
coefficient at all. With the horizontal Kd of 8, `Ts` = 0.002 s and σ = 0.0503 m
of GPS noise, the per-sample noise difference alone is of order σ√2/Ts ≈ 35 m/s,
i.e. ~283 N of demand against a 2.684 N rail.

**That arithmetic is the leading explanation for why the Simulink command rails,
but it has not been measured** — `u_raw` was not logged and the mechanism was
not isolated. It is stated here so SITL can compare structures, not as a result.

What matters for this task: if firmware differentiates position the same way,
both sides share the mechanism and it is not the differentiator. If firmware
filters it, or has a different `Ts`, the noise gain differs and that is a
candidate. There is a measured-velocity estimate available on the vehicle;
whether the firmware PD uses it is worth knowing either way.

### H4 — the reference filter, where the two sides have already diverged

The SITL session raised the attitude `TIME_TRAJECTORY` **50 → 250** to cure a
65 % overshoot. **Simulink still runs 50 on every axis**, because the parity work
was behaviour-preserving and changed no gain.

Two consequences. First, any attitude comparison between the two sides is now
comparing different reference filters — normalise before concluding anything.
Second, the parity kickoff's judgement stands and is worth carrying: that
overshoot is **probably present in real flight too** and simply was not provoked,
because the attitude setpoint was moved gently by hand over RC. It is a real
characteristic, not a SITL artefact. Note also that 250 is **not**
flight-validated; 50 is.

### H5 — the command EMA and integral action

Simulink runs `command_filter = 1` on all six axes, i.e. **the output EMA is
disabled everywhere**, and `Ki = 0` everywhere. The deleted comments claim
firmware runs `GZ_COMMAND_FILTER = 8`.

If firmware has an output EMA on z where Simulink has none, the two z channels
have different command bandwidth and the "choppy" comparison is not
like-for-like. If firmware has integral action anywhere, then windup is live
whenever the command is limited — and Simulink's anti-windup experience is
worthless as a guide, because with `Ki = 0` its integrator freeze never
mattered.

### H6 — the startup hold, which SITL is far more exposed to than Simulink

`est_hold_time` guards a division by t² at the first sample; the algebraic
second-order estimator has a numerator term that does not carry a factor of t,
so as t → 0 the ratio blows up. In `mfc_quad.slx` it is the literal **0** on all
three attitude blocks and 0.01 s on z and horizontal.

It never fires in the Simulink runs because that trajectory starts at the origin
at rest, so every drive signal is exactly 0 at t = 0. **SITL does not have that
luxury.** Any nonzero initial position, altitude or attitude fires it with the
noise off — and the attitude channels are exposed too, because their estimator
drive is `me_φ − sp_φ` and `sp_φ` is the position loop's output, which is not
zero at t = 0. `RETRO-2026-08-17-heol-quad.md` measured a 1.6e4 m/s² startup
spike from exactly this, at any window length.

Check what `mfc_core.c` does at startup and what the vehicle's initial position
is in the frame the GZ/GX channels are driven in.

## 3. Order of work

Follow the isolation ladder (`Generic_Quad/AGENTS.md` → *Tuning is debugging*).
SITL is at rung 2. **Do not skip to horizontal.**

1. **Read before running.** H1, H2, H5, H6 are all answered by reading
   `mfc_core.c`, `guidance_mfc.c` and `anton_mfc.xml`. Produce the corrected
   version of the §1 table first — a verified correspondence, replacing a set of
   claims that have already been shown to rot.
2. **Rung 3, attitude with noise.** Re-establish it under the 250
   `TIME_TRAJECTORY`, and record whether the 65 % overshoot is gone or moved.
3. **Rung 4, vertical alone.** This is where H2 should either pay off or be
   eliminated. Instrument F̂ on the GZ channel and get an rms figure; the numbers
   to compare against are in H2. **Do not tune z gains before that number
   exists** — "noisy and choppy" with a 4-sample window is a bandwidth
   observation, not a gain observation.
4. **Rung 5/6, horizontal.** Only after vertical holds. H1 first.
5. **Noise, one source at a time.** Report every result as a no-noise /
   with-noise pair.

## 4. What NOT to do

- **Do not tune gains before the §1 table is verified.** Four of the numbers this
  task inherited were wrong, and a gain searched against a misread structure is
  a fabricated result.
- **Do not copy Simulink's gains across.** The alphas differ and each side
  divides its feedback by its own alpha, so a bare gain number is meaningless
  without its alpha and its definition. The transferable content of this document
  is the **structure and the windows**, not the numbers.
- **Do not treat the Simulink rail fraction as a target.** Its noise model is
  degenerate and optimistic; see the cautions above.
- **Do not chase the roll/pitch Kd 2× difference as a bug** until H1–H3 are
  closed. It is real and recorded, but a factor of 2 on a derivative gain is not
  what turns a working loop into a limit cycle.
- **Do not edit anything in `Generic_Quad`.** This task consumes that result; the
  restructure of the Simulink position loop is work item 2 of the parity kickoff
  and belongs to that worker.
- **No GA.** There is no MFC cost function and the structure is in flux.
- **Do not invent a number.** If a value is not in a file read this session, say
  so. Several of the values above are marked unverified for exactly that reason.

## 5. Done

- A verified firmware correspondence replacing the §1 table — every window, alpha,
  gain and filter read from `anton_mfc.xml` / `mfc_core.c`, with the loop rate
  attached so the sample-count windows mean something.
- H1 and H2 answered explicitly, with the code path quoted.
- SITL reported at a named rung, with a no-noise / with-noise pair at that rung,
  and an F̂ rms figure for the GZ channel.
- A session record in `paparazzi_dev/Knowledge/Sessions/`, and the parity note
  updated where this work contradicts it.

## Provenance

Simulink numbers: measured 2026-08-17 in worktree
`claude/mfc-quad-firmware-parity-77ca7d`, git `999451f` + uncommitted parity
refactor; payload `Data/2026-08-17_mfc_quad_firmware_form/data/`. Structural
claims about the controller: read from `MFC_SISO/functions/mfc_siso.m`,
`mfc_iir_smoother.m` and `MFC_SISO/blocks/mfc_siso_core.m` the same day.
Firmware claims: attributed inline, and unverified where marked.
