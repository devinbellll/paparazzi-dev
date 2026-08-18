# The Flat Reference Fixed-Point Ratchet

**A truncation bug in `gv_update_ref_from_flat_ref()` that a PID would never have
noticed, and that saturated the MFC thrust command once per 100 ms.** Found
2026-08-18 on Hoops_111_MFC, confirmed on ANTON_MFC, fixed in `guidance_v_ref.c`.

Companion to [[16 - The MFC Estimator Saturation Tap]]. Both are the same species:
something benign to a conventional controller that is lethal to one which
differentiates its own setpoint.

## The one-line version

`gv_z_ref` is stored **Q37.26**. The extrapolator round-tripped it through
**Q23.8** every tick, and `BFP_OF_REAL` truncates *toward zero* while NED altitude
is *negative* — so the reference ratcheted **upward by a full LSB (3.9 mm) per
2 ms tick**, a phantom **+1.95 m/s climb rate**.

## The code

```c
/* before */
float z_ref_f = (float)(gv_z_ref >> (GV_Z_REF_FRAC - INT32_POS_FRAC)) / (1 << INT32_POS_FRAC);
...
gv_z_ref = ((int64_t)BFP_OF_REAL(z_ref_f, INT32_POS_FRAC)) << (GV_Z_REF_FRAC - INT32_POS_FRAC);
```

`GV_Z_REF_FRAC = GV_ZD_REF_FRAC + GV_FREQ_FRAC = (8 + 9) + 9 = 26`.
`INT32_POS_FRAC = 8`. So the shift pair discards **18 bits** of the reference on
every tick, and `BFP_OF_REAL(_vr,_frac)` is `((_vr)*(1<<(_frac)))` assigned to an
integer — a C truncation toward zero.

## Why it is a ratchet and not a rounding error

Take the reference sitting at −3.0 m. In Q23.8 that is exactly −768.

Add *any* representable positive step, however small: −767.9999…
Truncate **toward zero**: **−767**. The reference has moved **+1/256 m**, a whole
LSB, from an input of ~1e-6 m.

Next tick starts at −767 (= −2.99609375, again exact in Q23.8) and does it again.

**The drift per tick is one full LSB regardless of the true step size.** Measured
in a standalone harness replicating the arithmetic:

| true step | old code, drift over 100 ms | apparent climb rate |
|---|---|---|
| 1e-6 m/tick | +0.19531 m | **+1.95 m/s** |
| 1e-5 m/tick | +0.19531 m | **+1.95 m/s** |
| 1e-4 m/tick | +0.19531 m | **+1.95 m/s** |

`+0.19531 m = 50 × 0.00390625` — exactly 50 LSB in 50 ticks.

Two sign conditions are needed and both hold here: truncation toward zero, and a
**negative** quantity. In NED, altitude above ground is always negative. A north
or east position would ratchet only while negative and would drift the other way
while positive — worth remembering if this pattern appears elsewhere.

## Why it reached the vehicle

`gv_set_flat_ref()` re-syncs the reference from the trajectory at the **nav rate**;
`gv_update_ref_from_flat_ref()` extrapolates at the **500 Hz loop rate**. So the
reference ramped up ~50 LSB and got snapped back, over and over: a **~10 Hz
sawtooth** on `guidance_v.z_ref`, amplitude ~0.2 m.

That is invisible to a PID — it is a small, fast wobble on a position reference.

But `mfc_core.c` forms `u = (−F_hat + rddot − fb)/alpha`, where `rddot` is
`dot_dot_setpoint_trajec`, the **second difference of the setpoint divided by
dt²**. A −0.0586 m snap in one 2 ms sample is

```
rddot = -0.0586 / (0.002)^2 = -14650 m/s^2
```

which after `/alpha` demands thousands of Newtons and pins the command at its
clamp. Observed in flight:

- `cmd_z` slamming between **+7.848 N** (the `u_max = m·g` clamp) and **−18 N** at 10 Hz
- `fk_z` swinging **±100**
- the vehicle climbing **~1.7 m past** the flat-trajectory endpoint and holding
  there, with a standing error the loop could not remove
- `MFC_GUIDANCE/sp_z` visibly **hatched** in PlotJuggler rather than a flat line —
  this is the tell, and it is visible at a glance

The asymmetry of the saturated oscillation is what produced net lift.

## The fix

Accumulate the **step** at the reference's native resolution instead of
round-tripping the absolute value:

```c
gv_z_ref  += (int64_t)llroundf(z_step * (float)((int64_t)1 << GV_Z_REF_FRAC));
gv_zd_ref  = (int32_t)lroundf(zd_ref_f  * (float)(1 << GV_ZD_REF_FRAC));
gv_zdd_ref = (int32_t)lroundf(zdd_ref_f * (float)(1 << GV_ZDD_REF_FRAC));
```

Note a float **cannot** hold the absolute position at Q37.26 — `3 m × 2^26 = 2e8`,
past a 24-bit mantissa. The step is ~1e-5 m, so converting the *step* at full
resolution is both representable and correct. That is why the fix is "accumulate
the step", not "use a wider read".

Rounding rather than truncation on the speed/accel write-backs closes the same
hole at their (much finer) resolutions.

## Scope

`guidance_v_set_flat()` / `gv_update_ref_from_flat_ref()` are shared. **This
affected every consumer of the flat vertical reference — HEOL as well as MFC**,
and any future controller that differentiates its setpoint.

## The general lesson

**A fixed-point round-trip inside a per-tick integrator is an integrator of its
own quantization error.** Two questions to ask of any such loop:

1. Does the read/write pair preserve the stored resolution? Here it threw away
   18 of 26 fractional bits, every tick, at 500 Hz.
2. Does the conversion round or truncate, and is the quantity signed? Truncation
   toward zero is a *biased* estimator for negative values, and bias inside an
   integrator is drift, not noise.

And the diagnostic habit that found it: when a setpoint looks wrong, plot the
**setpoint** at full rate before touching the controller. The sawtooth was
visible in the raw log the moment it was sampled per-tick rather than per-second
— but only at per-tick resolution. Decimated plots average it away and it looks
like a clean line at the right value.

Related: [[16 - The MFC Estimator Saturation Tap]],
[[Sessions/2026-08-18-flat-traj-gcs-fixes]], bug-274.
