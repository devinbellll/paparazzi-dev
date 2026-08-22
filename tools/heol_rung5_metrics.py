#!/usr/bin/env python3
"""Fixed-window rung-5 HEOL metrics. Windows are absolute sim seconds so every
run is scored on the same clock: Standby hold 12-21.5, flat-traj block 22-62,
return-to-Standby 63-76, late hold 76-end."""
import csv, math, sys
BANK = 0.3491
def main(path):
    r = list(csv.DictReader(open(path)))
    t = [float(x['time']) for x in r]
    bad = sum(1 for i in range(1, len(t)) if t[i] < t[i-1])
    def col(k): return [(tt, float(x[k])) for tt, x in zip(t, r) if x.get(k) not in (None, '', 'None')]
    def w(p, a, b): return [v for tt, v in p if a <= tt <= b]
    def rms(v): return math.sqrt(sum(a*a for a in v)/len(v)) if v else float('nan')
    def rail(v): return 100.0*sum(1 for a in v if abs(a) >= BANK*0.995)/len(v) if v else float('nan')
    tl = t[-1]
    print(f"{path}  rows {len(r)}  span {t[0]:.2f}..{tl:.2f}  backsteps {bad}"
          + ("  *** CONTAMINATED ***" if bad else ""))
    for lbl, a, b in (('Standby hold  12-21.5', 12, 21.5), ('flat-traj blk 22-62', 22, 62),
                      ('Standby       63-76', 63, 76), ('late hold     76-end', 76, tl)):
        ex, ey, ez = (w(col('/uav/MFC_GUIDANCE/err_'+k), a, b) for k in 'xyz')
        cx, cy = (w(col('/uav/MFC_GUIDANCE/cmd_'+k), a, b) for k in 'xy')
        fx, fy = (w(col('/uav/MFC_GUIDANCE/fk_'+k), a, b) for k in 'xy')
        print(f"  {lbl:22s} errRMS {rms(ex):7.3f} /{rms(ey):7.3f} /{rms(ez):6.4f} m"
              f"   maxErr {max(map(abs,ex+ey)):7.3f} m"
              f"   railed {rail(cx):5.1f} /{rail(cy):5.1f} %"
              f"   fk range [{min(fx+fy):+7.2f},{max(fx+fy):+7.2f}]")
if __name__ == '__main__':
    for a in sys.argv[1:]:
        main(a)
