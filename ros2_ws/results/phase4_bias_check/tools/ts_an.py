import sys, numpy as np, csv
for f in sys.argv[1:]:
    R = list(csv.DictReader(open(f)))
    g = lambda k: np.array([float(r[k]) for r in R])
    t, ex, ey = g("t"), g("est_x")-g("true_x"), g("est_y")-g("true_y"); th = g("true_yaw")
    x, y = g("true_x"), g("true_y")
    vx, vy = np.gradient(x, t), np.gradient(y, t)
    c, s = np.cos(th), np.sin(th)
    ebx, eby = c*ex+s*ey, -s*ex+c*ey; vbx, vby = c*vx+s*vy, -s*vx+c*vy
    sp = np.hypot(vx, vy); m = (t > t[0]+1) & np.isfinite(vbx)
    A = np.c_[np.ones(m.sum()), vbx[m], vby[m]]
    out = []
    for e in (ebx[m], eby[m]):
        k = np.linalg.lstsq(A, e, rcond=None)[0]; out.append(k)
    e = np.hypot(ex, ey)
    print(f"{f.split('/')[-1]:24s} body x: {1e3*out[0][0]:+5.1f} mm {1e3*out[0][1]:+5.1f} ms*vx {1e3*out[0][2]:+5.1f} ms*vy | body y: {1e3*out[1][0]:+5.1f} mm {1e3*out[1][1]:+5.1f} ms*vx {1e3*out[1][2]:+5.1f} ms*vy | |e| rest {1e3*e[m&(sp<0.05)].mean():.1f} fast {1e3*e[m&(sp>0.8)].mean():.1f} max {1e3*e[m].max():.1f} | rate {len(t)/(t[-1]-t[0]):.0f} Hz")
