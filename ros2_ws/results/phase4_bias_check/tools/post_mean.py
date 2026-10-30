import sys, numpy as np
src = open(sys.argv[1]).read().split("th_u = np.unwrap")[0]
sys.argv = [sys.argv[0], sys.argv[2], sys.argv[3], sys.argv[4]]
exec(src)
gt = d["gt"]; th_u = np.unwrap(gt[:,3]); offs = np.arange(-0.06, 0.0601, 0.003)
W = []
for k in range(0, len(scan), 6):
    t = scan[k,0]
    if t < gt[0,0] or t > gt[-1,0]: continue
    v = np.hypot(np.interp(t, gt[:,0], gt[:,4]), np.interp(t, gt[:,0], gt[:,5]))
    if v > 0.05: continue
    x, y, th = np.interp(t, gt[:,0], gt[:,1]), np.interp(t, gt[:,0], gt[:,2]), np.interp(t, gt[:,0], th_u)
    r = scan[k,1:].astype(float); m = keep_ang & np.isfinite(r) & (r > 0.3) & (r < rmax)
    idx = np.where(m)[0]; m2 = np.zeros_like(m); m2[idx[::max(1, len(idx)//120)]] = True   # ~120 beams like AMCL
    c, s = np.cos(th), np.sin(th)
    W.append([1 + score(x + c*b, y + s*b, th, r, m2)[0] for b in offs])
W = np.array(W); w = W.mean(0)
for spread in (0.01, 0.02, 0.03):
    p = w * np.exp(-offs**2/(2*spread**2)); p2 = (W*np.exp(-offs**2/(2*spread**2))).T
    means = (offs[:,None]*p2).sum(0)/p2.sum(0)
    print(f"  particle spread {1e3*spread:.0f} mm: weighted-mean body dx {1e3*means.mean():+.1f} mm; argmax {1e3*offs[np.argmax(w)]:+.0f}")
print("  w(dx) normalized: " + " ".join(f"{1e3*o:+.0f}:{v/w.max():.2f}" for o, v in zip(offs[::4], w[::4])))
