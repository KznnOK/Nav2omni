import sys, numpy as np
d = np.load(sys.argv[1])
gt, trk, ekf, dbg, mo = d["gt"], d["trk"], d["ekf"], d["dbg"], d["mo"]
def I(a, t, c): return np.interp(t, a[:,0], a[:,c])
def Iyaw(a, t, c): return np.interp(t, a[:,0], np.unwrap(a[:,c]))
def wrap(x): return (x+np.pi)%(2*np.pi)-np.pi
print("rates Hz:", {k: round(len(d[k])/(d[k][-1,0]-d[k][0,0]),1) for k in d.files if len(d[k])>2})
t = dbg[:,0]; t = t[(t>gt[0,0])&(t<gt[-1,0])]
dbg = dbg[np.isin(dbg[:,0], t)]
gx, gy, gyaw = I(gt,t,1), I(gt,t,2), Iyaw(gt,t,3)
gvx, gvy = I(gt,t,4), I(gt,t,5); sp = np.hypot(gvx, gvy)
fast = sp > 0.8
ux, uy = gvx/np.maximum(sp,1e-9), gvy/np.maximum(sp,1e-9)
def decomp(name, ex, ey):
    al = ex*ux+ey*uy; cr = -ex*uy+ey*ux   # cr>0 = left of travel
    print(f"{name:10s} fast n={fast.sum()} along mean {1e3*al[fast].mean():+.1f} cross mean {1e3*cr[fast].mean():+.1f} sd {1e3*cr[fast].std():.1f}  | slow(<0.1) |e| mean {1e3*np.hypot(ex,ey)[sp<0.1].mean():.1f}")
    return al, cr
decomp("control", dbg[:,1]-gx, dbg[:,2]-gy)
decomp("amcl_raw", dbg[:,4]-gx, dbg[:,5]-gy)
print(f"yaw err fast: control {np.degrees(wrap(dbg[:,3]-gyaw)[fast]).mean():+.3f} deg, amcl {np.degrees(wrap(dbg[:,6]-gyaw)[fast]).mean():+.3f} deg")
# EKF pose in odom vs gt: relative motion. Compare increments in body frame over windows
te = ekf[:,0]; te = te[(te>gt[0,0])&(te<gt[-1,0])]
for name, a in (("tracking", trk), ("ekf", ekf)):
    ta = a[:,0]; ta = ta[(ta>gt[0,0]+0.5)&(ta<gt[-1,0])]
    # body twist comparison
    yaw_g = Iyaw(gt, ta, 3); vxg, vyg = I(gt,ta,4), I(gt,ta,5)
    bx = np.cos(yaw_g)*vxg + np.sin(yaw_g)*vyg; by = -np.sin(yaw_g)*vxg + np.cos(yaw_g)*vyg
    ax, ay = I(a,ta,4), I(a,ta,5)
    s = np.hypot(bx,by); f = s>0.8
    # angle between measured and true body velocity
    ang = np.degrees(wrap(np.arctan2(ay,ax)-np.arctan2(by,bx)))[f]
    ratio = (np.hypot(ax,ay)/s)[f]
    print(f"{name:9s} twist fast: dir err mean {ang.mean():+.3f} deg sd {ang.std():.3f}, |v| ratio {ratio.mean():.4f}")
    # best lag between measured and true speed
    best = min(((np.mean((np.interp(ta[f]-L, ta, np.hypot(ax,ay)) - s[f])**2), L) for L in np.arange(-0.05,0.0501,0.002)))
    print(f"          speed lag (meas behind truth) {1e3*-best[1]:.0f} ms" )
    # odom yaw vs gt yaw (drift of the yaw estimate)
    dyaw = wrap(Iyaw(a, ta, 3) - yaw_g); print(f"          yaw(odom)-yaw(gt): start {np.degrees(dyaw[0]):+.3f} end {np.degrees(dyaw[-1]):+.3f} deg")
    # pose drift: align at first sample
    x0,y0,th0 = I(a,ta[:1],1)[0], I(a,ta[:1],2)[0], Iyaw(a,ta[:1],3)[0]
    gx0,gy0,gth0 = I(gt,ta[:1],1)[0], I(gt,ta[:1],2)[0], yaw_g[0]
    r = gth0-th0; px, py = I(a,ta,1)-x0, I(a,ta,2)-y0
    wx = gx0 + np.cos(r)*px - np.sin(r)*py; wy = gy0 + np.sin(r)*px + np.cos(r)*py
    e = np.hypot(wx-I(gt,ta,1), wy-I(gt,ta,2)); dist = np.cumsum(np.r_[0,np.hypot(np.diff(I(gt,ta,1)),np.diff(I(gt,ta,2)))])
    print(f"          dead-reckoning error: end {1e3*e[-1]:.1f} mm over {dist[-1]:.2f} m, max {1e3*e.max():.1f}")
mo_t = mo[:,0]
print(f"map->odom yaw: range {np.degrees(mo[:,3].min()):+.3f}..{np.degrees(mo[:,3].max()):+.3f} deg; xy range {1e3*np.ptp(mo[:,1]):.0f},{1e3*np.ptp(mo[:,2]):.0f} mm")
