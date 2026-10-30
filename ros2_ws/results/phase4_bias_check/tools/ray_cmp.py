import sys, numpy as np, warnings
warnings.filterwarnings("ignore")
from PIL import Image
d = np.load(sys.argv[1]); gt = d["gt"]; scan = d["scan"]; a0, da, rmin, rmax = d["scan_meta"]
img = np.array(Image.open("/home/ubuntu/robocon_study/ros2/omni_nav2/ros2_ws/src/omni_robot_nav2/maps/field_map.png").convert("L"))
occ = (img < 100)[::-1]; res, ox, oy = 0.01, -3.5, -3.5
n = scan.shape[1]-1; ang = a0 + da*np.arange(n)
keep = ~((np.abs(np.degrees(ang)) > 89.05) & (np.abs(np.degrees(ang)) < 136))
sys.path.insert(0,"")
from omni_robot_gazebo.field_layout import frame_rects, load_layout
rects = frame_rects(load_layout(None))
def cast(sx, sy, h):
    # exact ray vs frame rectangles (Gazebo geometry)
    dx, dy = np.cos(h), np.sin(h); best = np.full_like(h, np.inf)
    for _, x0, x1, y0, y1 in rects:
        with np.errstate(divide="ignore", invalid="ignore"):
            tx1, tx2 = (x0-sx)/dx, (x1-sx)/dx; ty1, ty2 = (y0-sy)/dy, (y1-sy)/dy
        tmin = np.maximum(np.minimum(tx1,tx2), np.minimum(ty1,ty2)); tmax = np.minimum(np.maximum(tx1,tx2), np.maximum(ty1,ty2))
        hit = (tmax >= tmin) & (tmin > 0)
        best = np.where(hit & (tmin < best), tmin, best)
    return best
th_u = np.unwrap(gt[:,3]); out = []
for k in range(0, len(scan), 2):
    t = scan[k,0]
    if t < gt[0,0] or t > gt[-1,0]: continue
    v = np.hypot(np.interp(t, gt[:,0], gt[:,4]), np.interp(t, gt[:,0], gt[:,5]))
    if v > 0.02: continue
    x, y, th = np.interp(t, gt[:,0], gt[:,1]), np.interp(t, gt[:,0], gt[:,2]), np.interp(t, gt[:,0], th_u)
    r = scan[k,1:].astype(float); m = keep & np.isfinite(r) & (r>0.3) & (r<rmax)
    exp = cast(x+0.25*np.cos(th), y+0.25*np.sin(th), th+ang)
    e = (r-exp)[m]; good = np.abs(e) < 0.1
    out.append(np.c_[ang[m][good], e[good]])
O = np.vstack(out)
print(f"rest beams n={len(O)} range err (meas-exact) mean {1e3*O[:,1].mean():+.1f} mm median {1e3*np.median(O[:,1]):+.1f} sd {1e3*O[:,1].std():.1f}")
for lo in range(-135, 135, 30):
    f = (np.degrees(O[:,0])>=lo)&(np.degrees(O[:,0])<lo+30)
    if f.sum(): print(f"  beam {lo:+4d}..{lo+30:+4d} deg: mean {1e3*O[f,1].mean():+.1f} mm n={f.sum()}")
