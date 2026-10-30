import sys, numpy as np, warnings
warnings.filterwarnings("ignore")
from PIL import Image
from scipy.ndimage import distance_transform_edt
d = np.load(sys.argv[1]); gt = d["gt"]; scan = d["scan"]; a0, da, rmin, rmax = d["scan_meta"]
img = np.array(Image.open("/home/ubuntu/robocon_study/ros2/omni_nav2/ros2_ws/src/omni_robot_nav2/maps/field_map.png").convert("L"))
occ = (img < 100)[::-1]          # row 0 = y min
mode = sys.argv[2] if len(sys.argv) > 2 else "filled"
if mode.startswith("r5"):
    from omni_robot_gazebo.field_layout import frame_rects, load_layout
    res = 0.005; nc = 1400; occ = np.zeros((nc, nc), bool)
    for _, x0, x1, y0, y1 in frame_rects(load_layout(None)):
        c0 = int(np.floor((x0+3.5)/res+1e-6)); c1 = int(np.ceil((x1+3.5)/res-1e-6))
        r0 = int(np.floor((y0+3.5)/res+1e-6)); r1 = int(np.ceil((y1+3.5)/res-1e-6))
        occ[r0:r1, c0:c1] = True
if mode == "surface":
    from omni_robot_gazebo.field_layout import frame_rects, load_layout
    cx = -3.5 + 0.01*(np.arange(occ.shape[1])+0.5); cy = -3.5 + 0.01*(np.arange(occ.shape[0])+0.5)
    X, Y = np.meshgrid(cx, cy); occ = np.zeros_like(occ); h = 0.005 + 1e-9
    for _, x0, x1, y0, y1 in frame_rects(load_layout(None)):
        inside = (X >= x0-h) & (X <= x1+h) & (Y >= y0-h) & (Y <= y1+h)
        deep = (X > x0+h) & (X < x1-h) & (Y > y0+h) & (Y < y1-h)
        occ |= inside & ~deep
if mode.startswith("center"):
    from omni_robot_gazebo.field_layout import frame_rects, load_layout
    cx = -3.5 + 0.01*(np.arange(occ.shape[1])+0.5); cy = -3.5 + 0.01*(np.arange(occ.shape[0])+0.5)
    occ = np.zeros_like(occ)
    for _, x0, x1, y0, y1 in frame_rects(load_layout(None)):
        occ[np.ix_((cy>=y0)&(cy<=y1), (cx>=x0)&(cx<=x1))] = True
ox, oy = -3.5, -3.5; res = 0.005 if mode.startswith("r5") else 0.01
from scipy.ndimage import binary_erosion
if mode.endswith("edge"): occ = occ & ~binary_erosion(occ)
dist = distance_transform_edt(~occ) * res
n = scan.shape[1]-1; ang = a0 + da*np.arange(n)
keep_ang = ~((np.abs(np.degrees(ang)) > 89.05) & (np.abs(np.degrees(ang)) < 136))
LX = 0.25
def endpoints(x, y, th, r, m):
    sx, sy = x + LX*np.cos(th), y + LX*np.sin(th)
    return sx + r[m]*np.cos(th+ang[m]), sy + r[m]*np.sin(th+ang[m])
def lookup(px, py):
    c = np.clip(((px-ox)/res).astype(int), 0, occ.shape[1]-1); rr = np.clip(((py-oy)/res).astype(int), 0, occ.shape[0]-1)
    return dist[rr, c]
SIG = float(sys.argv[3]) if len(sys.argv) > 3 else 0.05
def score(x,y,th,r,m, sig=None):
    sig = SIG if sig is None else sig
    z = lookup(*endpoints(x,y,th,r,m)); p = 0.95*np.exp(-z*z/(2*sig*sig)) + 0.05/30
    return np.sum(p**3), z
th_u = np.unwrap(gt[:,3])
offs = np.arange(-0.03, 0.0301, 0.002)
rows = []
for k in range(0, len(scan), 4):
    t = scan[k,0]; r = scan[k,1:].astype(float)
    if t < gt[0,0] or t > gt[-1,0]: continue
    x, y, th = np.interp(t, gt[:,0], gt[:,1]), np.interp(t, gt[:,0], gt[:,2]), np.interp(t, gt[:,0], th_u)
    v = np.hypot(np.interp(t, gt[:,0], gt[:,4]), np.interp(t, gt[:,0], gt[:,5]))
    m = keep_ang & np.isfinite(r) & (r > 0.3) & (r < rmax)
    _, z0 = score(x,y,th,r,m)
    c, s = np.cos(th), np.sin(th)
    best = max(((score(x+c*bx-s*by, y+s*bx+c*by, th, r, m)[0], bx, by) for bx in offs for by in offs))
    rows.append((t, v, best[1], best[2], np.median(z0), np.degrees(th)))
R = np.array(rows)
for name, f in (("rest v<0.05", R[:,1]<0.05), ("moving v>0.8", R[:,1]>0.8)):
    print(f"{name}: n={f.sum()} best body dx {1e3*R[f,2].mean():+.1f} mm (sd {1e3*R[f,2].std():.1f}), dy {1e3*R[f,3].mean():+.1f} mm; median endpoint dist at truth {1e3*R[f,4].mean():.1f} mm")
