import csv, math, sys
from omni_robot_description.geometry import load_geometry
from omni_robot_gazebo.field_layout import load_layout
from omni_robot_gazebo.lidar_mount_study import Study
st = Study(load_geometry(None), load_layout(None), 0.05)
R = [r for r in csv.DictReader(open(sys.argv[1])) if r["state"] == "GO_SAKURA"]
f = lambda r, k: float(r[k]) if r[k] not in ("", "nan") else float("nan")
prev = None
print("t      gt_x   gt_y  clr_mm  cross_est_mm cross_gt_mm  ref_v  |raw|  |cmd|  raw-ff_lat  dv_raw_cmd  gt_speed")
for i, r in enumerate(R):
    gx, gy, gth = f(r,"gt_x"), f(r,"gt_y"), f(r,"gt_yaw")
    if not (float(sys.argv[2]) <= f(r,"t") <= float(sys.argv[3])) or i % 2: continue
    c, n = st.clearance(gx, gy, gth)
    # path tangent from consecutive ref points
    j = min(i+1, len(R)-1); tx, ty = f(R[j],"ref_x")-f(r,"ref_x"), f(R[j],"ref_y")-f(r,"ref_y"); L = math.hypot(tx,ty) or 1; tx, ty = tx/L, ty/L
    cr_e = (-(f(r,"est_x")-f(r,"ref_x"))*ty + (f(r,"est_y")-f(r,"ref_y"))*tx)*1e3
    cr_g = (-(gx-f(r,"ref_x"))*ty + (gy-f(r,"ref_y"))*tx)*1e3
    raw = math.hypot(f(r,"raw_vx"), f(r,"raw_vy")); cmd = math.hypot(f(r,"cmd_vx"), f(r,"cmd_vy"))
    lat = (-f(r,"raw_vx")*ty + f(r,"raw_vy")*tx)
    dv = math.hypot(f(r,"raw_vx")-f(r,"cmd_vx"), f(r,"raw_vy")-f(r,"cmd_vy"))
    print(f"{f(r,'t'):.2f} {gx:6.3f} {gy:6.3f} {1e3*c:6.0f} {cr_e:+8.1f} {cr_g:+8.1f}   {f(r,'ref_v'):.2f}  {raw:.2f}  {cmd:.2f}   {lat:+.3f}     {dv:.3f}    {f(r,'gt_speed'):.2f}  {n}")
