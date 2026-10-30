"""Phase 5 summary: per condition and LiDAR mode, 3 runs."""
import csv, math, sys, collections
from omni_robot_description.geometry import load_geometry
from omni_robot_gazebo.field_layout import load_layout
from omni_robot_gazebo.lidar_mount_study import Study
st = Study(load_geometry(None), load_layout(None), 0.05)
runs, tsdir = sys.argv[1], sys.argv[2]
R = list(csv.DictReader(open(runs)))
def num(x):
    try: return float(str(x).replace("mm", ""))
    except ValueError: return float("nan")
def clearance(label):
    best = {"GO": 9.0, "RET": 9.0}
    try: rows = list(csv.DictReader(open(f"{tsdir}/{label}.csv")))
    except FileNotFoundError: return best
    for i, p in enumerate(rows):
        if i % 3: continue
        x, y, th = float(p["true_x"]), float(p["true_y"]), float(p["true_yaw"])
        k = "GO" if p["state"] == "GO_SAKURA" and y > -2.55 else "RET" if p["state"] == "RETURN" else None
        if k: best[k] = min(best[k], st.clearance(x, y, th)[0])
    return best
groups = collections.OrderedDict()
for r in R:
    cond, lidar, _ = r["label"].rsplit("_", 2)
    r.update({f"clr_{k}": v for k, v in clearance(r["label"]).items()})
    groups.setdefault((cond.replace("_lidartrue", "").replace("_lidarfalse", ""), lidar), []).append(r)
rows = []
for (cond, lidar), rs in groups.items():
    ok = [r for r in rs if r["success"] == "True"]
    stop = [abs(num(r[k])) for r in rs for k in r if k.endswith(("_dx_mm", "_dy_mm")) and r[k] != ""]
    ov = [num(r[k]) for r in ok for k in ("go_overshoot", "reapproach1_overshoot", "return_stop_overshoot") if r[k]]
    se = [num(r[k]) for r in ok for k in r if k.endswith("_settle_s") and r[k]]
    tot = [num(r["time_total_s"]) for r in ok]
    fails = sorted({r["failure"].split(";")[0][:40] for r in rs if r["success"] != "True"})
    rows.append(dict(cond=cond, lidar="あり" if lidar == "lidartrue" else "なし", n=len(rs), ok=len(ok),
        t_mean=sum(tot)/len(tot) if tot else math.nan, t_max=max(tot) if tot else math.nan,
        go=sum(num(r["time_go_s"]) for r in ok)/len(ok) if ok else math.nan,
        rot=sum(num(r["time_rotation_s"]) for r in ok)/len(ok) if ok else math.nan,
        ret=sum(num(r["time_return_s"]) for r in ok)/len(ok) if ok else math.nan,
        stop=max(stop) if stop else math.nan, ov=max(ov) if ov else math.nan, se=max(se) if se else math.nan,
        cgo=1e3*min(r["clr_GO"] for r in rs), cret=1e3*min(r["clr_RET"] for r in rs),
        est=max(num(r["max_est_error_mm"]) for r in rs), contact=sum(r["wall_contact"] == "True" for r in rs),
        fails="; ".join(fails)))
print("| 条件 | LiDAR補正 | 成功 | 合計 平均/最大 s | 往路/回転/復路 平均 s | 停止誤差 最大 mm | 行き過ぎ 最大 mm | 整定 最大 s | 最小距離 往路/復路 mm | 推定誤差ピーク 最大 mm | 接触 | 失敗内容 |")
print("|---|---|---|---|---|---|---|---|---|---|---|---|")
f = lambda v, d=1: "-" if v != v else f"{v:.{d}f}"
for x in rows:
    print(f"| {x['cond']} | {x['lidar']} | {x['ok']}/{x['n']} | {f(x['t_mean'],2)} / {f(x['t_max'],2)} | {f(x['go'],2)}/{f(x['rot'],2)}/{f(x['ret'],2)} | {f(x['stop'])} | {f(x['ov'])} | {f(x['se'],2)} | {f(x['cgo'],0)} / {f(x['cret'],0)} | {f(x['est'])} | {x['contact']} | {x['fails']} |")
import json; json.dump(rows, open(runs + ".summary.json", "w"), ensure_ascii=False, indent=1)
