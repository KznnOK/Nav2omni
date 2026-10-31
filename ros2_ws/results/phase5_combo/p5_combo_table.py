"""Phase 5 combined-error tables: per run, per condition/start, and vs baseline.
usage: p5_combo_table.py runs.csv stalls.csv"""
import csv
import statistics
import sys

runs = list(csv.DictReader(open(sys.argv[1])))
stalls = {r[0]: int(r[1]) for r in csv.reader(open(sys.argv[2])) if r}


def num(x):
    try:
        return float(str(x).replace("mm", "").replace("deg", ""))
    except ValueError:
        return float("nan")


def stop_error(r):
    vals = [abs(num(r[k])) for k in ("sakura_dx_mm", "sakura_dy_mm", "sakura_rotated_dx_mm",
                                     "sakura_rotated_dy_mm", "return_dx_mm", "return_dy_mm")
            if r[k] != ""]
    return max(vals) if vals else float("nan")


def overshoot(r):  # position overshoots only (mm); the rotation ones are degrees
    vals = [num(r[k]) for k in ("go_overshoot", "reapproach1_overshoot", "return_stop_overshoot")
            if r[k] != ""]
    return max(vals) if vals else float("nan")


def settle(r):
    vals = [num(r[k]) for k in ("go_settle_s", "rotate1_settle_s", "reapproach1_settle_s",
                                "rotate2_settle_s", "return_stop_settle_s") if r[k] != ""]
    return max(vals) if vals else float("nan")


def f(x, d=1):
    return "-" if x != x else f"{x:.{d}f}"


for r in runs:
    cond, mode, _, k = r["label"].rsplit("_", 3)
    r.update(cond=cond, mode=mode, k=k, stop=stop_error(r), over=overshoot(r),
             settle=settle(r), stalls=stalls.get(r["label"], 0),
             ok=r["success"] == "True")

print("## 1回ごと\n")
print("| 試行 | 開始 | 成功 | 区間 往路/回転/復路 s | 合計 s | 停止誤差 最大 mm | 行き過ぎ 最大 mm | 整定 最大 s | 壁との最小距離 mm（場所） | 推定誤差ピーク mm | 停止の監視 | 失敗内容 |")
print("|---|---|---|---|---|---|---|---|---|---|---|---|")
for r in runs:
    print(f"| {r['cond']} #{r['k']} | {'標準' if r['mode'] == 'std' else 'ランダム'} "
          f"({float(r['start_x']):.2f}, {float(r['start_y']):.2f}, {float(r['start_yaw_deg']):.0f}°) | "
          f"{'○' if r['ok'] else '×'} | {r['time_go_s']}/{r['time_rotation_s']}/{r['time_return_s']} | "
          f"{r['time_total_s']} | {f(r['stop'])} | {f(r['over'])} | {f(r['settle'], 2)} | "
          f"{r['min_wall_clearance_mm']} ({r['min_wall_clearance_at']}) | {r['max_est_error_mm']} | "
          f"{r['stalls']} | {r['failure']} |")

print("\n## 条件 × 開始位置の集計（平均 / 最大）\n")
print("| 条件 | 開始 | 成功 | 合計 s | 停止誤差 mm | 行き過ぎ mm | 整定 s | 壁との最小距離 mm（最小） | 推定誤差ピーク mm | 停止の監視 計 |")
print("|---|---|---|---|---|---|---|---|---|---|")
groups = {}
for r in runs:
    groups.setdefault((r["cond"], r["mode"]), []).append(r)


def agg(rs, key, fn=max):
    vals = [num(r[key]) if not isinstance(r[key], float) else r[key] for r in rs]
    vals = [v for v in vals if v == v]
    return vals


summary = {}
for (cond, mode), rs in groups.items():
    ok = [r for r in rs if r["ok"]]
    tot = [num(r["time_total_s"]) for r in ok]
    stop = [r["stop"] for r in rs if r["stop"] == r["stop"]]
    over = [r["over"] for r in rs if r["over"] == r["over"]]
    st = [r["settle"] for r in rs if r["settle"] == r["settle"]]
    wall = [num(r["min_wall_clearance_mm"]) for r in rs]
    est = [num(r["max_est_error_mm"]) for r in rs]
    s = dict(ok=f"{len(ok)}/{len(rs)}",
             total=statistics.mean(tot) if tot else float("nan"),
             total_max=max(tot) if tot else float("nan"),
             stop=max(stop) if stop else float("nan"),
             over=max(over) if over else float("nan"),
             settle=max(st) if st else float("nan"),
             wall=min(wall), est=max(est), stalls=sum(r["stalls"] for r in rs))
    summary[(cond, mode)] = s
    print(f"| {cond} | {'標準' if mode == 'std' else 'ランダム'} | {s['ok']} | "
          f"{f(s['total'])} / {f(s['total_max'])} | {f(s['stop'])} | {f(s['over'])} | "
          f"{f(s['settle'], 2)} | {f(s['wall'])} | {f(s['est'])} | {s['stalls']} |")

print("\n## baseline との差（同じ開始位置の種類どうし。正 = baseline より大きい）\n")
print("| 条件 | 開始 | 成功 | 合計 s | 停止誤差 mm | 行き過ぎ mm | 整定 s | 壁との最小距離 mm | 推定誤差ピーク mm | 停止の監視 |")
print("|---|---|---|---|---|---|---|---|---|---|")
for (cond, mode), s in summary.items():
    if cond == "combo_base" or ("combo_base", mode) not in summary:
        continue
    b = summary[("combo_base", mode)]
    d = lambda k, dd=1: f"{s[k] - b[k]:+.{dd}f}"
    print(f"| {cond} | {'標準' if mode == 'std' else 'ランダム'} | {s['ok']} (base {b['ok']}) | "
          f"{d('total')} | {d('stop')} | {d('over')} | {d('settle', 2)} | {d('wall')} | "
          f"{d('est')} | {s['stalls'] - b['stalls']:+d} |")
