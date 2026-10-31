# フェーズ5（誤差を1つずつ入れる試験）再開メモ

ブランチ: `feature/phase5-error-study`
走行条件: 2.0 m/s・3 m/s²・3.5 m/s²（既定値）、標準開始位置。各条件で LiDAR 補正あり3回・なし3回。
条件ファイル: `results/phase5_single_errors/conditions/*.yaml`（値はすべて仮値、`make_conditions.py` で生成）
結果: `results/phase5_single_error/`（`runs.csv`、`timeseries/`、`table.md`、`summary.json`）

## 状態: 全16条件 × 6回 = 96回 完了
未完了の条件なし。`runs.csv` は結合済み（66回 + `runs_resume.csv` の30回、ラベルの重複なし）、
`table.md` / `summary.json` は96回分から作り直した。`runs_resume.csv` は再開分の原本として残してある。
drive_slip は6回すべて失敗（GO_SAKURA / BACKOFF_1 でタイムアウト＋壁接触）。仮値が大きすぎる可能性があるため、結論の前に値を見直すこと。

## 再開コマンド（個別の条件だけやり直す場合）
```bash
cd /home/ubuntu/robocon_study/ros2/omni_nav2/ros2_ws
source /opt/ros/humble/setup.bash && colcon build && source install/setup.bash
CONDS="drive_slip" results/phase5_single_error/run_conditions.sh | tee /tmp/phase5_resume.out   # 結果は runs_resume.csv に追記
python3 results/phase5_single_error/p5_table.py <結合したruns.csv> results/phase5_single_error/timeseries > results/phase5_single_error/table.md
results/phase5_single_error/stop_sim.sh   # 停止
```
