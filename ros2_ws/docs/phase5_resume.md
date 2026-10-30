# フェーズ5（誤差を1つずつ入れる試験）再開メモ

ブランチ: `feature/phase5-error-study`
走行条件: 2.0 m/s・3 m/s²・3.5 m/s²（既定値）、標準開始位置。各条件で LiDAR 補正あり3回・なし3回。
条件ファイル: `results/phase5_single_errors/conditions/*.yaml`（値はすべて仮値、`make_conditions.py` で生成）
結果: `results/phase5_single_error/`（`runs.csv`、`timeseries/`、`table.md`、`summary.json`）

## 完了（11条件 × 6回 = 66回）
baseline, imu_bias, imu_drift, trk_diam_p1, trk_diam_m1, trk_angle_1deg, trk_pos_3mm,
trk_lift, trk_quant, lidar_pitch_0.5, lidar_pitch_1

## 未完了（5条件）
lidar_roll_0.5, lidar_roll_1, lidar_yaw_1, lidar_pos_5mm, drive_slip

## 再開コマンド
```bash
cd /home/ubuntu/robocon_study/ros2/omni_nav2/ros2_ws
source /opt/ros/humble/setup.bash && colcon build && source install/setup.bash
CONDS="lidar_roll_0.5 lidar_roll_1 lidar_yaw_1 lidar_pos_5mm drive_slip" \
  results/phase5_single_error/run_conditions.sh | tee /tmp/phase5_resume.out
```
- 結果は `results/phase5_single_error/runs_resume.csv` と `timeseries/` に追記される。
- 起動ログは `/tmp/phase5_logs/`。spawn 成功と AMCL 起動を確認し、失敗したら最大3回起動し直す。
- 終わったら `runs.csv` と `runs_resume.csv` を結合し、表を作り直す:
```bash
python3 results/phase5_single_error/p5_table.py <結合したruns.csv> results/phase5_single_error/timeseries
```
- 停止: `results/phase5_single_error/stop_sim.sh`

## 注意
- 1回目のバッチで trk_diam_p1 / trk_diam_m1 は Gazebo の spawn サービスが30 s以内に出ず失敗した
  （環境側の単発の不具合）。その分は破棄し、spawn確認付きのスクリプトでやり直した結果を runs.csv に入れている。
- 未回答: ランダム開始で回転区間が長くなった原因は、88a6b76（指令の加速度制限の修正）以降、後退・再接近の
  止まり際で目標の8〜12 mm手前から0.02〜0.03 m/sで這う時間が増えたため（標準開始でも同じ）。報告待ち。
