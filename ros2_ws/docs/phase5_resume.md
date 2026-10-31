# フェーズ5（誤差を1つずつ入れる試験）再開メモ

ブランチ: `feature/phase5-error-study`
走行条件: 2.0 m/s・3 m/s²・3.5 m/s²（既定値）、標準開始位置。各条件で LiDAR 補正あり3回・なし3回。
条件ファイル: `results/phase5_single_errors/conditions/*.yaml`（値はすべて仮値、`make_conditions.py` で生成）
結果: `results/phase5_single_error/`（`runs.csv`、`timeseries/`、`table.md`、`summary.json`）

## 状態: 全16条件 × 6回 = 96回 完了
未完了の条件なし。`runs.csv` は結合済み（66回 + `runs_resume.csv` の30回、ラベルの重複なし）、
`table.md` / `summary.json` は96回分から作り直した。`runs_resume.csv` は再開分の原本として残してある。
drive_mu2_0.4 は6回すべて失敗（GO_SAKURA / BACKOFF_1 でタイムアウト＋壁接触）。仮値が大きすぎる可能性があるため、結論の前に値を見直すこと。

## 再開コマンド（個別の条件だけやり直す場合）
```bash
cd /home/ubuntu/robocon_study/ros2/omni_nav2/ros2_ws
source /opt/ros/humble/setup.bash && colcon build && source install/setup.bash
CONDS="drive_mu2_0.4" results/phase5_single_error/run_conditions.sh | tee /tmp/phase5_resume.out   # 結果は runs_resume.csv に追記
python3 results/phase5_single_error/p5_table.py <結合したruns.csv> results/phase5_single_error/timeseries > results/phase5_single_error/table.md
results/phase5_single_error/stop_sim.sh   # 停止
```

## 摩擦による加速度上限（追加）
- 実装: `trajectory.py` の `FrictionLimit`、パラメータは `sakura_mission.yaml`（`friction_limit_enabled` 既定 false）。設計は `docs/friction_accel_limit_proposal.md`。
- 停止の見張り: 指令ありで3秒動かないと、解放して経路を作り直し、2回で FAILED（`stall_*` パラメータ）。
- 試験: 駆動輪 mu2 = 0.4 / 0.6 / 0.8、上限有効（`mu_drive` = mu2）、補正あり各3回。結果は `runs_friclim.csv` / `table_friclim.md`。全部 3/3 成功。
- 例: `CONDS=drive_mu2_0.4 LIDARS=true SUFFIX=_friclim EXTRA_SET="--set friction_limit_enabled=true --set mu_drive=0.4" OUT=<csv> results/phase5_single_error/run_conditions.sh`

- 結論: 計画の摩擦係数は実測値より低めに置く（`docs/friction_accel_limit_proposal.md` 6章）。補正指令には摩擦上限、経路追従の指令は1.3倍の余裕を残す。設計の全体像は `docs/design_overview.md`。

## 誤差の組み合わせ試験（完了）
- 結果と考察は `results/phase5_combo/README.md`（`table.md`、`runs.csv`）。15/15 成功、停止の監視は0回。
- baseline からの悪化は復路の行き過ぎ（real +8 mm、severe +57〜80 mm）と推定誤差のピーク（severe +36〜44 mm）が目立つ。計測輪の浮きが主因と考えられる（切り分けは未実施）。
