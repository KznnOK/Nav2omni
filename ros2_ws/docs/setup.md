# 環境構築（別PCで再現する手順）

動作確認した環境: Ubuntu 22.04.5 / ROS 2 Humble / Gazebo Classic 11.10.2 / Python 3.10.12

## apt パッケージ
ノード・launch・package.xml が使うものと、手で入れたものの一覧。
```bash
sudo apt install \
  ros-humble-desktop python3-colcon-common-extensions python3-rosdep \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins ros-humble-gazebo-ros2-control \
  ros-humble-robot-localization ros-humble-laser-filters ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui ros-humble-xacro ros-humble-rviz2 \
  ros-humble-joy ros-humble-teleop-twist-joy ros-humble-teleop-twist-keyboard ros-humble-twist-mux \
  ros-humble-tf-transformations \
  python3-numpy python3-yaml python3-pil python3-matplotlib python3-pytest
```
- 作業用PCには上のほかに rqt 系、slam-toolbox、turtlebot3 系、ros2-control 系も入っているが、このワークスペースのコードは使っていない。
- 入れ漏れは `rosdep install --from-paths src --ignore-src -r -y` でも拾える。
- `dpkg -l | grep ros-humble`（438件）の全量は `docs/ros-humble-packages.txt`（名前=版）。

## pip パッケージ
ノードが import する Python パッケージはすべて apt で足りる（numpy, yaml, PIL, matplotlib, pytest）。pip で必須のものはない。
作業用PCの pip には numpy 1.26.4 が入っているが、apt 版（1.21.5）でも動く想定。
結果の表づくりに `results/phase5_single_error/p5_table.py` を使うだけなら追加は不要。

## ビルドと実行
```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```
- `build/ install/ log/` は `.gitignore` 済みで、コミットしない。
- Gazebo を GUI なしで走らせる場合は `gzserver` だけで足りる（試験は `results/phase5_single_error/run_conditions.sh`）。
