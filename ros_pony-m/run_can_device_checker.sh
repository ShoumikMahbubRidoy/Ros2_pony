#!/bin/bash

# 仮想環境をアクティベート
source venv/bin/activate

# ROS2環境をセットアップ
source install/setup.bash

# 診断ノードを実行
exec python3 src/ros2_pony/test/can_device_checker.py "$@" 