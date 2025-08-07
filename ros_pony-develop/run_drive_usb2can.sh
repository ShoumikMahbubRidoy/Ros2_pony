#!/bin/bash

# 仮想環境をアクティベート
source venv/bin/activate

# ROS2環境をセットアップ
source install/setup.bash

# ノードを実行
exec python3 -m ros2_pony.drive_usb2can_node "$@" 