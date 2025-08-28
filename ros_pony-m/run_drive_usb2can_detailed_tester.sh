#!/bin/bash

# 仮想環境をアクティベート
source venv/bin/activate

# ROS2環境をセットアップ
source install/setup.bash

# 詳細テストノードを実行
exec python3 src/ros2_pony/test/drive_usb2can_detailed_tester.py "$@" 