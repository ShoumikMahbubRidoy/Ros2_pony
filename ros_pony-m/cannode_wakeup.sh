#!/bin/bash

# 仮想環境をアクティベート
source venv/bin/activate

# ROS2環境をセットアップ
source install/setup.bash

# ノードを実行
PYTHONPATH=$PYTHONPATH:$(pwd)/venv/lib/python3.12/site-packages \
    python3 src/ros2_pony/ros2_pony/drive_usb2can_node.py \
    --ros-args -p can_interface:=socketcan -p enable_system_setup:=true 