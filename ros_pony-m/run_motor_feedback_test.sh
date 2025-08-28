#!/bin/bash

# 仮想環境をアクティベート
source venv/bin/activate

# ROS2環境をセットアップ
source install/setup.bash

# モータフィードバックテストノードを実行
exec python3 test_motor_feedback.py "$@" 