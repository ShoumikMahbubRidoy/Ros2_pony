#!/bin/bash

# motion_plannerノードのテスト実行スクリプト

echo "=== Motion Planner Node Test ==="
echo "Starting motion_planner_node..."

# 1つ目のターミナルでmotion_plannerノードを起動
gnome-terminal --title="Motion Planner Node" -- bash -c "
    cd /home/k-okina-d1/work/ros_pony
    source install/setup.bash
    ros2 run ros2_pony motion_planner_node --ros-args -p max_velocity:=500.0 -p max_acceleration:=500.0
    read -p 'Press Enter to close...'
"

# 少し待ってからテストノードを起動
sleep 3

echo "Starting motion_planner_tester..."

# 2つ目のターミナルでテストノードを起動
gnome-terminal --title="Motion Planner Tester" -- bash -c "
    cd /home/k-okina-d1/work/ros_pony
    source install/setup.bash
    python3 src/ros2_pony/test/test_motion_planner.py
    read -p 'Press Enter to close...'
"

echo "Test setup completed."
echo "Check the terminals for test results." 