#!/bin/bash

# motion_planner_nodeの統合テスト実行スクリプト

echo "=== Motion Planner Integration Test ==="
echo "This test will:"
echo "1. Start motion_planner_node"
echo "2. Call initialize_motion service"
echo "3. Send joint target message"
echo "4. Check motion status"

# 1つ目のターミナルでmotion_plannerノードを起動
gnome-terminal --title="Motion Planner Node" -- bash -c "
    cd /home/k-okina-d1/work/ros_pony
    source install/setup.bash
    ros2 run ros2_pony motion_planner_node
    read -p 'Press Enter to close...'
"

# 少し待ってから初期化サービスを呼び出し
sleep 3

echo "Calling initialize_motion service..."

# 2つ目のターミナルで初期化サービスを呼び出し
gnome-terminal --title="Initialize Service" -- bash -c "
    cd /home/k-okina-d1/work/ros_pony
    source install/setup.bash
    echo 'Calling initialize_motion service...'
    ros2 service call /initialize_motion std_srvs/srv/Trigger
    echo 'Initialization service called. Check motion_planner logs.'
    read -p 'Press Enter to close...'
"

# さらに待ってからテストメッセージを送信
sleep 3

echo "Sending test joint target..."

# 3つ目のターミナルでテストメッセージを送信
gnome-terminal --title="Test Message Sender" -- bash -c "
    cd /home/k-okina-d1/work/ros_pony
    source install/setup.bash
    echo 'Sending joint target message...'
    ros2 topic pub -1 /joint_target sensor_msgs/msg/JointState '{name: [\"front_left_hip\"], position: [0.5]}'
    echo 'Joint target sent. Check motion_planner logs for results.'
    read -p 'Press Enter to close...'
"

echo "Test setup completed."
echo "Check the terminals for test results."
echo ""
echo "Alternative initialization methods:"
echo "1. Service call: ros2 service call /initialize_motion std_srvs/srv/Trigger"
echo "2. SystemStatus: ros2 topic pub /system_status ros2_pony/msg/SystemStatus '{system_ready: true, errors: [], joint_positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'" 