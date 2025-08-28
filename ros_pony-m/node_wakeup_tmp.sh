#!/bin/bash

# printf "password: "
# read  password
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

# 仮想環境をアクティベート
source venv/bin/activate

# ROS2環境をセットアップ
source install/setup.bash

# 1. CANノードの起動
# echo "$password" | sudo -S \
# PYTHONPATH=$PYTHONPATH:$(pwd)/venv/lib/python3.12/site-packages \
#     python3 src/ros2_pony/ros2_pony/drive_usb2can_node.py \
#     --ros-args -p can_interface:=socketcan -p enable_system_setup:=true &

# 2. CANコンバーターノードの起動
python3 src/ros2_pony/ros2_pony/joint_can_converter_node.py \
--ros-args -p mode_id:=6 -p device_id_start:=104 &

# 3. モーションプランナーノードの起動
python3 src/ros2_pony/ros2_pony/motion_planner_node.py &


# 4. システム初期化
ros2 service call /initialize_system std_srvs/srv/Trigger 
ros2 service call /initialize_motion std_srvs/srv/Trigger 