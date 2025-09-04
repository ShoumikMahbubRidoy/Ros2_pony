- drive_usb2can_node
```
# 0x68を動かす
ros2 topic pub -1 /can_send can_msgs/msg/Frame "{id: 1640, dlc: 8, data: [255, 0, 0, 0, 0, 255, 0, 255], is_error: false, is_rtr: false, is_extended: false}"

# 0x68の初期位置オフセットをリセット
ros2 topic pub -1 /can_send can_msgs/msg/Frame "{id: 1384, dlc: 8, data: [1,0,0,0,0,0,0,0], is_error: false, is_rtr: false, is_extended: false}"
```

- joint_can_converter
```
ros2 topic pub -1 /joint_command sensor_msgs/msg/JointState "{name: ['front_left_hip'], position: [0]}"
```

- motion_planner_node
```
ros2 topic pub -1 /joint_target sensor_msgs/msg/JointState "{name: ['front_left_hip'], position: [0]}"

ros2 topic pub -1 /joint_target sensor_msgs/msg/JointState \
    "{name: ['front_left_hip', 'front_left_thigh', 'front_left_calf', \
            'front_right_hip', 'front_right_thigh', 'front_right_calf', \
            'rear_left_hip', 'rear_left_thigh', 'rear_left_calf', \
            'rear_right_hip', 'rear_right_thigh', 'rear_right_calf'], \
            position: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}"

ros2 topic pub --rate 1000 /joint_target sensor_msgs/msg/JointState \
    "{name: ['front_left_hip', 'front_left_thigh', 'front_left_calf', \
            'front_right_hip', 'front_right_thigh', 'front_right_calf', \
            'rear_left_hip', 'rear_left_thigh', 'rear_left_calf', \
            'rear_right_hip', 'rear_right_thigh', 'rear_right_calf'], \
            position: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}"            

```

- /motor_feedback_summary_jointstate
```
ros2 topic pub --rate 1 /motor_feedback_summary_jointstate sensor_msgs/msg/JointState \
    "{name: ['front_left_hip', 'front_left_thigh', 'front_left_calf', \
            'front_right_hip', 'front_right_thigh', 'front_right_calf', \
            'rear_left_hip', 'rear_left_thigh', 'rear_left_calf', \
            'rear_right_hip', 'rear_right_thigh', 'rear_right_calf'], \
            position: [1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], \
            velocity: [3, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], \
            effort:   [5, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}"
```