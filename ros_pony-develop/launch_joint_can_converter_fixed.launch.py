#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """joint_can_converterノードのlaunchファイル（修正版）."""
    
    # パラメータの宣言
    mode_id_arg = DeclareLaunchArgument(
        'mode_id',
        default_value='6',
        description='CAN mode ID'
    )
    
    device_id_start_arg = DeclareLaunchArgument(
        'device_id_start',
        default_value='104',  # 0x68
        description='Starting device ID'
    )
    
    num_motors_arg = DeclareLaunchArgument(
        'num_motors',
        default_value='12',
        description='Number of motors'
    )
    
    position_scale_arg = DeclareLaunchArgument(
        'position_scale',
        default_value='0.01',
        description='Position scale factor'
    )
    
    velocity_scale_arg = DeclareLaunchArgument(
        'velocity_scale',
        default_value='0.1',
        description='Velocity scale factor'
    )
    
    acceleration_scale_arg = DeclareLaunchArgument(
        'acceleration_scale',
        default_value='0.1',
        description='Acceleration scale factor'
    )
    
    default_velocity_arg = DeclareLaunchArgument(
        'default_velocity',
        default_value='1000',
        description='Default velocity (ERPM)'
    )
    
    default_acceleration_arg = DeclareLaunchArgument(
        'default_acceleration',
        default_value='1000',
        description='Default acceleration (ERPM/s2)'
    )
    
    homing_timeout_arg = DeclareLaunchArgument(
        'homing_timeout',
        default_value='30.0',
        description='Homing timeout (seconds)'
    )
    
    # joint_can_converterノード（直接Python実行）
    joint_can_converter_node = ExecuteProcess(
        cmd=[
            'python3', 
            'src/ros2_pony/ros2_pony/joint_can_converter_node.py',
            '--ros-args',
            '-p', f'mode_id:={LaunchConfiguration("mode_id")}',
            '-p', f'device_id_start:={LaunchConfiguration("device_id_start")}',
            '-p', f'num_motors:={LaunchConfiguration("num_motors")}',
            '-p', f'position_scale:={LaunchConfiguration("position_scale")}',
            '-p', f'velocity_scale:={LaunchConfiguration("velocity_scale")}',
            '-p', f'acceleration_scale:={LaunchConfiguration("acceleration_scale")}',
            '-p', f'default_velocity:={LaunchConfiguration("default_velocity")}',
            '-p', f'default_acceleration:={LaunchConfiguration("default_acceleration")}',
            '-p', f'homing_timeout:={LaunchConfiguration("homing_timeout")}',
            '-r', 'joint_command:=/joint_command',
            '-r', 'joint_states:=/joint_states',
            '-r', 'motor_feedback:=/motor_feedback',
            '-r', 'system_status:=/system_status',
            '-r', 'initialization_status:=/initialization_status',
            '-r', 'can_send:=/can_send',
            '-r', 'can_received:=/can_received',
            '-r', 'initialize_system:=/initialize_system',
        ],
        output='screen',
        cwd='/home/k-okina-d1/work/ros_pony'
    )
    
    return LaunchDescription([
        mode_id_arg,
        device_id_start_arg,
        num_motors_arg,
        position_scale_arg,
        velocity_scale_arg,
        acceleration_scale_arg,
        default_velocity_arg,
        default_acceleration_arg,
        homing_timeout_arg,
        joint_can_converter_node,
    ]) 