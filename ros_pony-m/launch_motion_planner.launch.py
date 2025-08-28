#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """motion_plannerノード用のランチファイル."""
    
    # パラメータの宣言
    max_velocity_arg = DeclareLaunchArgument(
        'max_velocity',
        default_value='1000.0',
        description='Maximum motor velocity [ERPM]'
    )
    
    max_acceleration_arg = DeclareLaunchArgument(
        'max_acceleration',
        default_value='1000.0',
        description='Maximum motor acceleration [ERPM/s²]'
    )
    
    can_send_frequency_arg = DeclareLaunchArgument(
        'can_send_frequency',
        default_value='10.0',
        description='CAN send frequency [Hz]'
    )
    
    status_publish_frequency_arg = DeclareLaunchArgument(
        'status_publish_frequency',
        default_value='1.0',
        description='Status publish frequency [Hz]'
    )
    
    # motion_plannerノード
    motion_planner_node = Node(
        package='ros2_pony',
        executable='motion_planner_node',
        name='motion_planner_node',
        output='screen',
        parameters=[{
            'max_velocity': LaunchConfiguration('max_velocity'),
            'max_acceleration': LaunchConfiguration('max_acceleration'),
            'can_send_frequency': LaunchConfiguration('can_send_frequency'),
            'status_publish_frequency': LaunchConfiguration('status_publish_frequency'),
        }],
        remappings=[
            ('joint_target', '/joint_target'),
            ('joint_command', '/joint_command'),
            ('joint_states', '/joint_states'),
            ('motor_feedback', '/motor_feedback'),
            ('system_status', '/system_status'),
            ('motion_status', '/motion_status'),
            ('initialize_motion', '/initialize_motion'),
        ]
    )
    
    return LaunchDescription([
        max_velocity_arg,
        max_acceleration_arg,
        can_send_frequency_arg,
        status_publish_frequency_arg,
        motion_planner_node,
    ]) 