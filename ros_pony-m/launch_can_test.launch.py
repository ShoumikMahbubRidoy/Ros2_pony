#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """CAN送信テスト用のランチファイル."""
    
    # パラメータの宣言
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='socketcan',
        description='CAN interface type (socketcan or usb2can)'
    )
    
    can_channel_arg = DeclareLaunchArgument(
        'can_channel',
        default_value='can0',
        description='CAN channel name'
    )
    
    can_bitrate_arg = DeclareLaunchArgument(
        'can_bitrate',
        default_value='1000000',
        description='CAN bitrate'
    )
    
    enable_system_setup_arg = DeclareLaunchArgument(
        'enable_system_setup',
        default_value='true',
        description='Enable CAN interface system setup'
    )
    
    # ノードの設定
    drive_usb2can_node = Node(
        package='ros2_pony',
        executable='drive_usb2can_node',
        name='drive_usb2can_node',
        parameters=[{
            'can_interface': LaunchConfiguration('can_interface'),
            'can_channel': LaunchConfiguration('can_channel'),
            'can_bitrate': LaunchConfiguration('can_bitrate'),
            'enable_system_setup': LaunchConfiguration('enable_system_setup'),
            'enable_diagnostics': True,
            'receive_timeout': 1.0
        }],
        output='screen'
    )
    
    simple_sender_node = Node(
        package='ros2_pony',
        executable='test_can_send.py',
        name='simple_can_sender',
        parameters=[{
            'can_id': 0x668,
            'data': [0, 0, 0, 0, 0, 255, 0, 255],
            'interval': 0.1
        }],
        output='screen'
    )
    
    return LaunchDescription([
        can_interface_arg,
        can_channel_arg,
        can_bitrate_arg,
        enable_system_setup_arg,
        drive_usb2can_node,
        simple_sender_node
    ]) 