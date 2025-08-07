#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ros2_pony.msg import SystemStatus
import time


class SystemStatusSender(Node):
    """SystemStatusメッセージを送信するテスト用ノード."""

    def __init__(self):
        super().__init__('system_status_sender')
        
        # パブリッシャーの設定
        self._system_status_publisher = self.create_publisher(
            SystemStatus,
            'system_status',
            10
        )
        
        # タイマー
        self._timer = self.create_timer(1.0, self._publish_system_status)
        
        self.get_logger().info('SystemStatusSender has been started')

    def _publish_system_status(self):
        """システム状態を送信する."""
        try:
            status_msg = SystemStatus()
            status_msg.system_ready = True  # システム準備完了
            status_msg.errors = []  # エラーなし
            status_msg.joint_positions = [0.0] * 12  # 12個の関節位置
            
            self._system_status_publisher.publish(status_msg)
            
            self.get_logger().info('Published SystemStatus: system_ready=true')
            
        except Exception as e:
            self.get_logger().error(f'Error publishing system status: {e}')


def main(args=None):
    """メイン関数."""
    rclpy.init(args=args)
    
    sender = SystemStatusSender()
    
    try:
        rclpy.spin(sender)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        sender.get_logger().error(f'Unexpected error: {e}')
    finally:
        sender.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 