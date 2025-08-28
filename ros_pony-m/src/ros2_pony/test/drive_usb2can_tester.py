#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from can_msgs.msg import Frame
from std_msgs.msg import String
import time
import threading
import sys
import os

# testディレクトリからros2_ponyパッケージをインポートするためのパス設定
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'ros2_pony'))


class DriveUsb2CanTester(Node):
    """drive_usb2canノードのテスト用ノード."""

    def __init__(self):
        super().__init__('drive_usb2can_tester')
        
        # サブスクライバーの設定
        self._can_received_subscription = self.create_subscription(
            Frame,
            'can_received',
            self._can_received_callback,
            10
        )
        
        self._can_status_subscription = self.create_subscription(
            String,
            'can_status',
            self._can_status_callback,
            10
        )
        
        # パブリッシャーの設定
        self._can_send_publisher = self.create_publisher(
            Frame,
            'can_send',
            10
        )
        
        # テスト用のタイマー
        self._test_timer = self.create_timer(5.0, self._send_test_message)
        
        # 受信メッセージのカウンター
        self.received_count = 0
        self.status_messages = []
        
        self.get_logger().info('DriveUsb2CanTester has been started')

    def _can_received_callback(self, msg: Frame):
        """CAN受信メッセージのコールバック."""
        self.received_count += 1
        self.get_logger().info(f'Received CAN message #{self.received_count}: ID={hex(msg.id)}, Data={bytes(msg.data).hex()}')

    def _can_status_callback(self, msg: String):
        """CAN状態メッセージのコールバック."""
        self.status_messages.append(msg.data)
        self.get_logger().info(f'CAN Status: {msg.data}')

    def _send_test_message(self):
        """テスト用のCANメッセージを送信."""
        test_msg = Frame()
        test_msg.id = 0x123  # テスト用ID
        test_msg.dlc = 4
        test_msg.data = [0x01, 0x02, 0x03, 0x04]  # テストデータ
        test_msg.is_error = False
        test_msg.is_rtr = False
        test_msg.is_extended = False
        
        self._can_send_publisher.publish(test_msg)
        self.get_logger().info(f'Sent test CAN message: ID={hex(test_msg.id)}, Data={test_msg.data}')


def main(args=None):
    """メイン関数."""
    rclpy.init(args=args)

    tester = DriveUsb2CanTester()

    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 