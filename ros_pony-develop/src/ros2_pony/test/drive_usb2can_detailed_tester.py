#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from can_msgs.msg import Frame
from std_msgs.msg import String
import time
import threading
import argparse
import sys
import os

# testディレクトリからros2_ponyパッケージをインポートするためのパス設定
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'ros2_pony'))


class DriveUsb2CanDetailedTester(Node):
    """詳細なCANフレーム設定が可能なdrive_usb2canノードのテスト用ノード."""

    def __init__(self, can_id=0x123, dlc=4, data=None, extended=False, remote=False, error=False, interval=5.0):
        super().__init__('drive_usb2can_detailed_tester')
        
        # パラメータの設定
        self.can_id = can_id
        self.dlc = dlc
        self.data = data if data else [0x01, 0x02, 0x03, 0x04]
        self.extended = extended
        self.remote = remote
        self.error = error
        self.interval = interval
        
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
        self._test_timer = self.create_timer(self.interval, self._send_test_message)
        
        # 受信メッセージのカウンター
        self.received_count = 0
        self.sent_count = 0
        self.status_messages = []
        
        self.get_logger().info(f'DriveUsb2CanDetailedTester has been started with:')
        self.get_logger().info(f'  CAN ID: {hex(self.can_id)}')
        self.get_logger().info(f'  DLC: {self.dlc}')
        self.get_logger().info(f'  Data: {[hex(x) for x in self.data]}')
        self.get_logger().info(f'  Extended: {self.extended}')
        self.get_logger().info(f'  Remote: {self.remote}')
        self.get_logger().info(f'  Error: {self.error}')
        self.get_logger().info(f'  Interval: {self.interval}s')

    def _can_received_callback(self, msg: Frame):
        """CAN受信メッセージのコールバック."""
        self.received_count += 1
        self.get_logger().info(f'Received CAN message #{self.received_count}:')
        self.get_logger().info(f'  ID: {hex(msg.id)}')
        self.get_logger().info(f'  DLC: {msg.dlc}')
        self.get_logger().info(f'  Data: {[hex(x) for x in msg.data]}')
        self.get_logger().info(f'  Extended: {msg.is_extended}')
        self.get_logger().info(f'  Remote: {msg.is_rtr}')
        self.get_logger().info(f'  Error: {msg.is_error}')

    def _can_status_callback(self, msg: String):
        """CAN状態メッセージのコールバック."""
        self.status_messages.append(msg.data)
        self.get_logger().info(f'CAN Status: {msg.data}')

    def _send_test_message(self):
        """テスト用のCANメッセージを送信."""
        test_msg = Frame()
        test_msg.id = self.can_id
        test_msg.dlc = self.dlc
        
        # send.pyと同様のシンプルなデータ処理
        # データが8バイト未満の場合は0で埋める
        data_list = list(self.data)
        while len(data_list) < 8:
            data_list.append(0)
        test_msg.data = data_list[:8]  # 最大8バイトに制限
        
        test_msg.is_error = self.error
        test_msg.is_rtr = self.remote
        test_msg.is_extended = self.extended
        
        self._can_send_publisher.publish(test_msg)
        self.sent_count += 1
        
        self.get_logger().info(f'Sent CAN message #{self.sent_count}:')
        self.get_logger().info(f'  ID: {hex(test_msg.id)}')
        self.get_logger().info(f'  DLC: {test_msg.dlc}')
        self.get_logger().info(f'  Data: {[hex(x) for x in test_msg.data]}')
        self.get_logger().info(f'  Extended: {test_msg.is_extended}')
        self.get_logger().info(f'  Remote: {test_msg.is_rtr}')
        self.get_logger().info(f'  Error: {test_msg.is_error}')

    def send_custom_message(self, can_id, dlc, data, extended=False, remote=False, error=False):
        """カスタムCANメッセージを送信."""
        custom_msg = Frame()
        custom_msg.id = can_id
        custom_msg.dlc = dlc
        
        # send.pyと同様のシンプルなデータ処理
        data_list = list(data)
        while len(data_list) < 8:
            data_list.append(0)
        custom_msg.data = data_list[:8]  # 最大8バイトに制限
        
        custom_msg.is_error = error
        custom_msg.is_rtr = remote
        custom_msg.is_extended = extended
        
        self._can_send_publisher.publish(custom_msg)
        self.sent_count += 1
        
        self.get_logger().info(f'Sent custom CAN message #{self.sent_count}:')
        self.get_logger().info(f'  ID: {hex(custom_msg.id)}')
        self.get_logger().info(f'  DLC: {custom_msg.dlc}')
        self.get_logger().info(f'  Data: {[hex(x) for x in custom_msg.data]}')
        self.get_logger().info(f'  Extended: {custom_msg.is_extended}')
        self.get_logger().info(f'  Remote: {custom_msg.is_rtr}')
        self.get_logger().info(f'  Error: {custom_msg.is_error}')


def main(args=None):
    """メイン関数."""
    rclpy.init(args=args)

    # コマンドライン引数の解析
    parser = argparse.ArgumentParser(description='CAN Detailed Tester')
    parser.add_argument('--can-id', type=lambda x: int(x, 0), default=0x123, 
                       help='CAN ID (hex format, default: 0x123)')
    parser.add_argument('--dlc', type=int, default=4, 
                       help='Data Length Code (default: 4)')
    parser.add_argument('--data', type=lambda x: [int(y, 16) for y in x.split(',')], 
                       default=[0x01, 0x02, 0x03, 0x04],
                       help='Data bytes (hex format, comma-separated, default: 01,02,03,04)')
    parser.add_argument('--extended', action='store_true', 
                       help='Use extended frame format')
    parser.add_argument('--remote', action='store_true', 
                       help='Send remote frame')
    parser.add_argument('--error', action='store_true', 
                       help='Send error frame')
    parser.add_argument('--interval', type=float, default=5.0, 
                       help='Send interval in seconds (default: 5.0)')
    
    # ROS2の引数を除外してパース
    import sys
    ros_args = []
    other_args = []
    for arg in sys.argv[1:]:
        if arg.startswith('--ros-args'):
            ros_args.append(arg)
        else:
            other_args.append(arg)
    
    parsed_args = parser.parse_args(other_args)
    
    # ノードの作成
    tester = DriveUsb2CanDetailedTester(
        can_id=parsed_args.can_id,
        dlc=parsed_args.dlc,
        data=parsed_args.data,
        extended=parsed_args.extended,
        remote=parsed_args.remote,
        error=parsed_args.error,
        interval=parsed_args.interval
    )

    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 