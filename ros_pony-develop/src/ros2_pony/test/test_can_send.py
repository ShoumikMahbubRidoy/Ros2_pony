#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from can_msgs.msg import Frame
import time
import argparse
import signal
import sys


class SimpleCanSender(Node):
    """シンプルなCAN送信テストノード."""

    def __init__(self, can_id=0x668, data=None, interval=0.1):
        super().__init__('simple_can_sender')
        
        # パラメータの設定
        self.can_id = can_id
        self.data = data if data else [0, 0, 0, 0, 0, 255, 0, 255]
        self.interval = interval
        
        # パブリッシャーの設定
        self._can_send_publisher = self.create_publisher(
            Frame,
            'can_send',
            10
        )
        
        # 送信タイマー
        self._send_timer = self.create_timer(self.interval, self._send_message)
        
        self.sent_count = 0
        self.get_logger().info(f'SimpleCanSender started with ID: {hex(can_id)}, Data: {data}, Interval: {interval}s')

    def _send_message(self):
        """CANメッセージを送信."""
        try:
            msg = Frame()
            msg.id = self.can_id
            msg.dlc = len(self.data)
            msg.data = self.data
            msg.is_error = False
            msg.is_rtr = False
            msg.is_extended = False
            
            self._can_send_publisher.publish(msg)
            self.sent_count += 1
            
            self.get_logger().info(f'Sent #{self.sent_count}: ID={hex(msg.id)}, Data={msg.data}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to send message: {e}')


def signal_handler(signum, frame):
    """シグナルハンドラー."""
    print(f"\nシグナル {signum} を受信しました。終了中...")
    sys.exit(0)

def main(args=None):
    """メイン関数."""
    # シグナルハンドラーの設定
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    rclpy.init(args=args)

    # コマンドライン引数の解析
    parser = argparse.ArgumentParser(description='Simple CAN Sender')
    parser.add_argument('--can-id', type=lambda x: int(x, 0), default=0x668, 
                       help='CAN ID (hex format, default: 0x668)')
    parser.add_argument('--data', type=lambda x: [int(y, 16) for y in x.split(',')], 
                       default=[0, 0, 0, 0, 0, 255, 0, 255],
                       help='Data bytes (hex format, comma-separated, default: 00,00,00,00,00,ff,00,ff)')
    parser.add_argument('--interval', type=float, default=0.1, 
                       help='Send interval in seconds (default: 0.1)')
    
    # ROS2の引数を除外してパース
    ros_args = []
    other_args = []
    for arg in sys.argv[1:]:
        if arg.startswith('--ros-args'):
            ros_args.append(arg)
        else:
            other_args.append(arg)
    
    parsed_args = parser.parse_args(other_args)
    
    # ノードの作成
    sender = SimpleCanSender(
        can_id=parsed_args.can_id,
        data=parsed_args.data,
        interval=parsed_args.interval
    )

    try:
        rclpy.spin(sender)
    except KeyboardInterrupt:
        print("\nKeyboardInterruptを受信しました。終了中...")
    except Exception as e:
        print(f"エラーが発生しました: {e}")
    finally:
        try:
            sender.destroy_node()
            rclpy.shutdown()
        except Exception as e:
            print(f"終了処理中にエラーが発生しました: {e}")
            sys.exit(1)


if __name__ == '__main__':
    main() 