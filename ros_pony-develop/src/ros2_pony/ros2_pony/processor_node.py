#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# カスタムメッセージのインポート（ビルド後に利用可能）
try:
    from ros2_pony.msg import AimDegree
    CUSTOM_MSG_AVAILABLE = True
except ImportError:
    CUSTOM_MSG_AVAILABLE = False
    # フォールバック用のダミークラス
    class AimDegree:
        def __init__(self):
            self.jointname = ""
            self.degree = 0.0


class ProcessorNode(Node):
    """トピックを受信し、加工して送信するノード."""

    def __init__(self):
        super().__init__('processor_node')

        if CUSTOM_MSG_AVAILABLE:
            # カスタムメッセージ用のパブリッシャー
            self._publisher = self.create_publisher(
                AimDegree,
                'aim_degree_output',
                10
            )

            # カスタムメッセージ用のサブスクライバー
            self._subscription = self.create_subscription(
                AimDegree,
                'aim_degree_input',
                self._message_callback,
                10
            )

            self.get_logger().info('ProcessorNode has been started with AimDegree messages')
        else:
            # フォールバック用のStringメッセージ
            self._publisher = self.create_publisher(
                String,
                'output_topic',
                10
            )

            self._subscription = self.create_subscription(
                String,
                'input_topic',
                self._fallback_message_callback,
                10
            )

            self.get_logger().info('ProcessorNode has been started with String messages (fallback)')

    def _message_callback(self, msg):
        """受信したAimDegreeメッセージを処理するコールバック."""
        self.get_logger().info(f'Received AimDegree: joint={msg.jointname}, degree={msg.degree}')

        # メッセージの加工
        processed_msg = self._process_aim_degree_message(msg)

        # 加工したメッセージを送信
        self._publisher.publish(processed_msg)

        self.get_logger().info(f'Published processed AimDegree: joint={processed_msg.jointname}, degree={processed_msg.degree}')

    def _fallback_message_callback(self, msg):
        """フォールバック用のStringメッセージコールバック."""
        self.get_logger().info(f'Received: {msg.data}')

        # メッセージの加工
        processed_data = self._process_message(msg.data)

        # 加工したメッセージを送信
        output_msg = String()
        output_msg.data = processed_data
        self._publisher.publish(output_msg)

        self.get_logger().info(f'Published: {processed_data}')

    def _process_aim_degree_message(self, msg):
        """AimDegreeメッセージを加工する処理.

        Args:
            msg (AimDegree): 入力のAimDegreeメッセージ.

        Returns:
            AimDegree: 加工されたAimDegreeメッセージ.

        """
        # 新しいAimDegreeメッセージを作成
        processed_msg = AimDegree()
        
        # 関節名を大文字に変換
        processed_msg.jointname = msg.jointname.upper()
        
        # 角度を2倍にする（例：実際の用途に応じて変更してください）
        processed_msg.degree = msg.degree * 2.0
        
        return processed_msg

    def _process_message(self, data):
        """メッセージを加工する処理.

        Args:
            data (str): 入力データ.

        Returns:
            str: 加工されたデータ.

        """
        # 基本的な加工処理（例：大文字変換）
        # 実際の用途に応じて、この処理を変更してください
        return data.upper()


def main(args=None):
    """メイン関数."""
    rclpy.init(args=args)

    node = ProcessorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 