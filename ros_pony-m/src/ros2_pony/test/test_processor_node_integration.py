import time

import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ros2_pony.processor_node import ProcessorNode


class TestProcessorNodeIntegration:
    """ProcessorNodeの統合テストクラス."""

    @pytest.fixture(autouse=True)
    def setup_teardown(self):
        """テストの前後処理."""
        rclpy.init()
        yield
        rclpy.shutdown()

    @pytest.fixture
    def processor_node(self):
        """ProcessorNodeのインスタンスを作成."""
        node = ProcessorNode()
        yield node
        node.destroy_node()

    @pytest.fixture
    def test_publisher(self):
        """テスト用のパブリッシャーを作成."""
        node = Node('test_publisher')
        publisher = node.create_publisher(String, 'input_topic', 10)
        yield publisher
        node.destroy_node()

    @pytest.fixture
    def test_subscriber(self):
        """テスト用のサブスクライバーを作成."""
        received_messages = []

        def callback(msg):
            received_messages.append(msg.data)

        node = Node('test_subscriber')
        node.create_subscription(String, 'output_topic', callback, 10)

        yield received_messages, node
        node.destroy_node()

    def test_message_flow(self, processor_node, test_publisher, test_subscriber):
        """メッセージの流れをテスト."""
        received_messages, subscriber_node = test_subscriber

        # テストメッセージを送信
        test_msg = String()
        test_msg.data = 'hello world'
        test_publisher.publish(test_msg)

        # メッセージの処理を待機（最大1秒）
        timeout = 1.0
        start_time = time.time()
        while len(received_messages) == 0 and (time.time() - start_time) < timeout:
            rclpy.spin_once(processor_node, timeout_sec=0.1)
            rclpy.spin_once(subscriber_node, timeout_sec=0.1)

        # 処理されたメッセージが受信されていることを確認
        assert len(received_messages) > 0
        assert received_messages[0] == 'HELLO WORLD'

    def test_multiple_messages(self, processor_node, test_publisher, test_subscriber):
        """複数のメッセージの処理をテスト."""
        received_messages, subscriber_node = test_subscriber

        # 複数のテストメッセージを送信
        test_messages = ['test1', 'test2', 'test3']

        for msg_data in test_messages:
            test_msg = String()
            test_msg.data = msg_data
            test_publisher.publish(test_msg)

            # 各メッセージの処理を待機
            timeout = 1.0
            start_time = time.time()
            while (len(received_messages) < len(test_messages) and
                   (time.time() - start_time) < timeout):
                rclpy.spin_once(processor_node, timeout_sec=0.1)
                rclpy.spin_once(subscriber_node, timeout_sec=0.1)

        # すべてのメッセージが処理されていることを確認
        assert len(received_messages) == len(test_messages)
        for i, expected in enumerate(test_messages):
            assert received_messages[i] == expected.upper()

    def test_empty_message(self, processor_node, test_publisher, test_subscriber):
        """空のメッセージの処理をテスト."""
        received_messages, subscriber_node = test_subscriber

        # 空のメッセージを送信
        test_msg = String()
        test_msg.data = ''
        test_publisher.publish(test_msg)

        # メッセージの処理を待機（最大1秒）
        timeout = 1.0
        start_time = time.time()
        while len(received_messages) == 0 and (time.time() - start_time) < timeout:
            rclpy.spin_once(processor_node, timeout_sec=0.1)
            rclpy.spin_once(subscriber_node, timeout_sec=0.1)

        # 空のメッセージが処理されていることを確認
        assert len(received_messages) > 0
        assert received_messages[0] == ''

    def test_special_characters(self, processor_node, test_publisher, test_subscriber):
        """特殊文字を含むメッセージの処理をテスト."""
        received_messages, subscriber_node = test_subscriber

        # 特殊文字を含むメッセージを送信
        test_msg = String()
        test_msg.data = 'test@#$%^&*()'
        test_publisher.publish(test_msg)

        # メッセージの処理を待機（最大1秒）
        timeout = 1.0
        start_time = time.time()
        while len(received_messages) == 0 and (time.time() - start_time) < timeout:
            rclpy.spin_once(processor_node, timeout_sec=0.1)
            rclpy.spin_once(subscriber_node, timeout_sec=0.1)

        # 特殊文字が正しく処理されていることを確認
        assert len(received_messages) > 0
        assert received_messages[0] == 'TEST@#$%^&*()' 