import pytest
import rclpy
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

from ros2_pony.processor_node import ProcessorNode


class TestProcessorNode:
    """ProcessorNodeのテストクラス."""

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

    def test_node_creation(self, processor_node):
        """ノードが正常に作成されることをテスト."""
        assert processor_node is not None
        assert processor_node.get_name() == 'processor_node'

    def test_subscription_creation(self, processor_node):
        """サブスクリプションが正常に作成されることをテスト."""
        # サブスクリプションが存在することを確認
        assert hasattr(processor_node, '_subscription')
        assert processor_node._subscription is not None

    def test_publisher_creation(self, processor_node):
        """パブリッシャーが正常に作成されることをテスト."""
        # パブリッシャーが存在することを確認
        assert hasattr(processor_node, '_publisher')
        assert processor_node._publisher is not None

    def test_message_processing(self, processor_node):
        """メッセージの加工処理をテスト."""
        # テスト用の入力メッセージ
        test_input = 'test message'

        # 加工処理のテスト
        processed = processor_node._process_message(test_input)

        # 期待される出力（例：大文字変換）
        expected_output = test_input.upper()
        assert processed == expected_output

    def test_message_callback(self, processor_node):
        """メッセージコールバックの動作をテスト."""
        # テスト用のメッセージを作成
        test_msg = String()
        test_msg.data = 'test callback'

        # コールバックを実行
        if CUSTOM_MSG_AVAILABLE:
            # AimDegree型が使える場合は、適切なメッセージを渡す
            # 例: test_msg = AimDegree(); test_msg.jointname = ...; test_msg.degree = ...
            processor_node._message_callback(test_msg)
        else:
            # String型の場合はフォールバックコールバックを使う
            processor_node._fallback_message_callback(test_msg)

        # パブリッシャーが呼ばれたことを確認
        # 実際のテストでは、パブリッシャーの呼び出しをモックするか、
        # 実際のメッセージ送信を待機する必要があります

    def test_empty_message_handling(self, processor_node):
        """空のメッセージの処理をテスト."""
        empty_input = ''
        processed = processor_node._process_message(empty_input)
        assert processed == ''

    def test_special_characters_handling(self, processor_node):
        """特殊文字を含むメッセージの処理をテスト."""
        special_input = 'test@#$%^&*()'
        processed = processor_node._process_message(special_input)
        expected_output = special_input.upper()
        assert processed == expected_output

    @pytest.mark.skipif(not CUSTOM_MSG_AVAILABLE, reason="Custom message not available")
    def test_aim_degree_message_processing(self, processor_node):
        """AimDegreeメッセージの加工処理をテスト."""
        # テスト用のAimDegreeメッセージを作成
        test_msg = AimDegree()
        test_msg.jointname = 'shoulder_joint'
        test_msg.degree = 45.0

        # 加工処理のテスト
        processed = processor_node._process_aim_degree_message(test_msg)

        # 期待される出力
        assert processed.jointname == 'SHOULDER_JOINT'  # 大文字変換
        assert processed.degree == 90.0  # 2倍

    @pytest.mark.skipif(not CUSTOM_MSG_AVAILABLE, reason="Custom message not available")
    def test_aim_degree_message_callback(self, processor_node):
        """AimDegreeメッセージコールバックの動作をテスト."""
        # テスト用のAimDegreeメッセージを作成
        test_msg = AimDegree()
        test_msg.jointname = 'elbow_joint'
        test_msg.degree = 30.0

        # コールバックを実行
        processor_node._message_callback(test_msg)

        # パブリッシャーが呼ばれたことを確認
        # 実際のテストでは、パブリッシャーの呼び出しをモックするか、
        # 実際のメッセージ送信を待機する必要があります 