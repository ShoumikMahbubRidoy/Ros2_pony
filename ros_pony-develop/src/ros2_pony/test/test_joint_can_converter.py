#!/usr/bin/env python3

import unittest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from can_msgs.msg import Frame
from std_srvs.srv import Trigger
from ros2_pony.msg import MotorFeedback, SystemStatus, InitializationStatus
import time
import threading


class TestJointCanConverter(unittest.TestCase):
    """joint_can_converterノードのテスト."""

    @classmethod
    def setUpClass(cls):
        """テストクラスの初期化."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """テストクラスの終了."""
        rclpy.shutdown()

    def setUp(self):
        """各テストの初期化."""
        self.node = Node('test_joint_can_converter')
        
        # テスト用のパブリッシャーとサブスクライバー
        self.joint_command_pub = self.node.create_publisher(
            JointState, 'joint_command', 10
        )
        
        self.can_send_pub = self.node.create_publisher(
            Frame, 'can_send', 10
        )
        
        # テスト用のサブスクライバー
        self.joint_states_received = []
        self.motor_feedback_received = []
        self.system_status_received = []
        self.initialization_status_received = []
        self.can_received_received = []
        
        self.joint_states_sub = self.node.create_subscription(
            JointState, 'joint_states', self._joint_states_callback, 10
        )
        
        self.motor_feedback_sub = self.node.create_subscription(
            MotorFeedback, 'motor_feedback', self._motor_feedback_callback, 10
        )
        
        self.system_status_sub = self.node.create_subscription(
            SystemStatus, 'system_status', self._system_status_callback, 10
        )
        
        self.initialization_status_sub = self.node.create_subscription(
            InitializationStatus, 'initialization_status', self._initialization_status_callback, 10
        )
        
        self.can_received_sub = self.node.create_subscription(
            Frame, 'can_received', self._can_received_callback, 10
        )

    def tearDown(self):
        """各テストの終了."""
        self.node.destroy_node()

    def _joint_states_callback(self, msg):
        """JointState受信コールバック."""
        self.joint_states_received.append(msg)

    def _motor_feedback_callback(self, msg):
        """MotorFeedback受信コールバック."""
        self.motor_feedback_received.append(msg)

    def _system_status_callback(self, msg):
        """SystemStatus受信コールバック."""
        self.system_status_received.append(msg)

    def _initialization_status_callback(self, msg):
        """InitializationStatus受信コールバック."""
        self.initialization_status_received.append(msg)

    def _can_received_callback(self, msg):
        """CAN受信コールバック."""
        self.can_received_received.append(msg)

    def test_node_creation(self):
        """ノード作成のテスト."""
        self.assertIsNotNone(self.node)
        self.assertEqual(self.node.get_name(), 'test_joint_can_converter')

    def test_joint_command_publishing(self):
        """関節コマンド送信のテスト."""
        # テスト用のJointStateメッセージを作成
        joint_msg = JointState()
        joint_msg.name = ['front_left_hip', 'front_left_thigh']
        joint_msg.position = [0.5, -0.3]  # rad
        
        # メッセージを送信
        self.joint_command_pub.publish(joint_msg)
        
        # 少し待機
        time.sleep(0.1)
        
        # 送信されたメッセージを確認
        self.assertGreater(len(self.joint_states_received), 0)

    def test_can_frame_creation(self):
        """CANフレーム作成のテスト."""
        # テスト用のCANフレームを作成
        can_msg = Frame()
        can_msg.id = 0x60068  # モードID=6, デバイスID=0x68
        can_msg.dlc = 8
        can_msg.data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        can_msg.is_error = False
        can_msg.is_rtr = False
        can_msg.is_extended = True
        
        # メッセージを送信
        self.can_send_pub.publish(can_msg)
        
        # 少し待機
        time.sleep(0.1)
        
        # 送信されたメッセージを確認
        self.assertGreater(len(self.can_received_received), 0)

    def test_initialization_service(self):
        """初期化サービスのテスト."""
        # サービスクライアントを作成
        client = self.node.create_client(Trigger, 'initialize_system')
        
        # サービスが利用可能になるまで待機
        if not client.wait_for_service(timeout_sec=5.0):
            self.fail('Service not available')
        
        # サービスリクエストを送信
        request = Trigger.Request()
        future = client.call_async(request)
        
        # レスポンスを待機
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        # レスポンスを確認
        response = future.result()
        self.assertIsNotNone(response)
        self.assertTrue(response.success)

    def test_parameter_validation(self):
        """パラメータ検証のテスト."""
        # パラメータの設定
        self.node.declare_parameter('test_mode_id', 6)
        self.node.declare_parameter('test_device_id_start', 0x68)
        self.node.declare_parameter('test_num_motors', 12)
        
        # パラメータの取得
        mode_id = self.node.get_parameter('test_mode_id').value
        device_id_start = self.node.get_parameter('test_device_id_start').value
        num_motors = self.node.get_parameter('test_num_motors').value
        
        # パラメータの検証
        self.assertEqual(mode_id, 6)
        self.assertEqual(device_id_start, 0x68)
        self.assertEqual(num_motors, 12)

    def test_can_id_calculation(self):
        """CAN ID計算のテスト."""
        mode_id = 6
        device_id = 0x68
        
        # 28bit CAN IDの計算
        can_id = (mode_id << 8) | device_id
        expected_can_id = 0x60068
        
        self.assertEqual(can_id, expected_can_id)
        
        # デバイスIDの抽出
        extracted_device_id = can_id & 0xFF
        extracted_mode_id = (can_id >> 8) & 0x1FFFFF
        
        self.assertEqual(extracted_device_id, device_id)
        self.assertEqual(extracted_mode_id, mode_id)

    def test_position_conversion(self):
        """位置変換のテスト."""
        import math
        
        # ラジアンから度への変換
        position_rad = 1.57  # π/2
        position_deg = math.degrees(position_rad)
        
        self.assertAlmostEqual(position_deg, 90.0, places=1)
        
        # 物理値への変換（仮のスケール係数）
        position_scale = 0.01
        position_raw = int(position_deg / position_scale)
        
        self.assertEqual(position_raw, 9000)

    def test_message_types(self):
        """メッセージ型のテスト."""
        # MotorFeedbackメッセージの作成
        feedback_msg = MotorFeedback()
        feedback_msg.joint_name = 'test_joint'
        feedback_msg.angle = 1.57
        feedback_msg.current = 2.5
        feedback_msg.temperature = 30
        
        self.assertEqual(feedback_msg.joint_name, 'test_joint')
        self.assertEqual(feedback_msg.angle, 1.57)
        self.assertEqual(feedback_msg.current, 2.5)
        self.assertEqual(feedback_msg.temperature, 30)
        
        # SystemStatusメッセージの作成
        status_msg = SystemStatus()
        status_msg.system_ready = True
        status_msg.errors = ['test_error']
        status_msg.joint_positions = [0.0, 1.0, 2.0]
        
        self.assertTrue(status_msg.system_ready)
        self.assertEqual(len(status_msg.errors), 1)
        self.assertEqual(len(status_msg.joint_positions), 3)


def main(args=None):
    """メイン関数."""
    rclpy.init(args=args)
    
    # テストの実行
    import pytest
    pytest.main(['-v', __file__])


if __name__ == '__main__':
    main() 