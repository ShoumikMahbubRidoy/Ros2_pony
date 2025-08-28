#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from can_msgs.msg import Frame
from sensor_msgs.msg import JointState
from ros2_pony.msg import MotorFeedback, SystemStatus
import time
import struct
import math


class MotorFeedbackTester(Node):
    """モータフィードバック解析のテスト用ノード."""

    def __init__(self):
        super().__init__('motor_feedback_tester')
        
        # パブリッシャーの設定
        self._can_send_publisher = self.create_publisher(
            Frame, 'can_send', 10
        )
        
        # サブスクライバーの設定
        self.motor_feedback_received = []
        self.joint_states_received = []
        self.system_status_received = []
        
        self._motor_feedback_sub = self.create_subscription(
            MotorFeedback, 'motor_feedback', self._motor_feedback_callback, 10
        )
        
        self._joint_states_sub = self.create_subscription(
            JointState, 'joint_states', self._joint_states_callback, 10
        )
        
        self._system_status_sub = self.create_subscription(
            SystemStatus, 'system_status', self._system_status_callback, 10
        )
        
        # テスト用のタイマー
        self._test_timer = self.create_timer(2.0, self._send_test_feedback)
        
        self.get_logger().info('MotorFeedbackTester has been started')

    def _motor_feedback_callback(self, msg: MotorFeedback):
        """MotorFeedback受信コールバック."""
        self.motor_feedback_received.append(msg)
        self.get_logger().info(f'Received MotorFeedback: {msg.joint_name} - '
                              f'angle={msg.angle:.3f}rad, current={msg.current:.2f}A, '
                              f'temp={msg.temperature}℃')

    def _joint_states_callback(self, msg: JointState):
        """JointState受信コールバック."""
        self.joint_states_received.append(msg)
        self.get_logger().info(f'Received JointState: {len(msg.name)} joints, '
                              f'positions={[f"{p:.3f}" for p in msg.position[:3]]}...')

    def _system_status_callback(self, msg: SystemStatus):
        """SystemStatus受信コールバック."""
        self.system_status_received.append(msg)
        self.get_logger().info(f'Received SystemStatus: ready={msg.system_ready}, '
                              f'errors={len(msg.errors)}, positions={len(msg.joint_positions)}')

    def _create_test_feedback(self, device_id: int, position_deg: float, velocity_rpm: float, 
                             current_a: float, temperature_c: int, error_code: int = 0) -> Frame:
        """テスト用のフィードバックメッセージを作成する."""
        # 物理値から生データに変換
        position_raw = int(position_deg * 10)  # 度 → 生データ
        if position_raw < 0:
            position_raw += 65536
        
        velocity_raw = int(velocity_rpm / 10)  # rpm → 生データ
        if velocity_raw < 0:
            velocity_raw += 65536
        
        current_raw = int(current_a * 100)  # A → 生データ
        if current_raw < 0:
            current_raw += 65536
        
        # 温度の処理
        if temperature_c < 0:
            temperature_c += 256
        
        # データバイトの作成
        data = [
            (position_raw >> 8) & 0xFF,  # Data[0]: 位置上位バイト
            position_raw & 0xFF,          # Data[1]: 位置下位バイト
            (velocity_raw >> 8) & 0xFF,  # Data[2]: 速度上位バイト
            velocity_raw & 0xFF,          # Data[3]: 速度下位バイト
            (current_raw >> 8) & 0xFF,   # Data[4]: 電流上位バイト
            current_raw & 0xFF,           # Data[5]: 電流下位バイト
            temperature_c,                 # Data[6]: 温度
            error_code                     # Data[7]: エラーコード
        ]
        
        # CAN IDの作成（0x29をモードIDとして使用）
        can_id = (0x29 << 8) | device_id
        
        # Frameメッセージの作成
        frame_msg = Frame()
        frame_msg.id = can_id
        frame_msg.dlc = 8
        frame_msg.data = data
        frame_msg.is_extended = True
        frame_msg.is_rtr = False
        frame_msg.is_error = False
        
        return frame_msg

    def _send_test_feedback(self):
        """テスト用のフィードバックメッセージを送信."""
        # テストケース1: 正常なデータ
        test_msg1 = self._create_test_feedback(
            device_id=0x68,  # デバイスID
            position_deg=45.0,  # 45度
            velocity_rpm=100.0,  # 100rpm
            current_a=2.5,  # 2.5A
            temperature_c=35,  # 35度C
            error_code=0  # エラーなし
        )
        
        # テストケース2: 負の値
        test_msg2 = self._create_test_feedback(
            device_id=0x69,  # デバイスID
            position_deg=-30.0,  # -30度
            velocity_rpm=-50.0,  # -50rpm
            current_a=-1.0,  # -1.0A
            temperature_c=-10,  # -10度C
            error_code=0  # エラーなし
        )
        
        # テストケース3: エラー状態
        test_msg3 = self._create_test_feedback(
            device_id=0x6A,  # デバイスID
            position_deg=0.0,  # 0度
            velocity_rpm=0.0,  # 0rpm
            current_a=0.0,  # 0A
            temperature_c=25,  # 25度C
            error_code=3  # 過電流エラー
        )
        
        # メッセージを送信
        self._can_send_publisher.publish(test_msg1)
        self.get_logger().info(f'Sent test feedback 1: ID={hex(test_msg1.id)}, Data={[hex(x) for x in test_msg1.data]}')
        
        time.sleep(0.1)
        
        self._can_send_publisher.publish(test_msg2)
        self.get_logger().info(f'Sent test feedback 2: ID={hex(test_msg2.id)}, Data={[hex(x) for x in test_msg2.data]}')
        
        time.sleep(0.1)
        
        self._can_send_publisher.publish(test_msg3)
        self.get_logger().info(f'Sent test feedback 3: ID={hex(test_msg3.id)}, Data={[hex(x) for x in test_msg3.data]}')


def main(args=None):
    """メイン関数."""
    rclpy.init(args=args)

    node = MotorFeedbackTester()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterruptを受信しました。MotorFeedbackTesterを終了中...")
    except Exception as e:
        print(f"エラーが発生しました: {e}")
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception as e:
            print(f"終了処理中にエラーが発生しました: {e}")


if __name__ == '__main__':
    main() 