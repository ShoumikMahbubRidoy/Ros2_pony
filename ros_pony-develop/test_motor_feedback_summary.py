#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from ros2_pony.msg import MotorFeedback, SystemStatus, MotorFeedbackArray
import time
import random


class MotorFeedbackSummaryTester(Node):
    """motor_feedbackサマリー機能のテスト用ノード."""
    
    def __init__(self):
        super().__init__('motor_feedback_summary_tester')
        
        # パブリッシャー
        self._motor_feedback_publisher = self.create_publisher(
            MotorFeedback,
            'motor_feedback',
            10
        )
        
        self._system_status_publisher = self.create_publisher(
            SystemStatus,
            'system_status',
            10
        )
        
        # サブスクライバー
        self._motor_feedback_summary_subscription = self.create_subscription(
            MotorFeedbackArray,
            'motor_feedback_summary',
            self._motor_feedback_summary_callback,
            10
        )
        
        # Unity対応用：JointState版のmotor_feedback_summaryも受信
        self._motor_feedback_summary_jointstate_subscription = self.create_subscription(
            JointState,
            'motor_feedback_summary_jointstate',
            self._motor_feedback_summary_jointstate_callback,
            10
        )
        
        self._motion_status_subscription = self.create_subscription(
            String,
            'motion_status',
            self._motion_status_callback,
            10
        )
        
        # タイマー
        self._motor_feedback_timer = self.create_timer(0.1, self._publish_motor_feedback)  # 10Hz
        self._system_status_timer = self.create_timer(1.0, self._publish_system_status)    # 1Hz
        
        # テスト用データ
        self.joint_names = [
            "front_left_hip", "front_left_thigh", "front_left_calf",
            "front_right_hip", "front_right_thigh", "front_right_calf", 
            "rear_left_hip", "rear_left_thigh", "rear_left_calf",
            "rear_right_hip", "rear_right_thigh", "rear_right_calf"
        ]
        
        self.get_logger().info('MotorFeedbackSummaryTester started')
        self.get_logger().info('Publishing motor feedback data and monitoring summary')
        
    def _publish_motor_feedback(self):
        """モータフィードバックデータを送信."""
        try:
            # ランダムな関節を選択
            joint_name = random.choice(self.joint_names)
            
            # ランダムなデータを生成
            msg = MotorFeedback()
            msg.joint_name = joint_name
            msg.angle = random.uniform(-3.14, 3.14)  # -π to π rad
            msg.current = random.uniform(0.0, 5.0)   # 0-5A
            msg.temperature = random.randint(20, 80)  # 20-80°C
            
            self._motor_feedback_publisher.publish(msg)
            
            self.get_logger().debug(f'Published motor feedback: {joint_name} - angle: {msg.angle:.3f}, current: {msg.current:.3f}, temp: {msg.temperature}')
            
        except Exception as e:
            self.get_logger().error(f'Error publishing motor feedback: {e}')
    
    def _publish_system_status(self):
        """システム状態を送信."""
        try:
            msg = SystemStatus()
            msg.system_ready = True
            msg.errors = []
            msg.joint_positions = [0.0] * len(self.joint_names)
            
            self._system_status_publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing system status: {e}')
    
    def _motor_feedback_summary_callback(self, msg: MotorFeedbackArray):
        """motor_feedbackサマリーを受信."""
        try:
            self.get_logger().info('=== Motor Feedback Summary Received ===')
            self.get_logger().info(f'Joint count: {len(msg.motor_feedbacks)}')
            
            for feedback in msg.motor_feedbacks:
                self.get_logger().info(f'{feedback.joint_name}: angle={feedback.angle:.3f}rad, current={feedback.current:.3f}A, temp={feedback.temperature}°C')
            
            self.get_logger().info('=====================================')
            
        except Exception as e:
            self.get_logger().error(f'Error processing motor feedback summary: {e}')
    
    def _motor_feedback_summary_jointstate_callback(self, msg: JointState):
        """JointState版のmotor_feedbackサマリーを受信."""
        try:
            self.get_logger().info('=== Motor Feedback Summary (JointState) Received ===')
            self.get_logger().info(f'Joint count: {len(msg.name)}')
            
            for i, joint_name in enumerate(msg.name):
                if i < len(msg.position) and i < len(msg.velocity) and i < len(msg.effort):
                    angle = msg.position[i]
                    current = msg.velocity[i]  # 電流はvelocityフィールドに格納
                    temperature = msg.effort[i]  # 温度はeffortフィールドに格納
                    self.get_logger().info(f'{joint_name}: angle={angle:.3f}rad, current={current:.3f}A, temp={temperature:.1f}°C')
            
            self.get_logger().info('=====================================')
            
        except Exception as e:
            self.get_logger().error(f'Error processing JointState motor feedback summary: {e}')
    
    def _motion_status_callback(self, msg: String):
        """動作状態を受信."""
        try:
            self.get_logger().info(f'Motion status: {msg.data}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing motion status: {e}')


def main(args=None):
    """メイン関数."""
    rclpy.init(args=args)
    
    node = MotorFeedbackSummaryTester()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 