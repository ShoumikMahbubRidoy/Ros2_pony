#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from ros2_pony.msg import MotorFeedback, SystemStatus, MotorFeedbackArray
from ros2_pony.msg import JointNames
import time
from typing import Dict, List, Optional
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class MotionPlannerNode(Node):
    """ユーザの指示を制御指示に変換するノード（初期段階はpass-through）."""

    def __init__(self):
        super().__init__('motion_planner_node')
        
        # パラメータの設定
        self.declare_parameter('max_velocity', 1000.0)  # モータ最大速度 [ERPM]
        self.declare_parameter('max_acceleration', 1000.0)  # モータ最大加速度 [ERPM/s²]
        self.declare_parameter('can_send_frequency', 10.0)  # CAN送信頻度 [Hz]
        self.declare_parameter('status_publish_frequency', 1.0)  # 状態送信頻度 [Hz]
        
        # パラメータの取得
        self.max_velocity = self.get_parameter('max_velocity').value
        self.max_acceleration = self.get_parameter('max_acceleration').value
        self.can_send_frequency = self.get_parameter('can_send_frequency').value
        self.status_publish_frequency = self.get_parameter('status_publish_frequency').value
        
        # 状態管理
        self.is_initialized = False
        self.is_error = False
        self.error_message = ""
        self.manually_initialized = False  # 手動初期化フラグ
        self.current_joint_targets: Dict[str, float] = {}  # 現在の目標角度
        self.current_joint_states: Dict[str, float] = {}  # 現在の関節状態
        self.motor_feedbacks: Dict[str, Dict] = {}  # モータフィードバック
        
        # 関節名の定義（joint_can_converterと同様）
        self.joint_names = [
            "front_left_hip", "front_left_thigh", "front_left_calf",
            "front_right_hip", "front_right_thigh", "front_right_calf", 
            "rear_left_hip", "rear_left_thigh", "rear_left_calf",
            "rear_right_hip", "rear_right_thigh", "rear_right_calf"
        ]
        
        # デバッグログ: joint_namesの内容を出力
        self.get_logger().info(f'MotionPlannerNode joint_names: {self.joint_names}')
        self.get_logger().info(f'MotionPlannerNode joint_names count: {len(self.joint_names)}')
        
        # パブリッシャーの設定
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE,
                                history=HistoryPolicy.KEEP_LAST,
                                depth=10)
        self._joint_command_publisher = self.create_publisher(
            JointState,
            'joint_command',
            10
        )
        
        self._motion_status_publisher = self.create_publisher(
            String,
            'motion_status',
            10
        )
        # セットオリジン専用トピック
        self._joint_set_origin_publisher = self.create_publisher(
            JointNames,
            'joint_set_origin',
            10
        )
        
        # motor_feedbackサマリー用のパブリッシャーを追加
        self._motor_feedback_summary_publisher = self.create_publisher(
            MotorFeedbackArray,
            'motor_feedback_summary',
            sensor_qos
        )
        
        # Unity対応用：JointStateでもmotor_feedbackサマリーを送信
        # 注意: JointStateの標準的な使い方とは異なります
        # position: 角度 [rad], velocity: 電流 [A], effort: 温度 [°C]
        self._motor_feedback_summary_jointstate_publisher = self.create_publisher(
            JointState,
            'motor_feedback_summary_jointstate',
            sensor_qos
        )
        
        # サブスクライバーの設定

        self._joint_target_subscription = self.create_subscription(
            JointState,
            'joint_target',
            self._joint_target_callback,
            sensor_qos
        )
        
        self._joint_states_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self._joint_states_callback,
            10
        )
        
        self._motor_feedback_subscription = self.create_subscription(
            MotorFeedback,
            'motor_feedback',
            self._motor_feedback_callback,
            10
        )
        
        self._system_status_subscription = self.create_subscription(
            SystemStatus,
            'system_status',
            self._system_status_callback,
            10
        )
        
        # サービス
        self._initialize_service = self.create_service(
            Trigger,
            'initialize_motion',
            self._initialize_motion_callback
        )
        
        # タイマー
        self._status_timer = self.create_timer(
            1.0 / self.status_publish_frequency, 
            self._publish_motion_status
        )
        
        # 初期化
        self._init_joint_states()
        
        self.get_logger().info('MotionPlannerNode has been started')
        self.get_logger().info(f'Parameters: max_velocity={self.max_velocity}, '
                              f'max_acceleration={self.max_acceleration}, '
                              f'can_send_frequency={self.can_send_frequency}')

    def _init_joint_states(self):
        """関節状態の初期化."""
        for joint_name in self.joint_names:
            self.current_joint_states[joint_name] = 0.0
            self.current_joint_targets[joint_name] = 0.0
            self.motor_feedbacks[joint_name] = {
                'angle': 0.0,
                'current': 0.0,
                'temperature': 0
            }

    def _joint_target_callback(self, msg: JointState):
        """目標角度のコールバック（pass-through実装）."""
        if self.is_error:
            self.get_logger().error(f'Node is in error state: {self.error_message}')
            return
        
        try:
            # effort のみ（position が空）の場合は SetOrigin と解釈
            if (len(msg.position) == 0) and (len(msg.effort) > 0):
                target_names = []
                for i, name in enumerate(msg.name):
                    if i < len(msg.effort) and msg.effort[i] == 0.0:
                        target_names.append(name)
                if target_names:
                    req = JointNames()
                    req.name = target_names
                    self._joint_set_origin_publisher.publish(req)
                    self.get_logger().info(f'Published joint_set_origin for: {target_names}')
                return

            # 目標角度の更新
            for i, name in enumerate(msg.name):
                if i < len(msg.position) and name in self.joint_names:
                    self.current_joint_targets[name] = msg.position[i]
            
            # pass-through: 目標角度をそのまま制御指示として送信
            self._publish_joint_command()
            
            self.get_logger().debug(f'Received joint targets: {dict(zip(msg.name, msg.position))}')
            
        except Exception as e:
            self.is_error = True
            self.error_message = f'Error processing joint target: {e}'
            self.get_logger().error(self.error_message)

    def _joint_states_callback(self, msg: JointState):
        """現在の関節状態のコールバック."""
        if self.is_error:
            return
        
        try:
            for i, name in enumerate(msg.name):
                if i < len(msg.position) and name in self.joint_names:
                    self.current_joint_states[name] = msg.position[i]
                    
        except Exception as e:
            self.is_error = True
            self.error_message = f'Error processing joint states: {e}'
            self.get_logger().error(self.error_message)

    def _motor_feedback_callback(self, msg: MotorFeedback):
        """モータフィードバックのコールバック."""
        if self.is_error:
            return
        
        try:
            if msg.joint_name in self.motor_feedbacks:
                self.motor_feedbacks[msg.joint_name] = {
                    'angle': msg.angle,
                    'current': msg.current,
                    'temperature': msg.temperature
                }
                
        except Exception as e:
            self.is_error = True
            self.error_message = f'Error processing motor feedback: {e}'
            self.get_logger().error(self.error_message)

    def _system_status_callback(self, msg: SystemStatus):
        """システム状態のコールバック."""
        if self.is_error:
            return
        
        try:
            # システム準備完了状態の更新
            if msg.system_ready and not self.is_initialized:
                self.is_initialized = True
                self.manually_initialized = False  # システム状態による初期化
                self.get_logger().info('System is ready, motion planner initialized')
            elif not msg.system_ready and self.is_initialized:
                # 手動初期化された場合は、システム状態に関係なく初期化を維持
                if self.manually_initialized:
                    self.get_logger().warn('System is not ready, but keeping motion planner initialized (manual initialization)')
                else:
                    self.is_initialized = False
                    self.get_logger().warn('System is not ready, motion planner deinitialized')
                
        except Exception as e:
            self.is_error = True
            self.error_message = f'Error processing system status: {e}'
            self.get_logger().error(self.error_message)

    def _publish_joint_command(self):
        """制御指示を送信する."""
        if not self.is_initialized:
            self.get_logger().warn('System not initialized, skipping joint command')
            self.get_logger().info('To initialize, either:')
            self.get_logger().info('1. Send SystemStatus message with system_ready=true')
            self.get_logger().info('2. Call initialize_motion service')
            return
        
        try:
            # JointStateメッセージの作成
            joint_msg = JointState()
            joint_msg.name = self.joint_names  # 12個すべての関節名
            joint_msg.position = [self.current_joint_targets.get(j, 0.0) for j in self.joint_names]
            joint_msg.velocity = [self.max_velocity] * len(joint_msg.name)
            joint_msg.effort = [self.max_acceleration] * len(joint_msg.name)
            
            # デバッグログ: 送信内容の詳細を出力
            self.get_logger().info(f'MotionPlannerNode publishing joint_command:')
            self.get_logger().info(f'  - name count: {len(joint_msg.name)}')
            self.get_logger().info(f'  - position count: {len(joint_msg.position)}')
            self.get_logger().info(f'  - names: {joint_msg.name}')
            self.get_logger().info(f'  - positions: {joint_msg.position}')
            
            # 送信
            self._joint_command_publisher.publish(joint_msg)
            
            self.get_logger().debug(f'Published joint command: {dict(zip(joint_msg.name, joint_msg.position))}')
            
        except Exception as e:
            self.is_error = True
            self.error_message = f'Error publishing joint command: {e}'
            self.get_logger().error(self.error_message)

    def _publish_motion_status(self):
        """動作状態を送信する."""
        try:
            status_msg = String()
            
            if self.is_error:
                status_msg.data = f"ERROR: {self.error_message}"
            elif not self.is_initialized:
                status_msg.data = "WAITING: System not initialized"
            else:
                # 目標到達判定（簡易版）
                all_targets_reached = True
                for joint_name in self.joint_names:
                    if joint_name in self.current_joint_targets and joint_name in self.current_joint_states:
                        target = self.current_joint_targets[joint_name]
                        current = self.current_joint_states[joint_name]
                        if abs(target - current) > 0.01:  # 0.01 rad (約0.57度) の閾値
                            all_targets_reached = False
                            break
                
                if all_targets_reached:
                    status_msg.data = "COMPLETE: All targets reached"
                else:
                    status_msg.data = "ACTIVE: Moving to targets"
            
            self._motion_status_publisher.publish(status_msg)
            
            # motor_feedbackサマリーの送信
            self._publish_motor_feedback_summary()
            
        except Exception as e:
            self.get_logger().error(f'Error publishing motion status: {e}')

    def _publish_motor_feedback_summary(self):
        """motor_feedbackサマリーを送信する."""
        try:
            if not self.is_initialized:
                return
            
            # JointStateメッセージの作成（motor_feedbackサマリー用）
            summary_msg = MotorFeedbackArray()
            
            # 各関節のmotor_feedback情報を配列に格納
            angles = []
            currents = []
            temperatures = []
            
            for joint_name in self.joint_names:
                if joint_name in self.motor_feedbacks:
                    feedback = self.motor_feedbacks[joint_name]
                    angles.append(feedback['angle'])
                    currents.append(feedback['current'])
                    temperatures.append(float(feedback['temperature']))
                else:
                    # データがない場合は0で初期化
                    angles.append(0.0)
                    currents.append(0.0)
                    temperatures.append(0.0)
            
            # MotorFeedbackArrayメッセージに設定
            # 各MotorFeedbackメッセージの作成
            for i in range(len(self.joint_names)):
                feedback_msg = MotorFeedback()
                feedback_msg.joint_name = self.joint_names[i]
                feedback_msg.angle = angles[i]
                feedback_msg.current = currents[i]
                feedback_msg.temperature = int(temperatures[i])
                summary_msg.motor_feedbacks.append(feedback_msg)
            
            # 送信
            self._motor_feedback_summary_publisher.publish(summary_msg)
            
            # Unity対応用：JointStateでもmotor_feedbackサマリーを送信
            # 注意: JointStateの標準的な使い方とは異なります
            # position: 角度 [rad], velocity: 電流 [A], effort: 温度 [°C]
            jointstate_summary_msg = JointState()
            jointstate_summary_msg.name = self.joint_names
            jointstate_summary_msg.position = angles
            jointstate_summary_msg.velocity = currents  # 電流をvelocityフィールドに格納
            jointstate_summary_msg.effort = temperatures
            
            self._motor_feedback_summary_jointstate_publisher.publish(jointstate_summary_msg)
            
            self.get_logger().debug(f'Published motor feedback summary: {len(summary_msg.motor_feedbacks)} joints (MotorFeedbackArray + JointState)')
            
        except Exception as e:
            self.get_logger().error(f'Error publishing motor feedback summary: {e}')

    def _initialize_motion_callback(self, request, response):
        """初期化サービスのコールバック."""
        try:
            self.get_logger().info('Motion initialization requested')
            
            # エラー状態のリセット
            self.is_error = False
            self.error_message = ""
            
            # 初期化処理（現在は状態リセットのみ）
            self._init_joint_states()
            
            # システムが準備完了していない場合は自動的に初期化
            if not self.is_initialized:
                self.is_initialized = True
                self.manually_initialized = True  # 手動初期化フラグを設定
                self.get_logger().info('Auto-initialized motion planner (no system status received)')
            
            response.success = True
            response.message = "Motion planner initialized successfully"
            
            self.get_logger().info('Motion initialization completed')
            
        except Exception as e:
            response.success = False
            response.message = f"Motion initialization failed: {e}"
            self.get_logger().error(f'Motion initialization error: {e}')
        
        return response

    def destroy_node(self):
        """ノードの破棄."""
        self.get_logger().info('MotionPlannerNode is shutting down')
        super().destroy_node()


def main(args=None):
    """メイン関数."""
    rclpy.init(args=args)
    
    node = MotionPlannerNode()
    
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