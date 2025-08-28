#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from can_msgs.msg import Frame
from std_srvs.srv import Trigger
from ros2_pony.msg import MotorFeedback, SystemStatus, InitializationStatus
import struct
import math
import threading
import time
from typing import Dict, List, Optional


class JointCanConverterNode(Node):
    """関節名と角度からCAN送信情報に変換し、CAN受信情報から関節名と角度に変換するノード."""

    def __init__(self):
        super().__init__('joint_can_converter_node')
        
        # パラメータの設定
        self.declare_parameter('mode_id', 6)  # モードID（固定）
        self.declare_parameter('device_id_start', 0x68)  # デバイスID開始値
        self.declare_parameter('num_motors', 12)  # モータ数
        self.declare_parameter('position_scale', 0.0001)  # 位置スケール係数（度→物理値）
        self.declare_parameter('velocity_scale', 10)  # 速度スケール係数（ERPM→物理値）
        self.declare_parameter('acceleration_scale', 10)  # 加速度スケール係数（ERPM/s2→物理値）
        self.declare_parameter('default_velocity', 10000)  # デフォルト速度（ERPM）
        self.declare_parameter('default_acceleration', 10000)  # デフォルト加速度（ERPM/s2）
        self.declare_parameter('homing_timeout', 30.0)  # ホーミングタイムアウト（秒）
        self.declare_parameter('debug_mode', False)  # デバッグモード
        
        # パラメータの取得
        self.mode_id = self.get_parameter('mode_id').value
        self.device_id_start = self.get_parameter('device_id_start').value
        self.num_motors = self.get_parameter('num_motors').value
        self.position_scale = self.get_parameter('position_scale').value
        self.velocity_scale = self.get_parameter('velocity_scale').value
        self.acceleration_scale = self.get_parameter('acceleration_scale').value
        self.default_velocity = self.get_parameter('default_velocity').value
        self.default_acceleration = self.get_parameter('default_acceleration').value
        self.homing_timeout = self.get_parameter('homing_timeout').value
        self.debug_mode = self.get_parameter('debug_mode').value
        
        # 状態管理
        self.is_initialized = False
        self.initialization_state = 0  # 0: 未開始, 1: 進行中, 2: 完了, 3: エラー
        self.homing_in_progress = False
        self.homing_start_time = 0.0
        self.motor_states: Dict[int, Dict] = {}  # モータ状態管理
        
        # TBD: 関節名の定義（仮実装）
        # TODO: 実際の関節名と物理配置に合わせて修正
        self.joint_names = [
            "front_left_hip", "front_left_thigh", "front_left_calf",
            "front_right_hip", "front_right_thigh", "front_right_calf", 
            "rear_left_hip", "rear_left_thigh", "rear_left_calf",
            "rear_right_hip", "rear_right_thigh", "rear_right_calf"
        ]
        
        # 新しいデバイスIDパターンの定義
        self.device_ids = [
            0x40, 0x41, 0x42,  # front_left: hip, thigh, calf
            0x50, 0x51, 0x52,  # front_right: hip, thigh, calf
            0x60, 0x61, 0x62,  # rear_left: hip, thigh, calf
            0x70, 0x71, 0x72   # rear_right: hip, thigh, calf
        ]
        
        # デバイスIDと関節名のマッピング
        self.device_id_to_joint = {}
        for i, device_id in enumerate(self.device_ids):
            if i < len(self.joint_names):
                self.device_id_to_joint[device_id] = self.joint_names[i]
        
        # デバッグログ: joint_namesとdevice_id_to_jointの内容を出力
        self.get_logger().info(f'JointCanConverterNode joint_names: {self.joint_names}')
        self.get_logger().info(f'JointCanConverterNode joint_names count: {len(self.joint_names)}')
        self.get_logger().info(f'JointCanConverterNode device_id_to_joint: {self.device_id_to_joint}')
        self.get_logger().info(f'JointCanConverterNode device_id_to_joint count: {len(self.device_id_to_joint)}')
        
        # パブリッシャーとサブスクライバーの設定
        self._can_send_publisher = self.create_publisher(
            Frame,
            'can_send',
            10
        )
        
        self._joint_state_publisher = self.create_publisher(
            JointState,
            'joint_states',
            10
        )
        
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
        
        self._initialization_status_publisher = self.create_publisher(
            InitializationStatus,
            'initialization_status',
            10
        )
        
        # サブスクライバー
        self._joint_command_subscription = self.create_subscription(
            JointState,
            'joint_command',
            self._joint_command_callback,
            10
        )
        
        self._can_received_subscription = self.create_subscription(
            Frame,
            'can_received',
            self._can_received_callback,
            10
        )
        
        # サービス
        self._initialize_service = self.create_service(
            Trigger,
            'initialize_system',
            self._initialize_system_callback
        )
        
        # タイマー
        self._status_timer = self.create_timer(1.0, self._publish_system_status)
        self._homing_timer = self.create_timer(0.1, self._check_homing_timeout)
        
        # 初期化
        self._init_motor_states()
        
        self.get_logger().info('JointCanConverterNode has been started')

    def _init_motor_states(self):
        """モータ状態の初期化."""
        for device_id in self.device_ids:
            self.motor_states[device_id] = {
                'position': 0.0,  # rad
                'velocity': 0.0,  # rad/s
                'current': 0.0,   # A
                'temperature': 0,  # ℃
                'is_online': False,
                'last_update': 0.0
            }

    def _joint_command_callback(self, msg: JointState):
        """関節コマンドのコールバック."""
        if not self.is_initialized:
            self.get_logger().warn('System not initialized, ignoring joint command')
            return
        
        try:
            # デバッグログ: 受信したjoint_commandの内容を出力
            self.get_logger().info(f'JointCanConverterNode received joint_command:')
            self.get_logger().info(f'  - name count: {len(msg.name)}')
            self.get_logger().info(f'  - position count: {len(msg.position)}')
            self.get_logger().info(f'  - names: {msg.name}')
            self.get_logger().info(f'  - positions: {msg.position}')
            
            # 関節名と位置のマッピング
            joint_positions = {}
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    joint_positions[name] = msg.position[i]
            
            # デバッグログ: joint_positionsの内容を出力
            self.get_logger().info(f'JointCanConverterNode joint_positions: {joint_positions}')
            
            # 各モータにコマンドを送信
            for device_id, joint_name in self.device_id_to_joint.items():
                if joint_name in joint_positions:
                    position_rad = joint_positions[joint_name]
                    self.get_logger().info(f'JointCanConverterNode sending command for device_id={device_id}, joint_name={joint_name}, position_rad={position_rad}')
                    self._send_motor_command(device_id, position_rad)
                else:
                    self.get_logger().warn(f'JointCanConverterNode joint_name {joint_name} not found in joint_positions')
                    
        except Exception as e:
            self.get_logger().error(f'Error processing joint command: {e}')

    def _send_motor_command(self, device_id: int, position_rad: float):
        """モータコマンドを送信する."""
        try:
            # CAN IDの計算（28bit）
            can_id = (self.mode_id << 8) | device_id
            
            # 位置を度に変換
            position_deg = math.degrees(position_rad)
            
            # 物理値の範囲チェック
            if position_deg < -36000 or position_deg > 36000:
                self.get_logger().warn(f'Position {position_deg} deg out of range for device {device_id}')
                return
            
            # 物理値に変換
            position_raw = int(position_deg / self.position_scale)
            velocity_raw = int(self.default_velocity / self.velocity_scale)
            acceleration_raw = int(self.default_acceleration / self.acceleration_scale)
            
            # データバイトの作成
            data = bytearray(8)
            
            # 位置（4バイト、ビッグエンディアン）
            data[0] = (position_raw >> 24) & 0xFF
            data[1] = (position_raw >> 16) & 0xFF
            data[2] = (position_raw >> 8) & 0xFF
            data[3] = position_raw & 0xFF
            
            # 速度（2バイト、ビッグエンディアン）
            data[4] = (velocity_raw >> 8) & 0xFF
            data[5] = velocity_raw & 0xFF
            
            # 加速度（2バイト、ビッグエンディアン）
            data[6] = (acceleration_raw >> 8) & 0xFF
            data[7] = acceleration_raw & 0xFF
            
            # CANフレームの作成
            frame_msg = Frame()
            frame_msg.id = can_id
            frame_msg.dlc = 8
            frame_msg.data = list(data)
            frame_msg.is_error = False
            frame_msg.is_rtr = False
            frame_msg.is_extended = True  # 28bit ID
            
            # 送信
            self._can_send_publisher.publish(frame_msg)
            
            self.get_logger().debug(f'Sent command to device {device_id}: position={position_deg:.2f}deg')
            self.get_logger().info(f'JointCanConverterNode sent CAN message: ID=0x{can_id:x}, Data={list(data)}')
            
        except Exception as e:
            self.get_logger().error(f'Error sending motor command: {e}')

    def _can_received_callback(self, msg: Frame):
        """CAN受信コールバック."""
        try:
            # 28bit IDからデバイスIDを抽出
            device_id = msg.id & 0xFF
            feedback_mode_id = (msg.id >> 8) & 0xFF  # [15-8]bit
            
            # フィードバックモードIDの確認（0x29と仮定）
            if feedback_mode_id != 0x29:
                return  # フィードバック以外のメッセージは無視
            
            # デバイスIDの範囲チェック（新しいデバイスIDパターンに対応）
            if device_id not in self.device_ids:
                return  # 定義されていないデバイスIDは無視
            
            # デバッグログ
            if self.debug_mode:
                self.get_logger().info(f'Received feedback from device {device_id}: ID={hex(msg.id)}, Data={[hex(x) for x in msg.data]}')
            else:
                self.get_logger().debug(f'Received feedback from device {device_id}: ID={hex(msg.id)}, Data={[hex(x) for x in msg.data]}')
            
            # フィードバックデータの解析
            self._parse_motor_feedback(device_id, msg.data)
            
        except Exception as e:
            self.get_logger().error(f'Error processing CAN message: {e}')

    def _parse_motor_feedback(self, device_id: int, data: List[int]):
        """モータフィードバックを解析する."""
        try:
            # データ長チェック
            if len(data) < 8:
                self.get_logger().warn(f'Invalid data length for device {device_id}: {len(data)} bytes')
                return
            
            current_time = time.time()
            
            data0_int = int(data[0])
            data1_int = int(data[1])
            data2_int = int(data[2])
            data3_int = int(data[3])
            data4_int = int(data[4])
            data5_int = int(data[5])
            data6_int = int(data[6])
            
            # データ解析
            # Data[0,1]: 位置 (-32000~32000 → -3200度~3200度)
            position_raw = (data0_int << 8) | data1_int
            if position_raw > 32767:  # 負の値の処理
                position_raw -= 65536
            position_deg = (position_raw / 10.0)  # スケール係数: 10
            position_rad = math.radians(position_deg)
            
            # Data[2,3]: 速度 (-32000~32000 → -320000rpm~320000rpm)
            velocity_raw = (data2_int << 8) | data3_int
            if velocity_raw > 32767:  # 負の値の処理
                velocity_raw -= 65536
            velocity_rpm = velocity_raw * 10.0  # スケール係数: 10
            
            # Data[4,5]: 電流 (-6000~6000 → -60A~60A)
            current_raw = (data4_int << 8) | data5_int
            if current_raw > 32767:  # 負の値の処理
                current_raw -= 65536
            current_a = current_raw / 100.0  # スケール係数: 100
            
            # Data[6]: 温度 (-20~127度C)
            temperature_c = data6_int
            if temperature_c > 127:  # 負の値の処理
                temperature_c -= 256
            
            # Data[7]: エラーコード (0が正常)
            error_code = data[7]
            is_error = error_code != 0
            
            # 状態更新
            if device_id in self.motor_states:
                self.motor_states[device_id].update({
                    'position': position_rad,
                    'velocity': velocity_rpm,
                    'current': current_a,
                    'temperature': temperature_c,
                    'error_code': error_code,
                    'is_online': True,
                    'last_update': current_time
                })
                
                # フィードバックメッセージの送信
                if device_id in self.device_id_to_joint:
                    joint_name = self.device_id_to_joint[device_id]
                    
                    feedback_msg = MotorFeedback()
                    feedback_msg.joint_name = joint_name
                    feedback_msg.angle = position_rad
                    feedback_msg.current = current_a
                    feedback_msg.temperature = temperature_c
                    
                    self._motor_feedback_publisher.publish(feedback_msg)
                    
                    # デバッグログ（エラー時は警告レベル）
                    log_level = self.get_logger().warn if is_error else self.get_logger().debug
                    log_level(f'Motor {device_id} ({joint_name}): pos={position_rad:.3f}rad ({position_deg:.1f}deg), '
                             f'vel={velocity_rpm:.1f}rpm, current={current_a:.2f}A, temp={temperature_c}℃, '
                             f'error={error_code}')
                    
                    # エラー時は警告
                    if is_error:
                        error_msg = self._get_error_message(error_code)
                        self.get_logger().warn(f'Motor {device_id} ({joint_name}) error: {error_msg} (code: {error_code})')
            
        except Exception as e:
            self.get_logger().error(f'Error parsing motor feedback for device {device_id}: {e}')
            self.get_logger().error(f'Raw data: {[hex(x) for x in data]}')

    def _get_error_message(self, error_code: int) -> str:
        """エラーコードからエラーメッセージを取得する."""
        error_messages = {
            0: "No error",
            1: "Over voltage",
            2: "Under voltage", 
            3: "Over current",
            4: "Over temperature",
            5: "Motor stall",
            6: "Hall sensor error",
            7: "Encoder error",
            8: "Communication error",
            9: "Position limit error",
            10: "Velocity limit error",
            11: "Torque limit error",
            12: "System error",
            13: "Calibration error",
            14: "Emergency stop",
            15: "Unknown error"
        }
        return error_messages.get(error_code, f"Unknown error code: {error_code}")

    def _initialize_system_callback(self, request, response):
        """システム初期化サービスのコールバック."""
        try:
            self.get_logger().info('Starting system initialization...')
            
            # 初期化状態を進行中に設定
            self.initialization_state = 1
            self._publish_initialization_status()
            
            # 初期化シーケンスの開始
            self._start_initialization_sequence()
            
            response.success = True
            response.message = 'Initialization started'
            
        except Exception as e:
            self.get_logger().error(f'Error starting initialization: {e}')
            response.success = False
            response.message = f'Initialization failed: {e}'
            
        return response

    def _start_initialization_sequence(self):
        """初期化シーケンスを開始する."""
        try:
            # 1. システム状態確認
            self.get_logger().info('Step 1: Checking system status...')
            self._check_system_status()
            
            # 2. ホーミング開始
            self.get_logger().info('Step 2: Starting homing sequence...')
            self._start_homing()
            
        except Exception as e:
            self.get_logger().error(f'Error in initialization sequence: {e}')
            self.initialization_state = 3  # エラー
            self._publish_initialization_status()

    def _check_system_status(self):
        """システム状態を確認する."""
        # TBD: システム状態確認の実装
        # TODO: 実際のシステム状態確認に合わせて実装
        self.get_logger().info('System status check completed (TBD implementation)')

    def _start_homing(self):
        """ホーミングを開始する."""
        # TBD: ホーミングコマンドの実装
        # TODO: 実際のホーミングコマンドに合わせて実装
        self.get_logger().info('Starting homing sequence (TBD implementation)')
        
        self.homing_in_progress = True
        self.homing_start_time = time.time()
        
        # 仮のホーミング完了（実際の実装では、ホーミング完了を待機）
        # ここでは3秒後に完了と仮定
        threading.Timer(3.0, self._complete_homing).start()

    def _complete_homing(self):
        """ホーミング完了処理."""
        self.get_logger().info('Homing completed')
        self.homing_in_progress = False
        
        # 初期化完了
        self.initialization_state = 2  # 完了
        self.is_initialized = True
        self._publish_initialization_status()
        
        self.get_logger().info('System initialization completed successfully')

    def _check_homing_timeout(self):
        """ホーミングタイムアウトをチェックする."""
        if self.homing_in_progress:
            elapsed_time = time.time() - self.homing_start_time
            if elapsed_time > self.homing_timeout:
                self.get_logger().error(f'Homing timeout after {elapsed_time:.1f} seconds')
                self.homing_in_progress = False
                self.initialization_state = 3  # エラー
                self._publish_initialization_status()

    def _publish_initialization_status(self):
        """初期化状態をパブリッシュする."""
        status_msg = InitializationStatus()
        status_msg.state = self.initialization_state
        status_msg.is_complete = (self.initialization_state == 2)
        
        if self.initialization_state == 0:
            status_msg.status_msg = "Initialization not started"
        elif self.initialization_state == 1:
            status_msg.status_msg = "Initialization in progress"
        elif self.initialization_state == 2:
            status_msg.status_msg = "Initialization completed"
        elif self.initialization_state == 3:
            status_msg.status_msg = "Initialization failed"
        
        self._initialization_status_publisher.publish(status_msg)

    def _publish_system_status(self):
        """システム状態をパブリッシュする."""
        # 全モータのオンライン状態をチェック
        online_motors = sum(1 for state in self.motor_states.values() if state['is_online'])
        system_ready = self.is_initialized and (online_motors == self.num_motors)
        
        # エラーリスト（現在は空）
        errors = []
        
        # 現在位置の取得
        joint_positions = []
        for device_id in self.device_ids:
            if device_id in self.motor_states:
                joint_positions.append(self.motor_states[device_id]['position'])
            else:
                joint_positions.append(0.0)
        
        # システム状態メッセージの作成
        status_msg = SystemStatus()
        status_msg.system_ready = system_ready
        status_msg.errors = errors
        status_msg.joint_positions = joint_positions
        
        self._system_status_publisher.publish(status_msg)
        
        # 速度と電流の取得
        joint_velocities = []
        joint_currents = []
        for device_id in self.device_ids:
            if device_id in self.motor_states:
                # 速度をrpmからrad/sに変換
                velocity_rpm = self.motor_states[device_id].get('velocity', 0.0)
                velocity_rad_s = math.radians(velocity_rpm / 60.0) # rpm → rad/s
                joint_velocities.append(velocity_rad_s)
                joint_currents.append(self.motor_states[device_id].get('current', 0.0))
            else:
                joint_velocities.append(0.0)
                joint_currents.append(0.0)
        
        # JointStateメッセージの送信
        joint_state_msg = JointState()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = joint_positions
        joint_state_msg.velocity = joint_velocities
        joint_state_msg.effort = joint_currents  # 電流をトルクとして使用
        
        self._joint_state_publisher.publish(joint_state_msg)

    def destroy_node(self):
        """ノードの破棄."""
        super().destroy_node()


def main(args=None):
    """メイン関数."""
    rclpy.init(args=args)

    node = JointCanConverterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterruptを受信しました。JointCanConverterNodeを終了中...")
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