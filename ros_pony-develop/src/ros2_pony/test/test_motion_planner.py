#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from ros2_pony.msg import SystemStatus
import time
import sys
import os

# testディレクトリからros2_ponyパッケージをインポートするためのパス設定
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'ros2_pony'))


class MotionPlannerTester(Node):
    """motion_plannerノードのテスト用ノード."""

    def __init__(self):
        super().__init__('motion_planner_tester')
        
        # パブリッシャーの設定
        self.joint_target_pub = self.create_publisher(
            JointState,
            'joint_target',
            10
        )
        
        self.system_status_pub = self.create_publisher(
            SystemStatus,
            'system_status',
            10
        )
        
        # サブスクライバーの設定
        self.joint_command_sub = self.create_subscription(
            JointState,
            'joint_command',
            self._joint_command_callback,
            10
        )
        
        self.motion_status_sub = self.create_subscription(
            String,
            'motion_status',
            self._motion_status_callback,
            10
        )
        
        # サービス
        self.initialize_client = self.create_client(
            Trigger,
            'initialize_motion'
        )
        
        # テスト用のタイマー
        self._test_timer = self.create_timer(5.0, self._run_test)
        
        # テスト状態
        self.test_step = 0
        self.received_commands = []
        self.received_status = []
        
        self.get_logger().info('MotionPlannerTester has been started')

    def _joint_command_callback(self, msg: JointState):
        """関節コマンドのコールバック."""
        self.received_commands.append(msg)
        self.get_logger().info(f'Received joint command: {dict(zip(msg.name, msg.position))}')

    def _motion_status_callback(self, msg: String):
        """動作状態のコールバック."""
        self.received_status.append(msg.data)
        self.get_logger().info(f'Received motion status: {msg.data}')

    def _run_test(self):
        """テストの実行."""
        if self.test_step == 0:
            # システム状態を準備完了に設定
            self._publish_system_ready()
            self.test_step += 1
            
        elif self.test_step == 1:
            # 初期化サービスのテスト
            self._test_initialization()
            self.test_step += 1
            
        elif self.test_step == 2:
            # 関節目標のテスト
            self._test_joint_target()
            self.test_step += 1
            
        elif self.test_step == 3:
            # テスト結果の表示
            self._show_test_results()
            self.test_step += 1
            
        else:
            # テスト完了
            self.get_logger().info('Test completed')
            rclpy.shutdown()

    def _publish_system_ready(self):
        """システム準備完了状態を送信."""
        status_msg = SystemStatus()
        status_msg.system_ready = True
        status_msg.errors = []
        status_msg.joint_positions = [0.0] * 12
        
        self.system_status_pub.publish(status_msg)
        self.get_logger().info('Published system ready status')

    def _test_initialization(self):
        """初期化サービスのテスト."""
        self.get_logger().info('Testing initialization service...')
        
        # サービスが利用可能になるまで待機
        while not self.initialize_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for initialization service...')
        
        # サービス呼び出し
        request = Trigger.Request()
        future = self.initialize_client.call_async(request)
        
        # 結果の確認
        if future.done():
            response = future.result()
            self.get_logger().info(f'Initialization result: {response.success} - {response.message}')
        else:
            self.get_logger().warn('Initialization service call timeout')

    def _test_joint_target(self):
        """関節目標のテスト."""
        self.get_logger().info('Testing joint target...')
        
        # テスト用の関節目標を送信
        joint_msg = JointState()
        joint_msg.name = ['front_left_hip', 'front_left_thigh']
        joint_msg.position = [0.5, 1.0]  # rad
        
        self.joint_target_pub.publish(joint_msg)
        self.get_logger().info(f'Sent joint target: {dict(zip(joint_msg.name, joint_msg.position))}')

    def _show_test_results(self):
        """テスト結果の表示."""
        self.get_logger().info('=== Test Results ===')
        self.get_logger().info(f'Received commands: {len(self.received_commands)}')
        self.get_logger().info(f'Received status messages: {len(self.received_status)}')
        
        if self.received_status:
            self.get_logger().info(f'Latest status: {self.received_status[-1]}')


def main(args=None):
    """メイン関数."""
    rclpy.init(args=args)
    
    tester = MotionPlannerTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        tester.get_logger().error(f'Unexpected error: {e}')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 