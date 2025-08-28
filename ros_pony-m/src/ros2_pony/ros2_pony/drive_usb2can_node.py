#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from can_msgs.msg import Frame
from std_msgs.msg import String
import can
import threading
import time
import subprocess
import os
import signal
import sys
from typing import Optional


class DriveUsb2CanNode(Node):
    """usb2canデバイスを使用してCAN送受信を行うノード."""

    def __init__(self):
        super().__init__('drive_usb2can_node')
        
        # パラメータの設定
        self.declare_parameter('can_interface', 'socketcan')  # usb2canからsocketcanに変更
        self.declare_parameter('can_bitrate', 1000000)  # 1Mbps
        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('receive_timeout', 1.0)
        self.declare_parameter('enable_diagnostics', True)
        self.declare_parameter('enable_system_setup', True)  # システム設定を有効にする
        
        # パラメータの取得
        self.can_interface = self.get_parameter('can_interface').value
        self.can_bitrate = self.get_parameter('can_bitrate').value
        self.can_channel = self.get_parameter('can_channel').value
        self.receive_timeout = self.get_parameter('receive_timeout').value
        self.enable_diagnostics = self.get_parameter('enable_diagnostics').value
        self.enable_system_setup = self.get_parameter('enable_system_setup').value
        
        # CANバスの初期化
        self.can_bus: Optional[can.Bus] = None
        self.is_connected = False
        self.connection_errors = []
        
        # パブリッシャーとサブスクライバーの設定
        self._can_receive_publisher = self.create_publisher(
            Frame,
            'can_received',
            10
        )
        
        self._can_send_subscription = self.create_subscription(
            Frame,
            'can_send',
            self._can_send_callback,
            10
        )
        
        # 状態監視用のパブリッシャー
        self._status_publisher = self.create_publisher(
            String,
            'can_status',
            10
        )
        
        # 診断用のパブリッシャー
        self._diagnostics_publisher = self.create_publisher(
            String,
            'can_diagnostics',
            10
        )
        
        # 定期的な状態監視タイマー
        self._status_timer = self.create_timer(1.0, self._publish_status)
        
        # 診断タイマー（5秒間隔）
        if self.enable_diagnostics:
            self._diagnostics_timer = self.create_timer(5.0, self._run_diagnostics)
        
        # CAN受信スレッド
        self._receive_thread = None
        self._stop_receive_thread = False
        
        # システム設定の実行
        if self.enable_system_setup:
            self._setup_can_interface()
        
        # 初期診断の実行
        if self.enable_diagnostics:
            self._run_initial_diagnostics()
        
        # CANバスの初期化
        self._init_can_bus()
        
        self.get_logger().info('DriveUsb2CanNode has been started')

    def _setup_can_interface(self):
        """CANインターフェースのシステム設定を実行する."""
        try:
            self.get_logger().info('Setting up CAN interface...')
            
            # CANインターフェースを停止
            result = subprocess.run(['sudo', 'ifconfig', self.can_channel, 'down'], 
                                  capture_output=True, text=True)
            if result.returncode != 0:
                self.get_logger().warn(f'Failed to bring down {self.can_channel}: {result.stderr}')
            
            # CANインターフェースを設定
            result = subprocess.run([
                'sudo', 'ip', 'link', 'set', self.can_channel, 'type', 'can', 
                'bitrate', str(self.can_bitrate)
            ], capture_output=True, text=True)
            if result.returncode != 0:
                self.get_logger().error(f'Failed to set CAN bitrate: {result.stderr}')
                self.connection_errors.append(f'CAN bitrate setup failed: {result.stderr}')
            else:
                self.get_logger().info(f'CAN bitrate set to {self.can_bitrate}')
            
            # 送信キュー長を設定
            result = subprocess.run([
                'sudo', 'ifconfig', self.can_channel, 'txqueuelen', '100000'
            ], capture_output=True, text=True)
            if result.returncode != 0:
                self.get_logger().warn(f'Failed to set txqueuelen: {result.stderr}')
            
            # CANインターフェースを起動
            result = subprocess.run(['sudo', 'ifconfig', self.can_channel, 'up'], 
                                  capture_output=True, text=True)
            if result.returncode != 0:
                self.get_logger().error(f'Failed to bring up {self.can_channel}: {result.stderr}')
                self.connection_errors.append(f'CAN interface startup failed: {result.stderr}')
            else:
                self.get_logger().info(f'CAN interface {self.can_channel} is up')
                
        except Exception as e:
            self.get_logger().error(f'Error setting up CAN interface: {e}')
            self.connection_errors.append(f'CAN interface setup failed: {e}')

    def _run_initial_diagnostics(self):
        """初期診断を実行する."""
        self.get_logger().info('Running initial CAN diagnostics...')
        
        # 1. USBデバイスの確認
        self._check_usb_devices()
        
        # 2. CANインターフェースの確認
        self._check_can_interfaces()
        
        # 3. カーネルモジュールの確認
        self._check_kernel_modules()
        
        # 4. 権限の確認
        self._check_permissions()

    def _check_usb_devices(self):
        """USBデバイスを確認する."""
        try:
            result = subprocess.run(['lsusb'], capture_output=True, text=True)
            if result.returncode == 0:
                usb_devices = result.stdout
                self.get_logger().info(f'USB devices found:\n{usb_devices}')
                
                # usb2canデバイスを探す
                if 'usb2can' in usb_devices.lower() or 'innomaker' in usb_devices.lower():
                    self.get_logger().info('USB2CAN device detected')
                else:
                    self.get_logger().warn('USB2CAN device not found in lsusb output')
                    self.connection_errors.append('USB2CAN device not detected')
            else:
                self.get_logger().error('Failed to run lsusb command')
                self.connection_errors.append('Cannot check USB devices')
        except Exception as e:
            self.get_logger().error(f'Error checking USB devices: {e}')
            self.connection_errors.append(f'USB device check failed: {e}')

    def _check_can_interfaces(self):
        """CANインターフェースを確認する."""
        try:
            # ip link show でCANインターフェースを確認
            result = subprocess.run(['ip', 'link', 'show'], capture_output=True, text=True)
            if result.returncode == 0:
                interfaces = result.stdout
                self.get_logger().info(f'Network interfaces:\n{interfaces}')
                
                if self.can_channel in interfaces:
                    self.get_logger().info(f'CAN interface {self.can_channel} found')
                else:
                    self.get_logger().warn(f'CAN interface {self.can_channel} not found')
                    self.connection_errors.append(f'CAN interface {self.can_channel} not found')
            else:
                self.get_logger().error('Failed to check network interfaces')
                self.connection_errors.append('Cannot check network interfaces')
        except Exception as e:
            self.get_logger().error(f'Error checking CAN interfaces: {e}')
            self.connection_errors.append(f'CAN interface check failed: {e}')

    def _check_kernel_modules(self):
        """カーネルモジュールを確認する."""
        try:
            result = subprocess.run(['lsmod'], capture_output=True, text=True)
            if result.returncode == 0:
                modules = result.stdout
                self.get_logger().info(f'Loaded kernel modules:\n{modules}')
                
                # CAN関連モジュールを確認
                can_modules = ['can', 'can_raw', 'can_dev']
                for module in can_modules:
                    if module in modules:
                        self.get_logger().info(f'CAN module {module} is loaded')
                    else:
                        self.get_logger().warn(f'CAN module {module} is not loaded')
                        self.connection_errors.append(f'CAN module {module} not loaded')
            else:
                self.get_logger().error('Failed to check kernel modules')
                self.connection_errors.append('Cannot check kernel modules')
        except Exception as e:
            self.get_logger().error(f'Error checking kernel modules: {e}')
            self.connection_errors.append(f'Kernel module check failed: {e}')

    def _check_permissions(self):
        """権限を確認する."""
        try:
            # /dev/ttyUSB* デバイスの権限を確認
            result = subprocess.run(['ls', '-la', '/dev/ttyUSB*'], capture_output=True, text=True)
            if result.returncode == 0:
                devices = result.stdout
                self.get_logger().info(f'USB serial devices:\n{devices}')
            else:
                self.get_logger().warn('No USB serial devices found or permission denied')
                self.connection_errors.append('No USB serial devices found')
        except Exception as e:
            self.get_logger().error(f'Error checking permissions: {e}')
            self.connection_errors.append(f'Permission check failed: {e}')

    def _run_diagnostics(self):
        """定期的な診断を実行する."""
        if not self.enable_diagnostics:
            return
            
        try:
            # CANバスの状態確認
            if self.can_bus is not None:
                try:
                    # 簡単なテストメッセージを送信して接続を確認
                    test_message = can.Message(arbitration_id=0x123, data=[0x01, 0x02, 0x03, 0x04])
                    self.can_bus.send(test_message)
                    self.get_logger().debug('CAN bus connection test successful')
                except Exception as e:
                    self.get_logger().warn(f'CAN bus connection test failed: {e}')
                    self.is_connected = False
                    self.connection_errors.append(f'CAN bus test failed: {e}')
            
            # 診断情報をパブリッシュ
            diagnostics_msg = String()
            diagnostics_msg.data = f'CAN Diagnostics - Connected: {self.is_connected}, Errors: {len(self.connection_errors)}'
            self._diagnostics_publisher.publish(diagnostics_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in diagnostics: {e}')

    def _init_can_bus(self):
        """CANバスを初期化する."""
        try:
            # send.pyと同様の設定を使用
            if self.can_interface == 'socketcan':
                # socketcanインターフェースの場合（非推奨警告を回避）
                config = {
                    'channel': self.can_channel,
                    'interface': 'socketcan'
                }
            else:
                # その他のインターフェースの場合
                config = {
                    'interface': self.can_interface,
                    'channel': self.can_channel,
                    'bitrate': self.can_bitrate,
                    'receive_timeout': self.receive_timeout
                }
            
            self.get_logger().info(f'Initializing CAN bus with config: {config}')
            
            # 接続エラーがある場合は警告
            if self.connection_errors:
                self.get_logger().warn(f'Connection errors detected: {self.connection_errors}')
            
            # CANバスの作成
            self.can_bus = can.Bus(**config)
            self.is_connected = True
            
            # 受信スレッドの開始
            self._start_receive_thread()
            
            self.get_logger().info('CAN bus initialized successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize CAN bus: {e}')
            self.is_connected = False
            self.connection_errors.append(f'CAN bus initialization failed: {e}')

    def _start_receive_thread(self):
        """CAN受信スレッドを開始する."""
        if self._receive_thread is None or not self._receive_thread.is_alive():
            self._stop_receive_thread = False
            self._receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self._receive_thread.start()
            self.get_logger().info('CAN receive thread started')

    def _receive_loop(self):
        """CAN受信ループ."""
        while not self._stop_receive_thread and self.is_connected:
            try:
                if self.can_bus is not None:
                    # CANメッセージを受信
                    message = self.can_bus.recv(timeout=self.receive_timeout)
                    
                    if message is not None:
                        # ROSメッセージに変換
                        frame_msg = Frame()
                        frame_msg.id = message.arbitration_id
                        frame_msg.dlc = message.dlc
                        frame_msg.data = list(message.data)
                        frame_msg.is_error = message.is_error_frame
                        frame_msg.is_rtr = message.is_remote_frame
                        frame_msg.is_extended = message.is_extended_id
                        
                        # パブリッシュ
                        self._can_receive_publisher.publish(frame_msg)
                        
                        self.get_logger().debug(f'Received CAN message: ID={hex(message.arbitration_id)}, Data={message.data.hex()}')
                        
            except Exception as e:
                self.get_logger().error(f'Error in CAN receive loop: {e}')
                time.sleep(0.1)

    def _can_send_callback(self, msg: Frame):
        """CAN送信コールバック."""
        if not self.is_connected or self.can_bus is None:
            self.get_logger().warn('CAN bus not connected, cannot send message')
            return
            
        try:
            # send.pyと同様のシンプルなメッセージ作成
            can_message = can.Message(
                arbitration_id=msg.id,
                data=msg.data  # bytes()変換を削除して、直接リストを使用
            )
            
            # CANメッセージを送信
            self.can_bus.send(can_message)
            
            self.get_logger().info(f'Sent CAN message: ID={hex(msg.id)}, Data={msg.data}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to send CAN message: {e}')
            # エラーが発生した場合、接続状態を再確認
            self.is_connected = False
            self.connection_errors.append(f'Send failed: {e}')

    def _publish_status(self):
        """CAN状態をパブリッシュする."""
        status_msg = String()
        
        if self.is_connected:
            status_msg.data = f'CAN connected - Interface: {self.can_interface}, Channel: {self.can_channel}'
        else:
            status_msg.data = f'CAN disconnected - Interface: {self.can_interface}, Channel: {self.can_channel}'
        
        self._status_publisher.publish(status_msg)

    def destroy_node(self):
        """ノードの破棄."""
        self._stop_receive_thread = True
        
        if self._receive_thread is not None:
            self._receive_thread.join(timeout=2.0)
        
        if self.can_bus is not None:
            self.can_bus.shutdown()
        
        super().destroy_node()


def signal_handler(signum, frame):
    """シグナルハンドラー."""
    print(f"\nシグナル {signum} を受信しました。CANノードを終了中...")
    sys.exit(0)

def main(args=None):
    """メイン関数."""
    # シグナルハンドラーの設定
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    rclpy.init(args=args)

    node = DriveUsb2CanNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterruptを受信しました。CANノードを終了中...")
    except Exception as e:
        print(f"エラーが発生しました: {e}")
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception as e:
            print(f"終了処理中にエラーが発生しました: {e}")
            sys.exit(1)


if __name__ == '__main__':
    main() 