#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import time
import os
import sys

# testディレクトリからros2_ponyパッケージをインポートするためのパス設定
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'ros2_pony'))


class CanDeviceChecker(Node):
    """CANデバイスの接続確認専用ノード."""

    def __init__(self):
        super().__init__('can_device_checker')
        
        # 診断結果をパブリッシュ
        self._diagnostics_publisher = self.create_publisher(
            String,
            'can_diagnostics',
            10
        )
        
        # 詳細診断を実行
        self._run_comprehensive_diagnostics()
        
        self.get_logger().info('CanDeviceChecker has been started')

    def _run_comprehensive_diagnostics(self):
        """包括的な診断を実行する."""
        self.get_logger().info('=== CAN Device Comprehensive Diagnostics ===')
        
        # 1. システム情報
        self._check_system_info()
        
        # 2. USBデバイス
        self._check_usb_devices()
        
        # 3. CANインターフェース
        self._check_can_interfaces()
        
        # 4. カーネルモジュール
        self._check_kernel_modules()
        
        # 5. 権限とデバイスファイル
        self._check_permissions_and_devices()
        
        # 6. ネットワーク設定
        self._check_network_config()
        
        # 7. ログメッセージ
        self._check_system_logs()
        
        # 8. 推奨事項
        self._provide_recommendations()

    def _check_system_info(self):
        """システム情報を確認する."""
        self.get_logger().info('--- System Information ---')
        
        try:
            # OS情報
            result = subprocess.run(['uname', '-a'], capture_output=True, text=True)
            if result.returncode == 0:
                self.get_logger().info(f'OS: {result.stdout.strip()}')
            
            # カーネルバージョン
            result = subprocess.run(['cat', '/proc/version'], capture_output=True, text=True)
            if result.returncode == 0:
                self.get_logger().info(f'Kernel: {result.stdout.strip()}')
            
        except Exception as e:
            self.get_logger().error(f'Error checking system info: {e}')

    def _check_usb_devices(self):
        """USBデバイスを詳細に確認する."""
        self.get_logger().info('--- USB Devices ---')
        
        try:
            # lsusb
            result = subprocess.run(['lsusb'], capture_output=True, text=True)
            if result.returncode == 0:
                usb_devices = result.stdout
                self.get_logger().info(f'USB devices:\n{usb_devices}')
                
                # usb2canデバイスを探す
                if 'usb2can' in usb_devices.lower() or 'innomaker' in usb_devices.lower():
                    self.get_logger().info('✓ USB2CAN device detected')
                else:
                    self.get_logger().warn('✗ USB2CAN device not found')
            else:
                self.get_logger().error('Failed to run lsusb')
            
            # /sys/bus/usb/devices
            result = subprocess.run(['ls', '/sys/bus/usb/devices/'], capture_output=True, text=True)
            if result.returncode == 0:
                self.get_logger().info(f'USB device IDs: {result.stdout.strip()}')
            
        except Exception as e:
            self.get_logger().error(f'Error checking USB devices: {e}')

    def _check_can_interfaces(self):
        """CANインターフェースを詳細に確認する."""
        self.get_logger().info('--- CAN Interfaces ---')
        
        try:
            # ip link show
            result = subprocess.run(['ip', 'link', 'show'], capture_output=True, text=True)
            if result.returncode == 0:
                interfaces = result.stdout
                self.get_logger().info(f'Network interfaces:\n{interfaces}')
                
                # CANインターフェースを探す
                can_interfaces = [line for line in interfaces.split('\n') if 'can' in line.lower()]
                if can_interfaces:
                    self.get_logger().info(f'✓ CAN interfaces found: {can_interfaces}')
                else:
                    self.get_logger().warn('✗ No CAN interfaces found')
            else:
                self.get_logger().error('Failed to check network interfaces')
            
            # /sys/class/net
            result = subprocess.run(['ls', '/sys/class/net/'], capture_output=True, text=True)
            if result.returncode == 0:
                net_devices = result.stdout
                can_devices = [dev for dev in net_devices.split() if 'can' in dev.lower()]
                if can_devices:
                    self.get_logger().info(f'CAN network devices: {can_devices}')
            
        except Exception as e:
            self.get_logger().error(f'Error checking CAN interfaces: {e}')

    def _check_kernel_modules(self):
        """カーネルモジュールを詳細に確認する."""
        self.get_logger().info('--- Kernel Modules ---')
        
        try:
            # lsmod
            result = subprocess.run(['lsmod'], capture_output=True, text=True)
            if result.returncode == 0:
                modules = result.stdout
                self.get_logger().info(f'Loaded modules:\n{modules}')
                
                # CAN関連モジュールを確認
                can_modules = ['can', 'can_raw', 'can_dev', 'usb2can']
                for module in can_modules:
                    if module in modules:
                        self.get_logger().info(f'✓ CAN module {module} is loaded')
                    else:
                        self.get_logger().warn(f'✗ CAN module {module} is not loaded')
            else:
                self.get_logger().error('Failed to check kernel modules')
            
            # modinfo
            for module in ['can', 'can_raw', 'can_dev']:
                try:
                    result = subprocess.run(['modinfo', module], capture_output=True, text=True)
                    if result.returncode == 0:
                        self.get_logger().info(f'Module {module} info available')
                except:
                    pass
            
        except Exception as e:
            self.get_logger().error(f'Error checking kernel modules: {e}')

    def _check_permissions_and_devices(self):
        """権限とデバイスファイルを確認する."""
        self.get_logger().info('--- Permissions and Device Files ---')
        
        try:
            # /dev/ttyUSB*
            result = subprocess.run(['ls', '-la', '/dev/ttyUSB*'], capture_output=True, text=True)
            if result.returncode == 0:
                devices = result.stdout
                self.get_logger().info(f'USB serial devices:\n{devices}')
            else:
                self.get_logger().warn('No USB serial devices found')
            
            # /dev/can*
            result = subprocess.run(['ls', '-la', '/dev/can*'], capture_output=True, text=True)
            if result.returncode == 0:
                can_devices = result.stdout
                self.get_logger().info(f'CAN devices:\n{can_devices}')
            else:
                self.get_logger().warn('No CAN devices found')
            
            # ユーザーグループ
            result = subprocess.run(['groups'], capture_output=True, text=True)
            if result.returncode == 0:
                groups = result.stdout.strip()
                self.get_logger().info(f'Current user groups: {groups}')
                
                if 'dialout' in groups:
                    self.get_logger().info('✓ User is in dialout group')
                else:
                    self.get_logger().warn('✗ User is not in dialout group')
            
        except Exception as e:
            self.get_logger().error(f'Error checking permissions: {e}')

    def _check_network_config(self):
        """ネットワーク設定を確認する."""
        self.get_logger().info('--- Network Configuration ---')
        
        try:
            # CANインターフェースの状態
            result = subprocess.run(['ip', 'link', 'show', 'can0'], capture_output=True, text=True)
            if result.returncode == 0:
                self.get_logger().info(f'CAN0 interface:\n{result.stdout}')
            else:
                self.get_logger().warn('CAN0 interface not found')
            
            # CANインターフェースの統計
            result = subprocess.run(['ip', '-s', 'link', 'show', 'can0'], capture_output=True, text=True)
            if result.returncode == 0:
                self.get_logger().info(f'CAN0 statistics:\n{result.stdout}')
            
        except Exception as e:
            self.get_logger().error(f'Error checking network config: {e}')

    def _check_system_logs(self):
        """システムログを確認する."""
        self.get_logger().info('--- System Logs ---')
        
        try:
            # dmesg
            result = subprocess.run(['dmesg', '|', 'grep', '-i', 'can'], capture_output=True, text=True, shell=True)
            if result.returncode == 0 and result.stdout.strip():
                self.get_logger().info(f'CAN-related kernel messages:\n{result.stdout}')
            else:
                self.get_logger().info('No CAN-related kernel messages found')
            
            # usb2can関連
            result = subprocess.run(['dmesg', '|', 'grep', '-i', 'usb2can'], capture_output=True, text=True, shell=True)
            if result.returncode == 0 and result.stdout.strip():
                self.get_logger().info(f'USB2CAN-related messages:\n{result.stdout}')
            else:
                self.get_logger().info('No USB2CAN-related messages found')
            
        except Exception as e:
            self.get_logger().error(f'Error checking system logs: {e}')

    def _provide_recommendations(self):
        """推奨事項を提供する."""
        self.get_logger().info('--- Recommendations ---')
        
        recommendations = [
            "1. USB2CANデバイスが接続されているか確認してください",
            "2. デバイスドライバーが正しくインストールされているか確認してください",
            "3. ユーザーがdialoutグループに所属しているか確認してください",
            "4. CANインターフェースが正しく設定されているか確認してください",
            "5. 必要なカーネルモジュールがロードされているか確認してください",
            "6. デバイスファイルの権限を確認してください",
            "7. システムログでエラーメッセージを確認してください"
        ]
        
        for rec in recommendations:
            self.get_logger().info(rec)
        
        # 診断結果をパブリッシュ
        diagnostics_msg = String()
        diagnostics_msg.data = "CAN device diagnostics completed - check logs for details"
        self._diagnostics_publisher.publish(diagnostics_msg)


def main(args=None):
    """メイン関数."""
    rclpy.init(args=args)

    checker = CanDeviceChecker()

    try:
        rclpy.spin(checker)
    except KeyboardInterrupt:
        pass
    finally:
        checker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 