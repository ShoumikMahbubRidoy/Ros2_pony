#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import csv
import sys
import time
import threading
import argparse
import tty
import termios
import os
import shutil
from typing import List, Dict, Optional
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class CsvMessageSender(Node):
    """CSVファイルからROS2メッセージを送信するノード."""

    def __init__(self, csv_files: List[str]):
        super().__init__('csv_message_sender')
        
        # CSVファイルの読み込み
        self.csv_data = self._load_csv_files(csv_files)
        if not self.csv_data:
            self.get_logger().error('No valid CSV files loaded. Exiting.')
            sys.exit(1)
        
        # QoS設定（motion_planner_nodeと同じ）
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # パブリッシャーの辞書（動的に作成）- 名前を変更して競合を避ける
        self._csv_publishers: Dict[str, any] = {}
        
        # 再生状態
        self.is_playing = False
        self.current_file_index = -1
        
        # キーボード入力用のスレッド
        self.input_thread = threading.Thread(target=self._input_handler, daemon=True)
        self.input_thread.start()
        
        self.get_logger().info(f'CsvMessageSender started with {len(self.csv_data)} CSV files')
        self._print_usage()

    def _load_csv_files(self, csv_files: List[str]) -> List[Dict]:
        """CSVファイルを読み込む."""
        csv_data = []
        
        for i, file_path in enumerate(csv_files):
            try:
                with open(file_path, 'r', encoding='utf-8-sig') as f:  # BOMを自動除去
                    reader = csv.reader(f)
                    
                    # 1行目: メッセージ型とトピック名
                    header_row = next(reader)
                    if len(header_row) < 2:
                        self.get_logger().error(f'Invalid header in {file_path}')
                        continue
                    
                    message_type, topic_name = header_row[0], header_row[1]
                    
                    # 2行目: カラム名
                    column_names = next(reader)
                    
                    # データ行の読み込み
                    data_rows = []
                    for row in reader:
                        if len(row) >= len(column_names):
                            data_rows.append(row)
                    
                    csv_data.append({
                        'file_path': file_path,
                        'file_index': i,
                        'message_type': message_type,
                        'topic_name': topic_name,
                        'column_names': column_names,
                        'data_rows': data_rows
                    })
                    
                    log_message = f'Loaded {file_path}: {message_type} -> {topic_name} ({len(data_rows)} rows)'
                    wrapped_log = self._wrap_text(log_message, self._get_terminal_width())
                    self.get_logger().info(wrapped_log)
                    
            except Exception as e:
                self.get_logger().error(f'Failed to load {file_path}: {e}')
                continue
        
        return csv_data

    def _create_publisher(self, message_type: str, topic_name: str):
        """メッセージ型に応じてパブリッシャーを作成."""
        if topic_name in self._csv_publishers:
            return self._csv_publishers[topic_name]
        
        # メッセージ型の正確な比較（BOM除去済み）
        if message_type.strip() == 'sensor_msgs/JointState':
            try:
                publisher = self.create_publisher(JointState, topic_name, self.sensor_qos)
                self._csv_publishers[topic_name] = publisher
                log_message = f'Created publisher for {message_type} -> {topic_name}'
                wrapped_log = self._wrap_text(log_message, self._get_terminal_width())
                self.get_logger().info(wrapped_log)
                return publisher
            except Exception as e:
                self.get_logger().error(f'Failed to create publisher: {e}')
                return None
        else:
            self.get_logger().error(f'Unsupported message type: "{message_type.strip()}"')
            self.get_logger().info(f'Supported types: sensor_msgs/JointState')
            return None

    def _get_joint_names_for_origin(self) -> List[str]:
        """SetOrigin送信用の関節名リストを取得.

        優先度:
        1) 読み込んだCSVのカラム名(sensor_msgs/JointState)から取得
        2) 既定の12関節名( motion_planner_node と同一 )
        """
        try:
            for csv_info in self.csv_data:
                if csv_info.get('message_type', '').strip() == 'sensor_msgs/JointState':
                    names = [n for n in csv_info.get('column_names', []) if n != 'timestamp']
                    if names:
                        return names
        except Exception:
            pass

        return [
            "front_left_hip", "front_left_thigh", "front_left_calf",
            "front_right_hip", "front_right_thigh", "front_right_calf",
            "rear_left_hip", "rear_left_thigh", "rear_left_calf",
            "rear_right_hip", "rear_right_thigh", "rear_right_calf",
        ]

    def _send_set_origin(self, target_names: Optional[List[str]] = None):
        """motion_planner_node が SetOrigin と解釈する JointState を送信."""
        try:
            names = target_names if target_names else self._get_joint_names_for_origin()
            if not names:
                self.get_logger().error('No joint names available for SetOrigin')
                return

            publisher = self._create_publisher('sensor_msgs/JointState', 'joint_target')
            if not publisher:
                self.get_logger().error('Failed to create publisher for joint_target')
                return

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = names
            msg.position = []  # 空であることが SetOrigin 判定条件
            msg.effort = [0.0] * len(names)  # 0.0 が指定された関節を SetOrigin 対象に

            publisher.publish(msg)
            log_message = f'Sent SetOrigin request to joint_target for joints: {names}'
            wrapped_log = self._wrap_text(log_message, self._get_terminal_width())
            self.get_logger().info(wrapped_log)
        except Exception as e:
            self.get_logger().error(f'Failed to send SetOrigin: {e}')

    def _play_csv_file(self, file_index: int):
        """指定されたCSVファイルを再生."""
        if file_index < 0 or file_index >= len(self.csv_data):
            self.get_logger().error(f'Invalid file index: {file_index}')
            return
        
        csv_info = self.csv_data[file_index]
        log_message = f'Playing {csv_info["file_path"]}'
        wrapped_log = self._wrap_text(log_message, self._get_terminal_width())
        self.get_logger().info(wrapped_log)
        
        # パブリッシャーの作成
        publisher = self._create_publisher(csv_info['message_type'], csv_info['topic_name'])
        if not publisher:
            self.get_logger().error(f'Failed to create publisher for {csv_info["message_type"]}')
            return
        
        self.is_playing = True
        self.current_file_index = file_index
        
        try:
            # 最初のタイムスタンプを基準とする
            start_time = time.time()
            first_timestamp = float(csv_info['data_rows'][0][0])
            
            for i, row in enumerate(csv_info['data_rows']):
                if not self.is_playing:
                    break
                
                # タイムスタンプの処理
                timestamp = float(row[0])
                relative_time = timestamp - first_timestamp
                
                # 適切なタイミングまで待機
                elapsed_time = time.time() - start_time
                if relative_time > elapsed_time:
                    time.sleep(relative_time - elapsed_time)
                
                # メッセージの作成と送信
                if csv_info['message_type'].strip() == 'sensor_msgs/JointState':
                    msg = JointState()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    
                    # 関節名と位置を設定
                    joint_names = []
                    positions = []
                    
                    for j, value in enumerate(row[1:], 1):  # timestampを除く
                        if j < len(csv_info['column_names']):
                            joint_name = csv_info['column_names'][j]
                            if joint_name != 'timestamp':
                                joint_names.append(joint_name)
                                positions.append(float(value))
                    
                    msg.name = joint_names
                    msg.position = positions
                    
                    publisher.publish(msg)
                    log_message = f'Sent message {i+1}/{len(csv_info["data_rows"])} to {csv_info["topic_name"]}'
                    wrapped_log = self._wrap_text(log_message, self._get_terminal_width())
                    self.get_logger().info(wrapped_log)
                
        except Exception as e:
            self.get_logger().error(f'Error during playback: {e}')
        finally:
            self.is_playing = False
            self.current_file_index = -1
            log_message = f'Finished playing {csv_info["file_path"]}'
            wrapped_log = self._wrap_text(log_message, self._get_terminal_width())
            self.get_logger().info(wrapped_log)

    def _input_handler(self):
        """キーボード入力の処理（キー単位）."""
        # ターミナル設定を保存
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            # ターミナルをrawモードに設定
            tty.setraw(sys.stdin.fileno())
            
            while True:
                try:
                    # キー入力を読み取り
                    ch = sys.stdin.read(1)
                    
                    # Ctrl+Cで終了
                    if ch == '\x03':
                        print("\nShutting down...")
                        os._exit(0)
                    
                    # 数字キーの処理（1-9）
                    if ch in '123456789':
                        file_index = int(ch) - 1  # 1ベースから0ベースに変換
                        if 0 <= file_index < len(self.csv_data):
                            if self.is_playing:
                                self.get_logger().info('Stopping current playback...')
                                self.is_playing = False
                                time.sleep(0.1)  # 現在の再生を停止
                            
                            # 新しいファイルの再生を開始
                            threading.Thread(target=self._play_csv_file, args=(file_index,), daemon=True).start()
                        else:
                            error_message = f'Invalid file number: {ch}'
                            wrapped_error = self._wrap_text(error_message, self._get_terminal_width())
                            self.get_logger().error(wrapped_error)
                            self._print_usage()
                    # Enterキーでヘルプ表示
                    elif ch == '\r' or ch == '\n':
                        self._print_usage()
                    # 0キーで SetOrigin を送信
                    elif ch == '0':
                        # 再生中でも SetOrigin は即時送信（再生は継続）
                        self._send_set_origin()
                    
                except Exception as e:
                    self.get_logger().error(f'Input error: {e}')
                    break
                    
        finally:
            # ターミナル設定を復元
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def _get_terminal_width(self) -> int:
        """ターミナル幅を取得する."""
        try:
            return shutil.get_terminal_size().columns
        except:
            return 80  # デフォルト値

    def _wrap_text(self, text: str, width: int, indent: str = "") -> str:
        """テキストを指定幅で改行する."""
        if len(text) <= width:
            return text
        
        words = text.split()
        lines = []
        current_line = indent
        
        for word in words:
            if len(current_line) + len(word) + 1 <= width:
                current_line += word + " "
            else:
                lines.append(current_line.rstrip())
                current_line = indent + word + " "
        
        if current_line.strip():
            lines.append(current_line.rstrip())
        
        return "\n".join(lines)

    def _print_usage(self):
        """使用方法を表示."""
        terminal_width = self._get_terminal_width()
        
        print("\n=== CSV Message Sender ===")
        print("Available files:")
        for i, csv_info in enumerate(self.csv_data):
            file_info = f"  {i+1}: {csv_info['file_path']} -> {csv_info['topic_name']}"
            wrapped_info = self._wrap_text(file_info, terminal_width, "    ")
            print(wrapped_info)
        print("\nCommands:")
        print("  [1-9]: Play corresponding CSV file (instant)")
        print("  0: Send SetOrigin (to joint_target) for known joints")
        print("  Enter: Show this help")
        print("  Ctrl+C: Exit")
        print("========================\n")


def main(args=None):
    """メイン関数."""
    parser = argparse.ArgumentParser(description='Send ROS2 messages from CSV files')
    parser.add_argument('csv_files', nargs='+', help='CSV files to load')
    
    # コマンドライン引数の解析
    if len(sys.argv) < 2:
        print("Usage: python3 csv_message_sender.py <csv_file1> [csv_file2] ...")
        print("Example: python3 csv_message_sender.py data1.csv data2.csv")
        sys.exit(1)
    
    # ROS2の初期化
    rclpy.init(args=args)
    
    try:
        # ノードの作成と実行
        node = CsvMessageSender(sys.argv[1:])
        
        print(f"CSV Message Sender started with {len(sys.argv)-1} files")
        print("Press Enter to see available commands...")
        
        # ノードの実行
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main() 