# CAN関連テストファイル一覧

このドキュメントでは、CAN通信に関連するテストファイルとその使用方法について説明します。

## 使用可能なテストファイル

### 1. 基本的なCAN送信テスト

**ファイル**: `src/ros2_pony/test/test_can_send.py`

**説明**: シンプルなCANメッセージ送信テスト

**使用方法**:
```bash
# 基本的な送信テスト
python3 src/ros2_pony/test/test_can_send.py --can-id 0x668 --data 00,00,00,00,00,ff,00,ff --interval 0.5

# パラメータ
--can-id: CAN ID（16進数形式、デフォルト: 0x668）
--data: データバイト（16進数形式、カンマ区切り）
--interval: 送信間隔（秒、デフォルト: 0.1）
```

### 2. 詳細なCANフレームテスト

**ファイル**: `src/ros2_pony/test/drive_usb2can_detailed_tester.py`

**説明**: 詳細なCANフレーム設定が可能なテスト

**使用方法**:
```bash
# 詳細テストの実行
python3 src/ros2_pony/test/drive_usb2can_detailed_tester.py --can-id 0x668 --dlc 8 --data ff,00,00,00,00,ff,00,ff --interval 5.0

# または、スクリプトを使用
./run_drive_usb2can_detailed_tester.sh --can-id 0x668 --dlc 8 --data ff,00,00,00,00,ff,00,ff

# パラメータ
--can-id: CAN ID（16進数形式）
--dlc: データ長コード
--data: データバイト（16進数形式、カンマ区切り）
--extended: 拡張フレーム形式を使用
--remote: リモートフレームを送信
--error: エラーフレームを送信
--interval: 送信間隔（秒）
```

### 3. CANデバイス診断

**ファイル**: `src/ros2_pony/test/can_device_checker.py`

**説明**: CANデバイスとシステム設定の包括的な診断

**使用方法**:
```bash
# デバイス診断の実行
python3 src/ros2_pony/test/can_device_checker.py

# または、スクリプトを使用
./run_can_device_checker.sh
```

### 4. 基本的なCANテスト

**ファイル**: `src/ros2_pony/test/drive_usb2can_tester.py`

**説明**: 基本的なCAN通信テスト

**使用方法**:
```bash
# 基本テストの実行
python3 src/ros2_pony/test/drive_usb2can_tester.py

# または、スクリプトを使用
./run_drive_usb2can_tester.sh
```

## 自動テストスクリプト

### 1. 改善されたCANテスト

**ファイル**: `test_improved_can.sh`

**説明**: 改善されたCANノードとテストスクリプトの自動実行（プロセス終了処理改善済み）

**使用方法**:
```bash
# 自動テストの実行
./test_improved_can.sh
```

### 2. シンプル版CANテスト

**ファイル**: `test_improved_can_simple.sh`

**説明**: シンプルで確実なプロセス終了処理を持つテストスクリプト

**使用方法**:
```bash
# シンプル版テストの実行
./test_improved_can_simple.sh
```

### 2. ランチファイル

**ファイル**: `launch_can_test.launch.py`

**説明**: CANテスト用のROS2ランチファイル

**使用方法**:
```bash
# ランチファイルの実行
ros2 launch launch_can_test.launch.py
```

## テスト実行の前提条件

1. **環境設定**:
   ```bash
   source venv/bin/activate
   source src/ros2_pony/install/setup.bash
   ```

2. **CANノードの起動**:
   ```bash
   PYTHONPATH=$PYTHONPATH:$(pwd)/venv/lib/python3.12/site-packages python3 src/ros2_pony/ros2_pony/drive_usb2can_node.py --ros-args -p can_interface:=socketcan -p enable_system_setup:=true
   ```

3. **USB2CANデバイスの接続確認**:
   ```bash
   lsusb | grep -i can
   ```

## テスト結果の確認

### トピックの確認

```bash
# 利用可能なトピックの確認
ros2 topic list

# 送信トピックの確認
ros2 topic echo /can_send --once

# 受信トピックの確認
ros2 topic echo /can_received --once

# 状態トピックの確認
ros2 topic echo /can_status --once
```

### ノードの確認

```bash
# 実行中のノードの確認
ros2 node list

# ノードの詳細情報
ros2 node info /drive_usb2can_node
```

## トラブルシューティング

### よくある問題

1. **ModuleNotFoundError**: python-canモジュールが見つからない
   - 解決方法: `CAN_SETUP.md`の「トラブルシューティング」セクションを参照

2. **CANインターフェースが見つからない**
   - 解決方法: USB2CANデバイスの接続確認とシステム設定

3. **権限エラー**
   - 解決方法: sudo権限の確認とデバイス権限の設定

### 診断コマンド

```bash
# CANインターフェースの状態確認
ip link show can0

# USBデバイスの確認
lsusb

# カーネルモジュールの確認
lsmod | grep can

# システムログの確認
dmesg | grep -i can
```

## ファイル構成

```
ros_pony/
├── src/ros2_pony/
│   ├── ros2_pony/
│   │   └── drive_usb2can_node.py    # メインのCANノード
│   └── test/
│       ├── test_can_send.py                    # シンプル送信テスト
│       ├── drive_usb2can_detailed_tester.py    # 詳細テスト
│       ├── drive_usb2can_tester.py             # 基本テスト
│       └── can_device_checker.py               # デバイス診断
├── test_improved_can.sh                        # 自動テストスクリプト
├── run_drive_usb2can_detailed_tester.sh       # 詳細テスト実行
├── run_drive_usb2can_tester.sh                # 基本テスト実行
├── run_can_device_checker.sh                  # デバイス診断実行
└── launch_can_test.launch.py                  # ランチファイル
```

## 注意事項

- すべてのテストはCANノードが起動している状態で実行してください
- USB2CANデバイスが正しく接続されていることを確認してください
- 仮想環境でのpython-canモジュールの利用にはPYTHONPATH設定が必要です
- システム設定にはsudo権限が必要です 