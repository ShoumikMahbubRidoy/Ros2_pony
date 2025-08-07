# CANデバイス設定ガイド

このドキュメントでは、USB2CANデバイスを使用したCAN通信の設定方法とテスト手順について説明します。

## 前提条件

- Ubuntu 22.04以降
- ROS2 Jazzy
- Python 3.12
- USB2CANデバイス（Geschwister Schneider CAN adapter等）

## 1. システム設定

### 1.1 必要なパッケージのインストール

```bash
# システムパッケージの更新
sudo apt update

# CAN関連パッケージのインストール
sudo apt install can-utils python3-can

# カーネルモジュールの確認
lsmod | grep can
```

### 1.2 USB2CANデバイスの確認

```bash
# USBデバイスの確認
lsusb | grep -i can

# 期待される出力例：
# Bus 003 Device 007: ID 1d50:606f OpenMoko, Inc. Geschwister Schneider CAN adapter
```

### 1.3 カーネルモジュールの読み込み

```bash
# CAN関連モジュールの読み込み
sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev
sudo modprobe gs_usb

# モジュールの確認
lsmod | grep can
```

## 2. CANインターフェースの設定

### 2.1 手動設定

```bash
# CANインターフェースの停止
sudo ifconfig can0 down

# CANインターフェースの設定（1Mbps）
sudo ip link set can0 type can bitrate 1000000

# 送信キュー長の設定
sudo ifconfig can0 txqueuelen 100000

# CANインターフェースの起動
sudo ifconfig can0 up

# 設定の確認
ip link show can0
```

### 2.2 自動設定（ROS2ノード使用時）

ROS2ノードは自動的にCANインターフェースを設定します：

```bash
# 環境設定
source venv/bin/activate
source src/ros2_pony/install/setup.bash

# CANノードの起動（自動設定有効）
PYTHONPATH=$PYTHONPATH:$(pwd)/venv/lib/python3.12/site-packages python3 src/ros2_pony/ros2_pony/drive_usb2can_node.py --ros-args -p can_interface:=socketcan -p enable_system_setup:=true
```

## 3. 権限設定

### 3.1 sudo権限の設定

CANインターフェースの設定にはsudo権限が必要です：

```bash
# sudoersファイルの編集（必要に応じて）
sudo visudo

# または、パスワードなしでsudoを許可
echo "$USER ALL=(ALL) NOPASSWD: /sbin/ifconfig, /sbin/ip" | sudo tee /etc/sudoers.d/can-setup
```

### 3.2 デバイス権限の確認

```bash
# USBデバイスの権限確認
ls -la /dev/ttyUSB*

# 必要に応じて権限を変更
sudo chmod 666 /dev/ttyUSB*
```

## 4. テスト手順

### 4.1 基本的なCAN通信テスト

```bash
# 1. CANノードの起動
source venv/bin/activate
source src/ros2_pony/install/setup.bash
PYTHONPATH=$PYTHONPATH:$(pwd)/venv/lib/python3.12/site-packages python3 src/ros2_pony/ros2_pony/drive_usb2can_node.py --ros-args -p can_interface:=socketcan -p enable_system_setup:=true &

# 2. テストスクリプトの実行
python3 src/ros2_pony/test/test_can_send.py --can-id 0x668 --data 00,00,00,00,00,ff,00,ff --interval 0.5

# 3. トピックの確認
ros2 topic list
ros2 topic echo /can_send --once
ros2 topic echo /can_received --once
```

### 4.2 詳細テスト

```bash
# 詳細なCANフレーム設定でのテスト
./run_drive_usb2can_detailed_tester.sh --can-id 0x668 --dlc 8 --data ff,00,00,00,00,ff,00,ff
```

### 4.3 自動テスト

```bash
# 自動テストスクリプトの実行
./test_improved_can.sh
```

## 5. トラブルシューティング

### 5.1 よくある問題

#### 問題1: `ModuleNotFoundError: No module named 'can'`

**解決方法:**
```bash
# 仮想環境でのpython-canの確認
source venv/bin/activate
pip list | grep can

# 必要に応じて再インストール
pip install python-can
```

#### 問題2: CANインターフェースが見つからない

**解決方法:**
```bash
# USBデバイスの確認
lsusb

# カーネルモジュールの確認
lsmod | grep can

# 手動でCANインターフェースを設定
sudo ifconfig can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ifconfig can0 txqueuelen 100000
sudo ifconfig can0 up
```

#### 問題3: 権限エラー

**解決方法:**
```bash
# sudo権限の確認
sudo -l

# デバイス権限の確認
ls -la /dev/ttyUSB*
```

### 5.2 診断コマンド

```bash
# CANインターフェースの状態確認
ip link show can0

# CAN統計情報の確認
cat /proc/net/can/stats

# カーネルメッセージの確認
dmesg | grep -i can

# USBデバイスの詳細確認
lsusb -v | grep -A 10 -B 10 "1d50:606f"
```

## 6. 参考資料

- [python-can ドキュメント](https://python-can.readthedocs.io/)
- [Linux CAN ドキュメント](https://www.kernel.org/doc/html/latest/networking/can.html)
- [USB2CAN ドキュメント](https://github.com/linux-can/can-utils)

## 7. ファイル構成

```
ros_pony/
├── src/ros2_pony/
│   ├── ros2_pony/
│   │   ├── drive_usb2can_node.py    # メインのCANノード
│   │   └── test_can_send.py         # シンプルな送信テスト
│   └── test/
│       ├── drive_usb2can_detailed_tester.py  # 詳細テスト
│       ├── drive_usb2can_tester.py           # 基本テスト
│       └── can_device_checker.py             # デバイス診断
├── test_improved_can.sh              # 自動テストスクリプト
├── run_drive_usb2can_detailed_tester.sh  # 詳細テスト実行
└── launch_can_test.launch.py         # ランチファイル
```

## 8. 注意事項

- CANインターフェースの設定にはsudo権限が必要です
- USB2CANデバイスが正しく接続されていることを確認してください
- 仮想環境でのpython-canモジュールの利用にはPYTHONPATH設定が必要です
- システム設定は自動的に実行されますが、手動設定も可能です 