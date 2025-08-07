# Drive USB2CAN ノード テスト手順

## 概要
このドキュメントでは、drive_usb2canノードのテスト方法を説明します。

## 前提条件
- ROS2 Jazzyがインストールされている
- usb2canデバイスが接続されている
- python-canパッケージがインストールされている

## テスト手順

### 1. ビルド
```bash
# ワークスペースでビルドを実行
colcon build --packages-select ros2_pony
source install/setup.bash
```

### 2. CANデバイスの確認
```bash
# CANデバイスが認識されているか確認
lsusb | grep -i can
ip link show | grep can
```

### 3. CANインターフェースの設定
```bash
# CANインターフェースを設定（必要に応じて）
sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_bcm
sudo modprobe can_dev

# usb2canデバイスを設定
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
```

### 4. ノードの起動
```bash
# ターミナル1: drive_usb2canノードを起動
./run_drive_usb2can.sh

# ターミナル2: テストノードを起動
./run_drive_usb2can_tester.sh
```

### 5. トピックの確認
```bash
# トピック一覧を確認
ros2 topic list

# 特定のトピックの情報を確認
ros2 topic info /can_received
ros2 topic info /can_send
ros2 topic info /can_status
```

### 6. メッセージの監視
```bash
# CAN受信メッセージを監視
ros2 topic echo /can_received

# CAN状態を監視
ros2 topic echo /can_status
```

## パラメータ設定

### ノード起動時のパラメータ指定
```bash
# カスタムパラメータでノードを起動
./run_drive_usb2can.sh --ros-args \
  -p can_interface:=usb2can \
  -p can_bitrate:=1000000 \
  -p can_channel:=can0 \
  -p receive_timeout:=1.0
```

### パラメータの確認
```bash
# 現在のパラメータを確認
ros2 param list /drive_usb2can_node
ros2 param get /drive_usb2can_node can_interface
```

## トラブルシューティング

### 1. CANデバイスが認識されない場合
```bash
# デバイスドライバーの確認
dmesg | grep -i can
lsmod | grep can

# usb2canドライバーの確認
lsmod | grep usb2can
```

### 2. 権限エラーが発生する場合
```bash
# ユーザーをdialoutグループに追加
sudo usermod -a -G dialout $USER
# ログアウト・ログイン後に再試行
```

### 3. CANインターフェースがダウンしている場合
```bash
# インターフェースの状態確認
ip link show can0

# インターフェースをアップ
sudo ip link set up can0
```

## 期待される動作

### 正常な動作
1. ノード起動時に「CAN bus initialized successfully」メッセージが表示される
2. `/can_status`トピックで定期的に状態が送信される
3. テストノードから送信されたメッセージが`/can_received`トピックで受信される

### エラー時の動作
1. CANデバイスが接続されていない場合、エラーメッセージが表示される
2. 送信失敗時には警告メッセージが表示される
3. 受信エラー時にはエラーログが出力される

## ログレベルの設定
```bash
# デバッグログを有効にする
./run_drive_usb2can.sh --ros-args --log-level debug
```

## 注意事項
- 実機テスト時は、CANバスの設定が正しいことを確認してください
- 送信するメッセージのIDが他のデバイスと競合しないように注意してください
- テスト終了時は、Ctrl+Cでノードを適切に終了してください 