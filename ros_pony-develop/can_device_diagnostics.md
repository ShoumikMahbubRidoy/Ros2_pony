# CANデバイス診断ガイド

## 概要
CANデバイスの接続問題を診断するためのツールを提供します。USB2CANデバイスの接続状態、ドライバー、権限などを包括的に確認できます。

## 診断ツール

### 1. 包括的診断ツール
```bash
./run_can_device_checker.sh
```

このツールは以下の項目を自動的に診断します：
- システム情報（OS、カーネルバージョン）
- USBデバイスの接続状態
- CANインターフェースの存在
- カーネルモジュールのロード状態
- 権限とデバイスファイル
- ネットワーク設定
- システムログ
- 推奨事項

### 2. 強化されたCANノード
```bash
./run_drive_usb2can.sh
```

通常のCANノードに診断機能を追加しました：
- 初期診断の自動実行
- 定期的な接続状態監視
- 詳細なエラー情報の記録
- 診断結果のパブリッシュ

## 診断項目詳細

### USBデバイス確認
```bash
# USBデバイスの一覧表示
lsusb

# USBデバイスIDの確認
ls /sys/bus/usb/devices/
```

### CANインターフェース確認
```bash
# ネットワークインターフェースの確認
ip link show

# CANインターフェースの確認
ip link show can0

# CANインターフェースの統計
ip -s link show can0
```

### カーネルモジュール確認
```bash
# ロードされているモジュールの確認
lsmod | grep can

# 必要なCANモジュール
# - can
# - can_raw
# - can_dev
# - usb2can（USB2CANデバイス用）
```

### 権限確認
```bash
# USBシリアルデバイスの確認
ls -la /dev/ttyUSB*

# CANデバイスの確認
ls -la /dev/can*

# ユーザーグループの確認
groups

# dialoutグループへの追加（必要に応じて）
sudo usermod -a -G dialout $USER
```

### システムログ確認
```bash
# CAN関連のカーネルメッセージ
dmesg | grep -i can

# USB2CAN関連のメッセージ
dmesg | grep -i usb2can
```

## よくある問題と解決方法

### 1. USB2CANデバイスが認識されない
**症状**: `lsusb`でUSB2CANデバイスが表示されない

**解決方法**:
```bash
# USBデバイスの再接続
sudo usb-reset

# または、デバイスを物理的に抜き差し

# ドライバーの確認
lsmod | grep usb2can

# ドライバーの手動ロード（必要に応じて）
sudo modprobe usb2can
```

### 2. CANインターフェースが存在しない
**症状**: `ip link show`でcan0が表示されない

**解決方法**:
```bash
# CANインターフェースの作成
sudo ip link add dev can0 type can bitrate 1000000

# インターフェースの有効化
sudo ip link set up can0

# 設定の確認
ip link show can0
```

### 3. 権限エラー
**症状**: "Permission denied"エラーが発生

**解決方法**:
```bash
# ユーザーをdialoutグループに追加
sudo usermod -a -G dialout $USER

# ログアウト・ログインしてグループ変更を反映
# または
newgrp dialout

# デバイスファイルの権限変更（一時的な解決）
sudo chmod 666 /dev/ttyUSB*
```

### 4. カーネルモジュールがロードされていない
**症状**: `lsmod`でCANモジュールが表示されない

**解決方法**:
```bash
# 必要なモジュールのロード
sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev

# USB2CANドライバーのロード
sudo modprobe usb2can

# ロード状態の確認
lsmod | grep can
```

### 5. デバイスドライバーがインストールされていない
**症状**: USB2CANデバイスが認識されない

**解決方法**:
```bash
# カーネルヘッダーのインストール
sudo apt update
sudo apt install linux-headers-$(uname -r)

# USB2CANドライバーのインストール
# （メーカーの公式ドライバーを使用）
```

## 診断結果の解釈

### 正常な状態
```
✓ USB2CAN device detected
✓ CAN interfaces found: ['can0']
✓ CAN module can is loaded
✓ CAN module can_raw is loaded
✓ User is in dialout group
```

### 問題がある状態
```
✗ USB2CAN device not found
✗ No CAN interfaces found
✗ CAN module usb2can is not loaded
✗ User is not in dialout group
```

## 手動診断コマンド

### 基本的な確認
```bash
# 1. USBデバイスの確認
lsusb

# 2. CANインターフェースの確認
ip link show

# 3. カーネルモジュールの確認
lsmod | grep can

# 4. 権限の確認
groups
ls -la /dev/ttyUSB*

# 5. システムログの確認
dmesg | grep -i can
dmesg | grep -i usb2can
```

### 詳細な確認
```bash
# USBデバイスの詳細情報
lsusb -v

# CANインターフェースの詳細
ip -d link show can0

# カーネルモジュールの詳細
modinfo can
modinfo can_raw
modinfo can_dev

# システムログの詳細
journalctl -f | grep -i can
```

## トラブルシューティングの流れ

1. **診断ツールの実行**
   ```bash
   ./run_can_device_checker.sh
   ```

2. **問題の特定**
   - 診断結果から問題箇所を特定
   - エラーメッセージの確認

3. **解決方法の適用**
   - 上記の解決方法を順次適用
   - 各ステップ後に再診断

4. **動作確認**
   ```bash
   # CANノードの実行
   ./run_drive_usb2can.sh
   
   # テストノードの実行
   ./run_drive_usb2can_detailed_tester.sh
   ```

## 注意事項

- **権限**: 一部の診断コマンドには管理者権限が必要です
- **ドライバー**: USB2CANデバイスには専用ドライバーが必要な場合があります
- **カーネル**: カーネルバージョンによって対応状況が異なる場合があります
- **設定**: CANインターフェースの設定はシステムによって異なる場合があります

## 追加の診断情報

### システム情報の確認
```bash
# OS情報
uname -a

# カーネルバージョン
cat /proc/version

# ハードウェア情報
lshw | grep -i usb
```

### ネットワーク設定の確認
```bash
# ネットワークインターフェースの詳細
ip addr show

# ルーティングテーブル
ip route show

# ネットワーク統計
ss -i
```

この診断ツールを使用して、CANデバイスの接続問題を効率的に特定・解決できます。 