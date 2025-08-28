# CANフレーム詳細設定の使用例

## 概要
`drive_usb2can_detailed_tester`を使用して、詳細なCANフレーム設定でテストを実行できます。

## 基本的な使用方法

### 1. デフォルト設定での実行
```bash
./run_drive_usb2can_detailed_tester.sh
```
- CAN ID: 0x123
- DLC: 4
- Data: [0x01, 0x02, 0x03, 0x04]
- 送信間隔: 5秒

### 2. カスタムCAN IDでの実行
```bash
./run_drive_usb2can_detailed_tester.sh --can-id 0x456
```

### 3. カスタムデータでの実行
```bash
./run_drive_usb2can_detailed_tester.sh --data 0A,0B,0C,0D,0E,0F,10,11
```

### 4. 拡張フレームでの実行
```bash
./run_drive_usb2can_detailed_tester.sh --can-id 0x18FF1234 --extended
```

### 5. リモートフレームでの実行
```bash
./run_drive_usb2can_detailed_tester.sh --can-id 0x123 --remote
```

### 6. エラーフレームでの実行
```bash
./run_drive_usb2can_detailed_tester.sh --can-id 0x123 --error
```

### 7. 送信間隔の変更
```bash
./run_drive_usb2can_detailed_tester.sh --interval 1.0
```

## 実用的な例

### AK60-6モータ用のテストメッセージ

#### 1. 位置制御コマンド
```bash
# 位置制御コマンド (ID: 0x001)
./run_drive_usb2can_detailed_tester.sh \
  --can-id 0x001 \
  --dlc 8 \
  --data 00,00,00,00,00,00,00,00
```

#### 2. 速度制御コマンド
```bash
# 速度制御コマンド (ID: 0x002)
./run_drive_usb2can_detailed_tester.sh \
  --can-id 0x002 \
  --dlc 8 \
  --data 00,00,00,00,00,00,00,00
```

#### 3. 電流制御コマンド
```bash
# 電流制御コマンド (ID: 0x003)
./run_drive_usb2can_detailed_tester.sh \
  --can-id 0x003 \
  --dlc 8 \
  --data 00,00,00,00,00,00,00,00
```

#### 4. モータ状態取得リクエスト
```bash
# モータ状態取得リクエスト (ID: 0x004)
./run_drive_usb2can_detailed_tester.sh \
  --can-id 0x004 \
  --dlc 0 \
  --remote
```

### 複数のモータを同時にテスト

#### ターミナル1: モータ1のテスト
```bash
./run_drive_usb2can_detailed_tester.sh \
  --can-id 0x001 \
  --dlc 8 \
  --data 00,00,00,00,00,00,00,00 \
  --interval 2.0
```

#### ターミナル2: モータ2のテスト
```bash
./run_drive_usb2can_detailed_tester.sh \
  --can-id 0x002 \
  --dlc 8 \
  --data 00,00,00,00,00,00,00,00 \
  --interval 2.0
```

## パラメータ詳細

### --can-id
- 形式: 16進数（0x123 または 123）
- 範囲: 0x000 ～ 0x7FF（標準フレーム）、0x00000000 ～ 0x1FFFFFFF（拡張フレーム）
- デフォルト: 0x123

### --dlc
- 形式: 整数
- 範囲: 0 ～ 8
- デフォルト: 4

### --data
- 形式: 16進数のカンマ区切り（01,02,03,04）
- 長さ: DLCで指定した長さまで
- デフォルト: 01,02,03,04

### --extended
- 説明: 拡張フレーム形式を使用
- デフォルト: 標準フレーム

### --remote
- 説明: リモートフレームを送信
- デフォルト: データフレーム

### --error
- 説明: エラーフレームを送信
- デフォルト: 正常フレーム

### --interval
- 形式: 浮動小数点数
- 単位: 秒
- デフォルト: 5.0

## 注意事項

1. **CAN IDの競合**: 同じCAN IDを使用する他のデバイスがないことを確認してください
2. **データ長**: DLCとデータの長さが一致していることを確認してください
3. **拡張フレーム**: 拡張フレームを使用する場合、CAN IDは29ビット以内である必要があります
4. **リモートフレーム**: リモートフレームの場合、データは無視されます
5. **エラーフレーム**: エラーフレームは通常の通信に影響を与える可能性があります

## トラブルシューティング

### よくあるエラー

#### 1. 無効なCAN ID
```
Error: CAN ID must be between 0x000 and 0x7FF for standard frames
```
- 標準フレームの場合、CAN IDは0x000～0x7FFの範囲内である必要があります

#### 2. データ長の不一致
```
Error: Data length does not match DLC
```
- DLCで指定した長さとデータの長さが一致していることを確認してください

#### 3. 無効なデータ形式
```
Error: Invalid data format
```
- データは16進数のカンマ区切りで指定してください（例: 01,02,03,04）

## 高度な使用例

### 複雑なテストシーケンス
```bash
# 1秒間隔で異なるCAN IDを送信
for id in 0x001 0x002 0x003 0x004; do
  ./run_drive_usb2can_detailed_tester.sh \
    --can-id $id \
    --dlc 8 \
    --data 00,00,00,00,00,00,00,00 \
    --interval 1.0 &
  sleep 0.5
done
```

### ログレベルの設定
```bash
./run_drive_usb2can_detailed_tester.sh \
  --can-id 0x123 \
  --ros-args --log-level debug
``` 