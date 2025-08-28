# CSV Message Sender

CSVファイルからROS2メッセージを送信するPythonスクリプトです。

## 機能

- 複数のCSVファイルを読み込み
- キーボード入力による制御
- タイムスタンプに基づくリアルタイム再生
- motion_planner_nodeと同じQoS設定

## CSVファイル形式

CSVファイルは以下の形式である必要があります：

```
sensor_msgs/JointState,/joint_target
timestamp,front_left_hip,front_left_thigh,front_left_calf,front_right_hip,front_right_thigh,front_right_calf,rear_left_hip,rear_left_thigh,rear_left_calf,rear_right_hip,rear_right_thigh,rear_right_calf
0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0
1.0,0.1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0
```

- **1行目**: メッセージ型とトピック名（カンマ区切り）
- **2行目**: カラム名（ヘッダー）
- **3行目以降**: データ行（タイムスタンプ, 関節角度...）

## 使用方法

### 1. スクリプトの起動

```bash
# ROS2環境の設定
source install/setup.bash

# スクリプトの実行（複数のCSVファイルを指定）
python3 csv_message_sender.py file1.csv file2.csv file3.csv
```

### 2. キーボード制御

スクリプト起動後、以下のキー入力で制御できます：

- **数字キー（1-9）**: 対応するCSVファイルを再生
- **Enter**: 使用方法を表示
- **Ctrl+C**: スクリプトを終了

### 3. 再生の動作

- タイムスタンプに基づいてリアルタイムで再生
- 再生中に別のファイルを選択すると、現在の再生を停止して新しいファイルを再生
- 再生終了後はキー入力受付状態に戻る

## 例

```bash
# テストファイルで動作確認
python3 csv_message_sender.py test_data1.csv test_data2.csv

# 実際のデータファイルで使用
python3 csv_message_sender.py csvPattern/joint_data_20250805_104712.csv
```

## 対応メッセージ型

現在対応しているメッセージ型：
- `sensor_msgs/JointState`

## QoS設定

motion_planner_nodeと同じQoS設定を使用：
- Reliability: BEST_EFFORT
- Durability: VOLATILE
- History: KEEP_LAST
- Depth: 10

## 注意事項

- CSVファイルの指定がない場合はスクリプトが終了します
- 無効なCSVファイルはスキップされます
- 再生中にエラーが発生した場合はログに出力されます
- キーボード入力は非ブロッキングで処理されます 