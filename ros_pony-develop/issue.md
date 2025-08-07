# ROS2 Pony プロジェクト - 課題とTBD記録

## 概要
このドキュメントは、`joint_can_converter`ノードの実装時に発生した課題と、TBD（To Be Determined）として残された部分を記録しています。

## 実装済み機能

### ✅ 完了した機能
1. **基本的なノード構造**
   - ROS2ノードの初期化
   - パラメータ設定
   - パブリッシャー/サブスクライバーの設定
   - サービスの設定

2. **CAN通信機能**
   - 28ビットCAN IDの計算（モードID: 6, デバイスID: 0x68-0x73）
   - 8バイトデータパッケージング（位置、速度、加速度）
   - `can_msgs/Frame`の送信

3. **関節コマンド変換**
   - `sensor_msgs/JointState`からCANコマンドへの変換
   - ラジアンから度への変換
   - スケーリング係数の適用

4. **初期化シーケンス**
   - システム初期化サービスの実装
   - ホーミングシーケンスの基本構造
   - 状態管理とログ出力

5. **カスタムメッセージ**
   - `MotorFeedback.msg`
   - `SystemStatus.msg`
   - `InitializationStatus.msg`
   - `AimDegree.msg`

## TBD（To Be Determined）部分

### 🔄 AK60-6モータフィードバック形式
**場所**: `joint_can_converter_node.py` - `_parse_motor_feedback`メソッド
```python
def _parse_motor_feedback(self, device_id: int, data: bytes) -> None:
    """
    TBD: AK60-6モータのフィードバックデータを解析
    現在はダミー実装
    """
    # TODO: 実際のAK60-6フィードバック形式に基づいて実装
    # - 角度データの解析
    # - 電流データの解析
    # - 温度データの解析
    # - エラー状態の解析
```

### 🔄 AK60-6ホーミングコマンド
**場所**: `joint_can_converter_node.py` - `_start_homing`メソッド
```python
def _start_homing(self) -> None:
    """
    TBD: AK60-6モータのホーミングコマンドを実装
    現在はタイマーによるシミュレーション
    """
    # TODO: 実際のAK60-6ホーミングコマンドを実装
    # - ホーミング開始コマンド
    # - ホーミング完了検出
    # - エラー処理
```

### 🔄 モータ関節名と物理配置
**場所**: `joint_can_converter_node.py` - 初期化部分
```python
# TBD: 実際のロボットの物理配置に基づいて定義
self.joint_names = [
    'front_left_hip',    # TODO: 実際の関節名
    'front_left_thigh',  # TODO: 実際の関節名
    # ... 他の関節名
]

self.device_id_to_joint = {
    0x68: 'front_left_hip',    # TODO: 実際のマッピング
    0x69: 'front_left_thigh',  # TODO: 実際のマッピング
    # ... 他のマッピング
}
```

### 🔄 システム状態確認の詳細実装
**場所**: `joint_can_converter_node.py` - `_check_system_status`メソッド
```python
def _check_system_status(self) -> bool:
    """
    TBD: システム状態の詳細確認を実装
    現在は基本的なチェックのみ
    """
    # TODO: 実際のシステム状態確認を実装
    # - モータの接続状態確認
    # - 通信状態確認
    # - エラー状態確認
    # - 温度・電流の閾値チェック
```

## 技術的課題

### 🚨 launchファイルの問題
**問題**: `ros2 launch`コマンドでノードが起動しない
**原因**: CMakeのターゲット重複エラー
```
CMake Error: add_custom_target cannot create target "ament_cmake_python_copy_ros2_pony"
because another target with the same name already exists.
```

**解決策**: 
1. **直接Python実行**（推奨）:
   ```bash
   python3 src/ros2_pony/ros2_pony/joint_can_converter_node.py --ros-args -p mode_id:=6 -p device_id_start:=104
   ```

2. **修正版launchファイル**:
   ```bash
   ros2 launch launch_joint_can_converter_fixed.launch.py
   ```

### 🚨 メッセージ型の生成問題
**問題**: `ModuleNotFoundError: No module named 'ros2_pony.msg'`
**原因**: `ament_cmake`と`ament_python`の混在による複雑な相互作用

**解決策**: 
- 現在は`ament_cmake`を使用してメッセージ生成
- Pythonパッケージの実行は直接Pythonファイルを使用

## 実機動作テスト結果

### ✅ 成功したテスト
1. **ノード起動**: 正常に起動し、ログ出力を確認
2. **システム初期化**: サービス呼び出しが正常に動作
3. **関節コマンド送信**: 様々な角度でのコマンド送信が成功
4. **CAN通信**: CANフレームの送信が正常に動作

### 📊 テストデータ
```bash
# システム初期化
ros2 service call /initialize_system std_srvs/srv/Trigger

# 関節コマンド送信テスト
ros2 topic pub -1 /joint_command sensor_msgs/msg/JointState "{name: ['front_left_hip'], position: [0.5]}"
ros2 topic pub -1 /joint_command sensor_msgs/msg/JointState "{name: ['front_left_thigh'], position: [50.0]}"
```

## 今後の改善点

### 🔧 優先度：高
1. **AK60-6フィードバック形式の実装**
   - 実際のモータ仕様書に基づく実装
   - データ解析アルゴリズムの実装

2. **AK60-6ホーミングコマンドの実装**
   - 実際のホーミングシーケンスの実装
   - エラー処理の強化

3. **関節名と物理配置の定義**
   - ロボットの実際の構成に基づく定義
   - デバイスIDと関節名のマッピング

### 🔧 優先度：中
1. **システム状態監視の強化**
   - 詳細な状態確認機能
   - エラー検出と通知機能

2. **launchファイルの問題解決**
   - CMake設定の最適化
   - パッケージ構造の見直し

### 🔧 優先度：低
1. **パフォーマンス最適化**
   - メモリ使用量の最適化
   - 処理速度の向上

2. **テストカバレッジの向上**
   - 単体テストの追加
   - 統合テストの強化

## 参考資料

### 技術仕様
- **CAN ID**: 28ビット（モードID: 6, デバイスID: 0x68-0x73）
- **データ形式**: 8バイト（位置4バイト、速度2バイト、加速度2バイト）
- **角度範囲**: -36000度から36000度
- **速度範囲**: -327680ERPMから327680ERPM
- **加速度範囲**: 0から327680ERPM/s2

### ファイル構成
```
src/ros2_pony/
├── ros2_pony/
│   ├── joint_can_converter_node.py  # メインノード
│   ├── msg/                         # カスタムメッセージ
│   │   ├── MotorFeedback.msg
│   │   ├── SystemStatus.msg
│   │   ├── InitializationStatus.msg
│   │   └── AimDegree.msg
│   └── ...
├── CMakeLists.txt                   # ビルド設定
├── package.xml                      # パッケージ設定
└── setup.py                        # Pythonパッケージ設定
```

### launchファイル
- `launch_joint_can_converter.launch.py` - 元のバージョン（問題あり）
- `launch_joint_can_converter_fixed.launch.py` - 修正版（推奨）
- `launch_joint_can_converter_simple.launch.py` - 簡易版

---

**最終更新**: 2025-07-25
**作成者**: AI Assistant
**ステータス**: 実機動作テスト成功、TBD部分の実装待ち 