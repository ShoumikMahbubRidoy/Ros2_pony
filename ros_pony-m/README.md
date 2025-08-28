# ros_pony

ROS2開発用のリポジトリです。

## 環境情報

### OS
- **OS**: Ubuntu 24.04.2 LTS (Noble Numbat)
- **Kernel**: Linux 6.11.0-29-generic

### ROS2
- **ROS2 ディストリビューション**: Jazzy (2024年5月リリース)
- **ROS2 バージョン**: 2.0.2

## 前提条件

このリポジトリを使用するには、以下の環境が必要です：

- Ubuntu 24.04 LTS 以降
- ROS2 Jazzy がインストール済み
- Python 3.12+
- CMake 3.28+
- colcon ビルドシステム

## セットアップ手順

### 1. リポジトリのクローン

```bash
git clone git@github.com:k-okina-d1/ros_pony.git
cd ros_pony
```

### 2. 仮想環境の構築

このプロジェクトではPython仮想環境を使用しています。以下の手順で環境を構築してください：

```bash
# Python仮想環境の作成
python3 -m venv venv

# 仮想環境の有効化
source venv/bin/activate

# 依存関係のインストール
pip install -r requirements.txt

# 仮想環境の無効化（必要に応じて）
deactivate
```

**注意**: 仮想環境を使用する際は、必ず以下のコマンドで有効化してから作業してください：

```bash
source venv/bin/activate
```

### 3. 依存関係のインストール

```bash
# システムの更新
sudo apt update && sudo apt upgrade

# ROS2 Jazzyのインストール（まだインストールされていない場合）
# 詳細は https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html を参照

# 開発に必要なツールのインストール
sudo apt install python3-colcon-common-extensions python3-vcstool
```

### 4. ワークスペースの確認

```bash
# 既存のワークスペース構造を確認
ls -la

# 既存のパッケージを確認
ls src/

# 必要に応じて新しいパッケージを追加
cd src
# ros2 pkg create --build-type ament_cmake <new_package_name>
# ros2 pkg create --build-type ament_python <new_package_name>
cd ..
```

### 5. ビルド

```bash
# 依存関係の解決とビルド
colcon build

# または、特定のパッケージのみビルド
colcon build --packages-select <package-name>
```

### 6. 環境のセットアップ

```bash
# ワークスペースの環境をソース
source install/setup.bash

# または、.bashrcに追加して永続化
echo "source ~/work/ros_pony/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## QoS設定に関する注意事項

### 重要なQoS設定

このプロジェクトでは、異なるPC間での通信を確実にするため、以下のQoS設定を使用しています：

#### 推奨QoS設定

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# センサデータ・制御コマンド用のQoS設定
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,  # 低遅延を重視
    durability=DurabilityPolicy.VOLATILE,       # 過去のデータは不要
    history=HistoryPolicy.KEEP_LAST,            # 最新のデータのみ保持
    depth=10                                    # キューサイズ
)
```

#### QoS設定の重要性

- **ReliabilityPolicy.BEST_EFFORT**: 制御システムでは低遅延が重要で、一部のメッセージが失われても問題ない
- **DurabilityPolicy.VOLATILE**: 過去のセンサデータは意味がないため
- **HistoryPolicy.KEEP_LAST**: 最新のデータのみを保持し、メモリ使用量を抑制

#### 異なるPC間での通信時の注意点

1. **QoS設定の一致**: パブリッシャーとサブスクライバー間でQoS設定が一致しない場合、メッセージが受信されません
2. **環境変数の設定**: 以下の環境変数を両方のPCで設定してください

```bash
# 送信側・受信側両方のPCで設定
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

3. **QoS設定の確認**: 以下のコマンドでQoS設定を確認できます

```bash
# トピックのQoS設定を確認
ros2 topic info /joint_target --verbose
ros2 topic info /joint_command --verbose
```

#### トラブルシューティング

QoS設定の不一致による問題が発生した場合：

```bash
# 警告メッセージの例
[WARN] [motion_planner_node]: New publisher discovered on topic 'joint_target', offering incompatible QoS. No messages will be received from it. Last incompatible policy: RELIABILITY
```

この警告が表示された場合は、パブリッシャーとサブスクライバー間でQoS設定を一致させてください。

## 使用方法

**重要**: 以下のコマンドを実行する前に、必ず仮想環境を有効化してください：

```bash
source venv/bin/activate
```

### 基本的なROS2コマンド

```bash
# ノードの実行
ros2 run <package_name> <executable_name>

# トピックの確認
ros2 topic list
ros2 topic echo /topic_name

# トピックへの送信テスト例（CAN送信）
ros2 topic pub /can_send can_msgs/msg/Frame "{id: 1640, dlc: 8, data: [255, 0, 0, 0, 0, 255, 0, 255], is_error: false, is_rtr: false, is_extended: false}"

# 単発で送信したい場合は -1 または --once オプションを付けてください
ros2 topic pub -1 /can_send can_msgs/msg/Frame "{id: 1640, dlc: 8, data: [255, 0, 0, 0, 0, 255, 0, 255], is_error: false, is_rtr: false, is_extended: false}"
# または
ros2 topic pub --once /can_send can_msgs/msg/Frame "{id: 1640, dlc: 8, data: [255, 0, 0, 0, 0, 255, 0, 255], is_error: false, is_rtr: false, is_extended: false}"

# サービスの確認
ros2 service list
ros2 service call /service_name <service_type> <arguments>

# パラメータの確認
ros2 param list
ros2 param get /node_name parameter_name
```

### CAN通信の使用

CAN通信を使用する場合は、[CAN_SETUP.md](CAN_SETUP.md)を参照してください。

```bash
# CANノードの起動
source venv/bin/activate
source src/ros2_pony/install/setup.bash
PYTHONPATH=$PYTHONPATH:$(pwd)/venv/lib/python3.12/site-packages python3 src/ros2_pony/ros2_pony/drive_usb2can_node.py --ros-args -p can_interface:=socketcan -p enable_system_setup:=true

# CAN通信のテスト
python3 src/ros2_pony/test/test_can_send.py --can-id 0x668 --data 00,00,00,00,00,ff,00,ff --interval 0.5

# joint_can_converterノードの起動
ros2 launch launch_joint_can_converter.launch.py

# システム初期化
ros2 service call /initialize_system std_srvs/srv/Trigger

# 関節コマンドの送信テスト
ros2 topic pub /joint_command sensor_msgs/msg/JointState "{name: ['front_left_hip'], position: [0.5]}"
```
ros2 param list
ros2 param get /node_name parameter_name
```

### 開発用コマンド

```bash
# パッケージの作成
ros2 pkg create --build-type ament_cmake <package_name>
ros2 pkg create --build-type ament_python <package_name>

# テストの実行
colcon test
colcon test-result --verbose

# コードのフォーマット
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
```

## テスト

このプロジェクトでは、ユニットテストと統合テストの両方を実装しています。

### テストの種類

1. **ユニットテスト** (`test_processor_node.py`)
   - 個々のコンポーネント（クラス、メソッド）の動作をテスト
   - ノードの作成、メッセージ処理、エラーハンドリング等を検証
   - 外部依存関係を最小限に抑えたテスト

2. **統合テスト** (`test_processor_node_integration.py`)
   - 実際のROS2環境でのノード間通信をテスト
   - トピックの送受信、メッセージの流れを検証
   - エンドツーエンドの動作確認

### テストの実行方法

#### シェルスクリプトを使用（推奨）

```bash
# ヘルプの表示
./run_tests.sh --help

# 機能テストのみ実行（デフォルト）
./run_tests.sh

# ユニットテストのみ実行
./run_tests.sh -u

# 統合テストのみ実行
./run_tests.sh -i

# 詳細出力付きで実行
./run_tests.sh -v

# カバレッジレポート付きで実行
./run_tests.sh -c

# HTMLレポート付きで実行
./run_tests.sh -r

# カバレッジとレポートを同時に生成
./run_tests.sh -c -r

# 全テスト（スタイルチェック含む）を実行
./run_tests.sh -a
```

#### 手動でテストを実行

```bash
# 環境のセットアップ
source install/setup.bash

# 機能テストのみの実行（推奨）
python3 -m pytest src/ros2_pony/test/ -k "not flake8 and not pep257" -v

# または、特定のテストファイルのみ
python3 -m pytest src/ros2_pony/test/test_processor_node.py src/ros2_pony/test/test_processor_node_integration.py -v
```

#### コードスタイルチェックを含む全テスト

```bash
# コードスタイルチェックも含む全テスト（注意：スタイルエラーがあると失敗）
python3 -m pytest src/ros2_pony/test/ -v
```

#### ユニットテストのみ実行

```bash
# ユニットテストの実行
python3 -m pytest src/ros2_pony/test/test_processor_node.py -v
```

#### 統合テストのみ実行

```bash
# 統合テストの実行
python3 -m pytest src/ros2_pony/test/test_processor_node_integration.py -v
```

#### 特定のテストメソッドのみ実行

```bash
# 特定のテストメソッドを実行
python3 -m pytest src/ros2_pony/test/test_processor_node.py::TestProcessorNode::test_message_processing -v

# テスト名でフィルタリング
python3 -m pytest src/ros2_pony/test/ -k "message" -v
```

### テストの内容

#### ユニットテスト項目

- `test_node_creation`: ノードの正常な作成
- `test_subscription_creation`: サブスクリプションの作成
- `test_publisher_creation`: パブリッシャーの作成
- `test_message_processing`: メッセージ加工処理
- `test_message_callback`: コールバック関数の動作
- `test_empty_message_handling`: 空メッセージの処理
- `test_special_characters_handling`: 特殊文字の処理

#### 統合テスト項目

- `test_message_flow`: メッセージの流れ（受信→加工→送信）
- `test_multiple_messages`: 複数メッセージの連続処理
- `test_empty_message`: 空メッセージの統合テスト
- `test_special_characters`: 特殊文字の統合テスト

### テスト実行時の注意点

1. **環境の準備**
   ```bash
   # ROS2環境のセットアップ
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   ```

2. **テストの並列実行**
   ```bash
   # 並列実行（推奨）
   python3 -m pytest src/ros2_pony/test/ -n auto -v
   ```

3. **テストレポートの生成**
   ```bash
   # HTMLレポートの生成
   python3 -m pytest src/ros2_pony/test/ --html=test_report.html --self-contained-html
   ```

4. **カバレッジの確認**
   ```bash
   # カバレッジ付きでテスト実行
   python3 -m pytest src/ros2_pony/test/ --cov=ros2_pony --cov-report=html
   ```

### トラブルシューティング

#### よくあるテストエラー

1. **ImportError: No module named 'ros2_pony'**
   ```bash
   # ビルドを実行してからテスト
   colcon build --packages-select ros2_pony
   source install/setup.bash
   ```

2. **テストがタイムアウトする**
   ```bash
   # タイムアウト時間を延長
   python3 -m pytest src/ros2_pony/test/ --timeout=30 -v
   ```

3. **ROS2環境の問題**
   ```bash
   # ROS2環境の確認
   printenv | grep ROS
   
   # 環境の再読み込み
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   ```

### シェルスクリプトの機能

`run_tests.sh`スクリプトは以下の機能を提供します：

#### 自動化機能
- **環境チェック**: ROS2、Python3、pytestの存在確認
- **ビルドチェック**: 必要に応じて自動ビルド実行
- **環境セットアップ**: ROS2環境とワークスペース環境の自動設定
- **エラーハンドリング**: エラー時の適切な処理とメッセージ表示

#### オプション機能
- **色付き出力**: 情報、成功、警告、エラーを色分けして表示
- **詳細ログ**: 各ステップの進行状況を表示
- **柔軟なテスト選択**: ユニット、統合、機能、全テストから選択
- **レポート生成**: カバレッジとHTMLレポートの生成

#### 使用例
```bash
# 基本的な機能テスト
./run_tests.sh

# ユニットテストを詳細出力で
./run_tests.sh -u -v

# 統合テストにカバレッジとレポートを追加
./run_tests.sh -i -c -r

# 全テスト（スタイルチェック含む）
./run_tests.sh -a
```

### 新しいテストの追加

新しいテストを追加する場合は、以下の構造に従ってください：

```python
import pytest
import rclpy
from std_msgs.msg import String

from ros2_pony.processor_node import ProcessorNode


class TestNewFeature:
    """新しい機能のテストクラス."""

    @pytest.fixture(autouse=True)
    def setup_teardown(self):
        """テストの前後処理."""
        rclpy.init()
        yield
        rclpy.shutdown()

    def test_new_feature(self):
        """新しい機能のテスト."""
        # テストコードをここに記述
        pass
```

## プロジェクト構造

```
ros_pony/
├── src/                    # ソースコード
│   └── ros2_pony/         # メインのROS2パッケージ
│       ├── ros2_pony/     # Pythonパッケージディレクトリ
│       ├── test/          # テストファイル
│       ├── resource/      # リソースファイル
│       ├── package.xml    # パッケージ定義
│       └── setup.py       # Pythonセットアップファイル
├── build/                 # ビルドファイル（.gitignoreで除外）
├── install/               # インストールファイル（.gitignoreで除外）
├── log/                   # ログファイル（.gitignoreで除外）
└── README.md             # このファイル
```

## トラブルシューティング

### よくある問題

1. **ビルドエラー**
   ```bash
   # クリーンビルド
   rm -rf build/ install/ log/
   colcon build
   ```

2. **環境変数の問題**
   ```bash
   # ROS2環境の確認
   printenv | grep ROS
   
   # 環境の再読み込み
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   ```

3. **パッケージが見つからない**
   ```bash
   # パッケージの検索
   ros2 pkg list | grep <package_name>
   
   # 依存関係の確認
   rosdep check <package_name>
   ```

## 開発ガイドライン

- ROS2の命名規則に従ってパッケージ名を付ける
- 適切なライセンスファイルを含める
- ドキュメントを充実させる
- テストを書く
- コードフォーマットを統一する

## ライセンス

[ライセンス情報を記載]

## 貢献

[貢献方法を記載]

## 参考リンク

- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [ROS2 Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html)
- [colcon Documentation](https://colcon.readthedocs.io/)