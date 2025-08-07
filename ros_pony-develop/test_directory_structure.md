# テストディレクトリ構造

## 概要
テスト用のノードを`src/ros2_pony/test/`ディレクトリに移動し、適切な構造に整理しました。

## ディレクトリ構造

```
src/ros2_pony/
├── ros2_pony/
│   ├── __init__.py
│   ├── processor_node.py
│   └── drive_usb2can_node.py
└── test/
    ├── __pycache__/
    ├── drive_usb2can_tester.py          # 基本的なCANテストノード
    ├── drive_usb2can_detailed_tester.py # 詳細設定可能なCANテストノード
    ├── can_device_checker.py            # CANデバイス診断ツール
    ├── test_processor_node.py           # processor_nodeの単体テスト
    ├── test_processor_node_integration.py # processor_nodeの統合テスト
    ├── test_copyright.py                # 著作権テスト
    ├── test_flake8.py                  # コードスタイルテスト
    └── test_pep257.py                  # ドキュメントスタイルテスト
```

## 移動したファイル

### 1. drive_usb2can_tester.py
- **目的**: 基本的なCAN通信テスト
- **機能**: 
  - 定期的なテストメッセージ送信
  - CAN受信メッセージのログ出力
  - CAN状態の監視

### 2. drive_usb2can_detailed_tester.py
- **目的**: 詳細なCANフレーム設定でのテスト
- **機能**:
  - カスタムCAN ID、DLC、データの設定
  - 拡張フレーム、リモートフレーム、エラーフレームのテスト
  - 送信間隔の調整
  - コマンドライン引数による詳細設定

### 3. can_device_checker.py
- **目的**: CANデバイスの接続診断
- **機能**:
  - USBデバイスの確認
  - CANインターフェースの確認
  - カーネルモジュールの確認
  - 権限とデバイスファイルの確認
  - システムログの確認
  - 推奨事項の提供

## 実行方法

### 基本的なCANテスト
```bash
./run_drive_usb2can_tester.sh
```

### 詳細なCANテスト
```bash
./run_drive_usb2can_detailed_tester.sh --can-id 0x456 --data 0A,0B,0C,0D
```

### CANデバイス診断
```bash
./run_can_device_checker.sh
```

## 変更点

### 1. ファイルの移動
- テスト用ノードを`src/ros2_pony/test/`に移動
- メインのノードは`src/ros2_pony/ros2_pony/`に残存

### 2. インポートパスの調整
- testディレクトリからros2_ponyパッケージをインポートするためのパス設定を追加
- 各ファイルに`sys.path.insert()`を追加

### 3. setup.pyの更新
- テスト用ノードのエントリーポイントを削除
- メインのノードのみエントリーポイントに残存

### 4. 実行スクリプトの更新
- 直接Pythonファイルを実行するように変更
- パスを`src/ros2_pony/test/`に更新

## 利点

1. **構造の整理**: テスト用ノードとメインのノードが明確に分離
2. **保守性の向上**: テスト関連のファイルが一箇所に集約
3. **実行の簡素化**: 直接Pythonファイルを実行することで依存関係を簡素化
4. **開発効率の向上**: テストとメインコードの分離により開発が効率的

## 注意事項

- テスト用ノードは`src/ros2_pony/test/`ディレクトリから直接実行
- メインのノードは従来通りROS2のエントリーポイントから実行
- テスト用ノードは独立して実行可能（ROS2パッケージの依存関係なし）

この構造により、テスト用のノードが適切に整理され、開発とテストが効率的に行えるようになりました。 