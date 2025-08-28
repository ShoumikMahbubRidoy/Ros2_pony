#!/bin/bash

# 強化版CANテストスクリプト
# プロセスグループ管理と確実なクリーンアップ

set -e  # エラー時に終了

# グローバル変数
CAN_NODE_PID=""
SENDER_PID=""
CAN_NODE_PGID=""
SENDER_PGID=""

# シグナルハンドリング関数
cleanup() {
    echo "=== クリーンアップ開始 ==="
    
    # プロセスグループ全体を終了
    if [ ! -z "$CAN_NODE_PGID" ]; then
        echo "CANノードプロセスグループ ($CAN_NODE_PGID) を終了中..."
        kill -TERM -$CAN_NODE_PGID 2>/dev/null || true
        sleep 3
        kill -KILL -$CAN_NODE_PGID 2>/dev/null || true
    fi
    
    if [ ! -z "$SENDER_PGID" ]; then
        echo "送信テストプロセスグループ ($SENDER_PGID) を終了中..."
        kill -TERM -$SENDER_PGID 2>/dev/null || true
        sleep 3
        kill -KILL -$SENDER_PGID 2>/dev/null || true
    fi
    
    # 個別プロセスの終了
    if [ ! -z "$CAN_NODE_PID" ]; then
        echo "CANノードプロセス ($CAN_NODE_PID) を終了中..."
        kill -TERM $CAN_NODE_PID 2>/dev/null || true
        sleep 2
        kill -KILL $CAN_NODE_PID 2>/dev/null || true
    fi
    
    if [ ! -z "$SENDER_PID" ]; then
        echo "送信テストプロセス ($SENDER_PID) を終了中..."
        kill -TERM $SENDER_PID 2>/dev/null || true
        sleep 2
        kill -KILL $SENDER_PID 2>/dev/null || true
    fi
    
    # 関連プロセスの強制終了
    echo "関連プロセスを終了中..."
    pkill -f "drive_usb2can_node.py" 2>/dev/null || true
    pkill -f "test_can_send.py" 2>/dev/null || true
    pkill -f "simple_can_sender" 2>/dev/null || true
    
    # ROS2ノードの終了
    echo "ROS2ノードを終了中..."
    ros2 node kill /drive_usb2can_node 2>/dev/null || true
    ros2 node kill /simple_can_sender 2>/dev/null || true
    
    # 残存プロセスの確認と終了
    echo "残存プロセスの確認中..."
    sleep 2
    if pgrep -f "drive_usb2can_node.py" > /dev/null; then
        echo "残存CANノードプロセスを強制終了中..."
        pkill -9 -f "drive_usb2can_node.py" 2>/dev/null || true
    fi
    
    if pgrep -f "test_can_send.py" > /dev/null; then
        echo "残存送信テストプロセスを強制終了中..."
        pkill -9 -f "test_can_send.py" 2>/dev/null || true
    fi
    
    echo "=== クリーンアップ完了 ==="
}

# シグナルハンドラーの設定
trap cleanup SIGINT SIGTERM EXIT

echo "=== 強化版CAN送信テスト開始 ==="

# 環境設定
source src/ros2_pony/install/setup.bash

echo "1. 改善されたCANノードを起動..."
source venv/bin/activate && source src/ros2_pony/install/setup.bash && PYTHONPATH=$PYTHONPATH:$(pwd)/venv/lib/python3.12/site-packages python3 src/ros2_pony/ros2_pony/drive_usb2can_node.py --ros-args -p can_interface:=socketcan -p enable_system_setup:=true &
CAN_NODE_PID=$!
CAN_NODE_PGID=$(ps -o pgid= -p $CAN_NODE_PID 2>/dev/null | tr -d ' ') || ""

echo "CANノードPID: $CAN_NODE_PID, PGID: $CAN_NODE_PGID"

# ノードの起動を待つ
echo "CANノードの起動を待機中..."
sleep 5

# プロセスの生存確認
if ! kill -0 $CAN_NODE_PID 2>/dev/null; then
    echo "エラー: CANノードの起動に失敗しました"
    exit 1
fi

echo "2. シンプルな送信テストを実行..."
source venv/bin/activate && source src/ros2_pony/install/setup.bash && python3 src/ros2_pony/test/test_can_send.py --can-id 0x668 --data ff,00,00,00,00,ff,00,ff --interval 0.5 &
SENDER_PID=$!
SENDER_PGID=$(ps -o pgid= -p $SENDER_PID 2>/dev/null | tr -d ' ') || ""

echo "送信テストPID: $SENDER_PID, PGID: $SENDER_PGID"

# プロセスの生存確認
if ! kill -0 $SENDER_PID 2>/dev/null; then
    echo "エラー: 送信テストの起動に失敗しました"
    exit 1
fi

echo "3. 10秒間テストを実行中..."
for i in {1..10}; do
    echo "テスト実行中... ($i/10)"
    
    # プロセスの生存確認
    if ! kill -0 $CAN_NODE_PID 2>/dev/null; then
        echo "警告: CANノードが予期せず終了しました"
        break
    fi
    
    if ! kill -0 $SENDER_PID 2>/dev/null; then
        echo "警告: 送信テストが予期せず終了しました"
        break
    fi
    
    sleep 1
done

echo "4. テスト終了、プロセスを停止..."
# cleanup関数が自動的に実行される

echo "=== テスト完了 ===" 