#!/bin/bash

# シグナルハンドリング関数
cleanup() {
    echo "=== クリーンアップ開始 ==="
    
    # プロセスの終了
    if [ ! -z "$SENDER_PID" ]; then
        echo "送信テストプロセスを終了中..."
        kill -TERM $SENDER_PID 2>/dev/null || true
        sleep 2
        kill -KILL $SENDER_PID 2>/dev/null || true
    fi
    
    if [ ! -z "$CAN_NODE_PID" ]; then
        echo "CANノードプロセスを終了中..."
        kill -TERM $CAN_NODE_PID 2>/dev/null || true
        sleep 2
        kill -KILL $CAN_NODE_PID 2>/dev/null || true
    fi
    
    # 残存プロセスの強制終了
    echo "残存プロセスの強制終了中..."
    pkill -9 -f "drive_usb2can_node.py" 2>/dev/null || true
    pkill -9 -f "test_can_send.py" 2>/dev/null || true
    
    echo "=== クリーンアップ完了 ==="
    exit 0
}

# シグナルハンドラーの設定
trap cleanup SIGINT SIGTERM EXIT

echo "=== CAN送信テスト開始 ==="

# 環境設定
source src/ros2_pony/install/setup.bash

echo "1. 改善されたCANノードを起動..."
source venv/bin/activate && source src/ros2_pony/install/setup.bash && PYTHONPATH=$PYTHONPATH:$(pwd)/venv/lib/python3.12/site-packages python3 src/ros2_pony/ros2_pony/drive_usb2can_node.py --ros-args -p can_interface:=socketcan -p enable_system_setup:=true &
CAN_NODE_PID=$!

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

# プロセスの生存確認
if ! kill -0 $SENDER_PID 2>/dev/null; then
    echo "エラー: 送信テストの起動に失敗しました"
    cleanup
    exit 1
fi

echo "3. 10秒間テストを実行中..."
for i in {1..10}; do
    echo "テスト実行中... ($i/10)"
    sleep 1
    
    # プロセスの生存確認
    if ! kill -0 $CAN_NODE_PID 2>/dev/null; then
        echo "警告: CANノードが予期せず終了しました"
        break
    fi
    
    if ! kill -0 $SENDER_PID 2>/dev/null; then
        echo "警告: 送信テストが予期せず終了しました"
        break
    fi
done

echo "4. テスト終了、プロセスを停止..."
cleanup

echo "=== テスト完了 ===" 