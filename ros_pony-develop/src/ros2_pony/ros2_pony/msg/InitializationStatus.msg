# 初期化状態管理用カスタムメッセージ
uint8 state          # 初期化状態 (0: 未開始, 1: 進行中, 2: 完了, 3: エラー)
bool is_complete     # 完了フラグ
string status_msg    # 状態メッセージ 