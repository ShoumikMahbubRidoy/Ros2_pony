#!/bin/bash

# ROS2 Pony 機能テスト実行スクリプト
# 使用方法: ./run_tests.sh [オプション]

set -e  # エラー時にスクリプトを停止

# 色付き出力の定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# ヘルプメッセージ
show_help() {
    echo "ROS2 Pony 機能テスト実行スクリプト"
    echo ""
    echo "使用方法: $0 [オプション]"
    echo ""
    echo "オプション:"
    echo "  -h, --help     このヘルプメッセージを表示"
    echo "  -u, --unit     ユニットテストのみ実行"
    echo "  -i, --integration 統合テストのみ実行"
    echo "  -v, --verbose  詳細出力"
    echo "  -c, --coverage カバレッジレポートを生成"
    echo "  -r, --report   HTMLレポートを生成"
    echo "  -a, --all      全テスト（スタイルチェック含む）を実行"
    echo ""
    echo "例:"
    echo "  $0                    # 機能テストのみ実行"
    echo "  $0 -u                 # ユニットテストのみ"
    echo "  $0 -i -v              # 統合テストを詳細出力で"
    echo "  $0 -c -r              # カバレッジとレポート生成"
}

# ログ関数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 環境チェック
check_environment() {
    log_info "環境チェック中..."
    
    # ROS2環境の確認
    if ! command -v ros2 &> /dev/null; then
        log_error "ROS2が見つかりません。環境をセットアップしてください。"
        exit 1
    fi
    
    # Python環境の確認
    if ! command -v python3 &> /dev/null; then
        log_error "Python3が見つかりません。"
        exit 1
    fi
    
    # pytestの確認
    if ! python3 -c "import pytest" &> /dev/null; then
        log_error "pytestがインストールされていません。"
        exit 1
    fi
    
    log_success "環境チェック完了"
}

# ビルドチェック
check_build() {
    log_info "ビルド状態をチェック中..."
    
    if [ ! -d "install/ros2_pony" ]; then
        log_warning "ビルドが必要です。ビルドを実行します..."
        colcon build --packages-select ros2_pony
    fi
    
    log_success "ビルドチェック完了"
}

# 環境セットアップ
setup_environment() {
    log_info "環境をセットアップ中..."
    
    # ROS2環境のセットアップ
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        source /opt/ros/jazzy/setup.bash
    fi
    
    # ワークスペース環境のセットアップ
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
    else
        log_error "install/setup.bashが見つかりません。ビルドを実行してください。"
        exit 1
    fi
    
    log_success "環境セットアップ完了"
}

# テスト実行
run_tests() {
    local test_type="$1"
    local verbose_flag="$2"
    local coverage_flag="$3"
    local report_flag="$4"
    
    log_info "テスト実行中: $test_type"
    
    # 基本コマンド
    local cmd="python3 -m pytest"
    
    # テストタイプに応じてコマンドを構築
    case "$test_type" in
        "unit")
            cmd="$cmd src/ros2_pony/test/test_processor_node.py"
            ;;
        "integration")
            cmd="$cmd src/ros2_pony/test/test_processor_node_integration.py"
            ;;
        "functional")
            cmd="$cmd src/ros2_pony/test/ -k \"not flake8 and not pep257\""
            ;;
        "all")
            cmd="$cmd src/ros2_pony/test/"
            ;;
        *)
            log_error "不明なテストタイプ: $test_type"
            exit 1
            ;;
    esac
    
    # オプションを追加
    if [ "$verbose_flag" = "true" ]; then
        cmd="$cmd -v"
    fi
    
    if [ "$coverage_flag" = "true" ]; then
        cmd="$cmd --cov=ros2_pony --cov-report=html --cov-report=term"
    fi
    
    if [ "$report_flag" = "true" ]; then
        cmd="$cmd --html=test_report.html --self-contained-html"
    fi
    
    # テスト実行
    log_info "実行コマンド: $cmd"
    echo ""
    
    if eval $cmd; then
        log_success "テスト完了: $test_type"
        return 0
    else
        log_error "テスト失敗: $test_type"
        return 1
    fi
}

# メイン処理
main() {
    local test_type="functional"
    local verbose_flag="false"
    local coverage_flag="false"
    local report_flag="false"
    
    # オプション解析
    while [[ $# -gt 0 ]]; do
        case $1 in
            -h|--help)
                show_help
                exit 0
                ;;
            -u|--unit)
                test_type="unit"
                shift
                ;;
            -i|--integration)
                test_type="integration"
                shift
                ;;
            -v|--verbose)
                verbose_flag="true"
                shift
                ;;
            -c|--coverage)
                coverage_flag="true"
                shift
                ;;
            -r|--report)
                report_flag="true"
                shift
                ;;
            -a|--all)
                test_type="all"
                shift
                ;;
            *)
                log_error "不明なオプション: $1"
                show_help
                exit 1
                ;;
        esac
    done
    
    # スクリプト開始
    echo "========================================"
    echo "ROS2 Pony 機能テスト実行スクリプト"
    echo "========================================"
    echo ""
    
    # 環境チェック
    check_environment
    
    # ビルドチェック
    check_build
    
    # 環境セットアップ
    setup_environment
    
    # テスト実行
    if run_tests "$test_type" "$verbose_flag" "$coverage_flag" "$report_flag"; then
        echo ""
        log_success "すべてのテストが正常に完了しました！"
        
        # レポートファイルの場所を表示
        if [ "$coverage_flag" = "true" ]; then
            echo ""
            log_info "カバレッジレポート: htmlcov/index.html"
        fi
        
        if [ "$report_flag" = "true" ]; then
            echo ""
            log_info "テストレポート: test_report.html"
        fi
        
        exit 0
    else
        echo ""
        log_error "テストが失敗しました。"
        exit 1
    fi
}

# スクリプト実行
main "$@" 