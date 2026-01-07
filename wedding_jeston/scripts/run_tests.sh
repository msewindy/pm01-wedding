#!/bin/bash
# 运行测试脚本

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PKG_DIR="$( dirname "$SCRIPT_DIR" )"

# 进入包目录
cd "$PKG_DIR"

# 设置 PYTHONPATH
export PYTHONPATH="$PKG_DIR:$PYTHONPATH"

# 解析参数
TEST_TYPE="${1:-all}"

echo "=========================================="
echo "Running Wedding Interaction Tests"
echo "Test Type: $TEST_TYPE"
echo "=========================================="
echo ""

# 根据参数选择测试
case "$TEST_TYPE" in
    unit)
        echo "Running unit tests..."
        python3 -m pytest test/unit/ -v --tb=short
        RESULT=$?
        ;;
    integration)
        echo "Running integration tests..."
        echo "Note: Make sure ROS2 nodes are running!"
        python3 -m pytest test/integration/ -v --tb=short
        RESULT=$?
        ;;
    perception)
        echo "Running perception tests..."
        python3 -m pytest test/perception/ -v --tb=short
        RESULT=$?
        ;;
    states)
        echo "Running state tests..."
        python3 -m pytest test/states/ -v --tb=short
        RESULT=$?
        ;;
    all|*)
        echo "Running all tests..."
        python3 -m pytest test/unit/ test/integration/ test/perception/ test/states/ -v --tb=short
RESULT=$?
        ;;
esac

echo ""
echo "=========================================="
if [ $RESULT -eq 0 ]; then
    echo "All tests passed!"
else
    echo "Some tests failed!"
fi
echo "=========================================="

exit $RESULT

