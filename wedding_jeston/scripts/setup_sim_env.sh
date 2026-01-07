#!/bin/bash
# 仿真环境准备脚本
# 用于快速设置和构建仿真调试环境

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WEDDING_DIR="$( dirname "$SCRIPT_DIR" )"
WORKSPACE_DIR="/home/lingjing/project/engine_ai/engineai_ros2_workspace"

echo "=========================================="
echo "婚礼互动机器人仿真环境准备"
echo "=========================================="

# 检查 ROS2 环境
if [ -z "$ROS_DISTRO" ]; then
    echo "正在加载 ROS2 环境..."
    source /opt/ros/humble/setup.bash
fi

echo "ROS2 版本: $ROS_DISTRO"

# 设置环境变量
export ROS_DOMAIN_ID=69
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"

# 进入工作空间
cd "$WORKSPACE_DIR"
echo ""
echo "工作空间: $WORKSPACE_DIR"

# 检查是否需要安装第三方依赖
if [ ! -d "$WORKSPACE_DIR/install/mujoco_simulator" ]; then
    echo ""
    echo "正在安装第三方依赖..."
    ./src/third_party/install.sh
fi

# 创建软链接
if [ ! -L "$WORKSPACE_DIR/src/wedding_jeston" ]; then
    echo ""
    echo "创建 wedding_jeston 软链接..."
    ln -sf "$WEDDING_DIR" "$WORKSPACE_DIR/src/wedding_jeston"
fi

# 构建仿真包
echo ""
echo "正在构建仿真环境..."
./scripts/build_nodes.sh sim

# 构建 wedding_interaction
echo ""
echo "正在构建 wedding_interaction..."
colcon build --packages-select wedding_interaction

# source 环境
source install/setup.bash

echo ""
echo "=========================================="
echo "环境准备完成！"
echo "=========================================="
echo ""
echo "下一步："
echo ""
echo "1. 启动仿真器（新终端）："
echo "   cd $WORKSPACE_DIR"
echo "   source install/setup.bash"
echo "   ros2 launch mujoco_simulator mujoco_simulator.launch.py"
echo ""
echo "2. 启动 FSM 调试（新终端）："
echo "   cd $WORKSPACE_DIR"
echo "   source install/setup.bash"
echo "   ros2 launch wedding_interaction idle_debug.launch.py"
echo ""
echo "3. 监控状态（新终端）："
echo "   source $WORKSPACE_DIR/install/setup.bash"
echo "   ros2 topic echo /wedding/fsm/state"
echo ""

