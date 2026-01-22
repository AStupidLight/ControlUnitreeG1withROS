#!/bin/bash
# G1 Bridge Launcher Script (Simplified)
# 设置正确的环境变量避免库冲突

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 清理 conda 环境
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL CONDA_PYTHON_EXE CONDA_EXE

# Unitree SDK 路径
export UNITREE_SDK_ROOT="${UNITREE_SDK_ROOT:-/home/unitree/fanyh/unitree_sdk}"
ARCH_SUFFIX="aarch64"

# 设置 ROS_DOMAIN_ID，避免与 Unitree SDK 的 DDS (Domain 0) 冲突
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-1}

# 使用 FastRTPS 避免 CycloneDDS 冲突
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 强制使用 Unitree SDK 的 DDS 库
export LD_PRELOAD="${UNITREE_SDK_ROOT}/thirdparty/lib/${ARCH_SUFFIX}/libddsc.so.0:${UNITREE_SDK_ROOT}/thirdparty/lib/${ARCH_SUFFIX}/libddscxx.so.0"

# 设置库路径（系统库优先）
export LD_LIBRARY_PATH="/usr/lib/aarch64-linux-gnu:${UNITREE_SDK_ROOT}/lib/${ARCH_SUFFIX}:${UNITREE_SDK_ROOT}/thirdparty/lib/${ARCH_SUFFIX}:/opt/ros/foxy/lib"

# 进入工作空间
cd "${SCRIPT_DIR}/../.."

# Source ROS2 环境
source /opt/ros/foxy/setup.bash
source install/setup.bash 2>/dev/null

# 检查参数
if [ $# -eq 0 ]; then
    echo "Usage: $0 <asr_bridge|tts_bridge>"
    echo ""
    echo "Examples:"
    echo "  $0 asr_bridge"
    echo "  $0 tts_bridge"
    exit 1
fi

BRIDGE_TYPE=$1

# 确定要运行的程序
if [ "$BRIDGE_TYPE" = "asr_bridge" ]; then
    EXECUTABLE="g1_asr_bridge"
elif [ "$BRIDGE_TYPE" = "tts_bridge" ]; then
    EXECUTABLE="g1_tts_bridge"
else
    echo "Error: Unknown bridge type '$BRIDGE_TYPE'"
    echo "Use 'asr_bridge' or 'tts_bridge'"
    exit 1
fi

echo "==================================="
echo "  G1 Bridge (Simplified)"
echo "==================================="
echo "Bridge type: ${BRIDGE_TYPE}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo ""
echo "Running..."
echo "==================================="

# 运行程序
ros2 run g1_bridge ${EXECUTABLE}