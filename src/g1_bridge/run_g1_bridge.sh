#!/bin/bash
# G1 Bridge Launcher Script
# 设置正确的库路径以避免 DDS 版本冲突
# 根本原因：ROS Foxy 的 CycloneDDS 缺少 ddsi_sertype_v0 符号，必须使用 Unitree SDK 自带的 DDS 库

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 设置 unitree_sdk 的 DDS 库路径（优先使用）
export UNITREE_SDK_ROOT="${UNITREE_SDK_ROOT:-/home/unitree/fanyh/unitree_sdk}"

# 检测架构
if [ "$(uname -m)" = "aarch64" ]; then
    ARCH_SUFFIX="aarch64"
elif [ "$(uname -m)" = "x86_64" ]; then
    ARCH_SUFFIX="x86_64"
else
    ARCH_SUFFIX="aarch64"
fi

# 关键修复：强制使用 Unitree SDK 的 DDS 库
UNITREE_DDS_LIB_DIR="${UNITREE_SDK_ROOT}/thirdparty/lib/${ARCH_SUFFIX}"
UNITREE_DDS_LIB="${UNITREE_DDS_LIB_DIR}/libddsc.so.0"
UNITREE_DDSXX_LIB="${UNITREE_DDS_LIB_DIR}/libddscxx.so.0"

# 检查库文件是否存在
if [ ! -f "${UNITREE_DDS_LIB}" ] || [ ! -f "${UNITREE_DDSXX_LIB}" ]; then
    echo "Error: Unitree DDS libraries not found at ${UNITREE_DDS_LIB_DIR}"
    echo "Please check UNITREE_SDK_ROOT: ${UNITREE_SDK_ROOT}"
    exit 1
fi

# ============================================
# 关键：彻底清理 conda 环境，避免 libstdc++ 冲突
# ============================================
unset CONDA_PREFIX
unset CONDA_DEFAULT_ENV
unset CONDA_SHLVL
unset CONDA_PYTHON_EXE
unset CONDA_EXE

# 完全重置 LD_LIBRARY_PATH，移除所有 conda/miniconda/anaconda 路径
# 保留 ROS 和 CUDA 相关路径
CLEAN_LD_PATH=""
if [ -n "$LD_LIBRARY_PATH" ]; then
    IFS=':' read -ra PATHS <<< "$LD_LIBRARY_PATH"
    for p in "${PATHS[@]}"; do
        # 跳过 conda/miniconda/anaconda 相关路径
        if [[ "$p" != *"conda"* && "$p" != *"miniconda"* && "$p" != *"anaconda"* ]]; then
            if [ -n "$CLEAN_LD_PATH" ]; then
                CLEAN_LD_PATH="${CLEAN_LD_PATH}:${p}"
            else
                CLEAN_LD_PATH="${p}"
            fi
        fi
    done
fi

# 关键：让 ROS2 使用 FastRTPS，避免 ROS 自己加载 CycloneDDS
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 关键：设置 ROS2 使用不同的 Domain ID，避免与 Unitree SDK (Domain ID 0) 冲突
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-1}

# 方法1: 使用 LD_PRELOAD 强制加载正确的 DDS 库（最可靠）
export LD_PRELOAD="${UNITREE_DDS_LIB}:${UNITREE_DDSXX_LIB}"

# 方法2: 设置库路径优先级
# 重要：系统 libstdc++ 必须在最前面，避免 conda 的版本被加载
export LD_LIBRARY_PATH="/usr/lib/aarch64-linux-gnu:${UNITREE_DDS_LIB_DIR}:${UNITREE_SDK_ROOT}/lib/${ARCH_SUFFIX}:${CLEAN_LD_PATH}"

# 检查参数
if [ $# -eq 0 ]; then
    echo "Usage: $0 <asr_bridge|tts_bridge> [ros2_args...]"
    echo ""
    echo "Examples:"
    echo "  $0 asr_bridge --ros-args -p iface:=eth0"
    echo "  $0 tts_bridge --ros-args -p iface:=eth0"
    exit 1
fi

BRIDGE_TYPE=$1
shift

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

# 进入工作空间 (脚本在 src/g1_bridge/ 下，所以是 ../..)
cd "${SCRIPT_DIR}/../.."

# Source ROS2 环境
if [ -f "/opt/ros/foxy/setup.bash" ]; then
    source /opt/ros/foxy/setup.bash
fi

# Source 工作空间
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
fi

echo "==================================="
echo "  G1 Bridge Launcher"
echo "==================================="
echo "Bridge type: ${BRIDGE_TYPE}"
echo "Executable: ${EXECUTABLE}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "Library paths:"
echo "  - unitree_sdk DDS: ${UNITREE_SDK_ROOT}/thirdparty/lib/${ARCH_SUFFIX}"
echo ""
echo "Running..."
echo "==================================="

# 运行程序
ros2 run g1_bridge ${EXECUTABLE} "$@"
