#!/bin/bash
# G1 29DOF Controller Launcher Script
# 设置正确的环境变量避免库冲突

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 清理 conda 环境
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL CONDA_PYTHON_EXE CONDA_EXE

# Unitree SDK 路径
export UNITREE_SDK_ROOT="${UNITREE_SDK_ROOT:-/home/unitree/fanyh/unitree_sdk}"
ARCH_SUFFIX="aarch64"

# ONNX Runtime 路径
ONNX_ROOT="${SCRIPT_DIR}/../../thirdparty/onnxruntime-linux-aarch64-1.22.0"

# 设置 ROS_DOMAIN_ID，避免与 Unitree SDK 的 DDS (Domain 0) 冲突
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-1}

# 使用 FastRTPS 避免 CycloneDDS 冲突
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 强制使用 Unitree SDK 的 DDS 库
export LD_PRELOAD="${UNITREE_SDK_ROOT}/thirdparty/lib/${ARCH_SUFFIX}/libddsc.so.0:${UNITREE_SDK_ROOT}/thirdparty/lib/${ARCH_SUFFIX}/libddscxx.so.0"

# 设置库路径（系统库优先）
export LD_LIBRARY_PATH="/usr/lib/aarch64-linux-gnu:${UNITREE_SDK_ROOT}/lib/${ARCH_SUFFIX}:${UNITREE_SDK_ROOT}/thirdparty/lib/${ARCH_SUFFIX}:${ONNX_ROOT}/lib:/opt/ros/foxy/lib"

# 进入工作空间
cd "${SCRIPT_DIR}/../../../.."

# Source ROS2 环境
source /opt/ros/foxy/setup.bash
source install/setup.bash 2>/dev/null

echo "==================================="
echo "  G1 29DOF Controller"
echo "==================================="
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "Config dir: ${SCRIPT_DIR}/config"
echo ""
echo "Running..."
echo "==================================="

# 运行程序
ros2 run g1_29dof_controller g1_ctrl "$@"
