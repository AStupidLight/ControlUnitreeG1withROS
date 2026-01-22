#!/bin/bash
# G1 Voice System Launcher
# 只启动语音识别和语音合成功能

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
cd "${SCRIPT_DIR}"

# Source ROS2 环境
source /opt/ros/foxy/setup.bash
source install/setup.bash 2>/dev/null

echo "==================================="
echo "  G1 Voice System"
echo "==================================="
echo "Starting ASR and TTS bridges..."
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "==================================="

# 检查参数是否设置
if [ -z "${ROS_DOMAIN_ID}" ]; then
    export ROS_DOMAIN_ID=1
fi

# 自动检测可用的网络接口
detect_iface() {
    # 优先级：eth0, wlan0, usb0
    for iface in eth0 wlan0 usb0; do
        if ip link show $iface > /dev/null 2>&1; then
            echo $iface
            return
        fi
    done
    echo "eth0"  # 默认值
}

IFACE="${IFACE:-$(detect_iface)}"

# 启动ASR桥接
echo "Starting G1 ASR Bridge (iface: ${IFACE})..."
ros2 run g1_bridge g1_asr_bridge --ros-args -p iface:=${IFACE} > /tmp/asr_bridge.log 2>&1 &
ASR_PID=$!
echo "ASR Bridge started with PID: ${ASR_PID}"

# 等待一下
sleep 2

# 启动TTS桥接
echo "Starting G1 TTS Bridge (iface: ${IFACE})..."
ros2 run g1_bridge g1_tts_bridge --ros-args -p iface:=${IFACE} > /tmp/tts_bridge.log 2>&1 &
TTS_PID=$!
echo "TTS Bridge started with PID: ${TTS_PID}"

echo ""
echo "==================================="
echo "  Voice modules started!"
echo "==================================="
echo "ASR Bridge PID: ${ASR_PID}"
echo "TTS Bridge PID: ${TTS_PID}"
echo ""
echo "To stop: kill ${ASR_PID} ${TTS_PID}"
echo "Press Ctrl+C to exit"
echo "==================================="

# 等待用户中断
trap "echo 'Stopping voice launcher...'; exit 0" INT
while true; do
    sleep 1
done