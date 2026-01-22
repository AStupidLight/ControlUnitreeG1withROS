# G1 Bridge - ROS2 语音桥接节点

## 📋 功能说明

这个包提供了两个桥接节点，用于连接 ROS2 和 Unitree G1 机器人的语音系统：

### 1. `g1_asr_bridge` - 语音识别桥接
- **订阅**: Unitree DDS 话题 `rt/audio_msg` (语音识别结果)
- **发布**: ROS2 话题 `/voice/prompt` (识别出的文本)
- **功能**: 将机器人的语音识别结果转发到 ROS2 系统

### 2. `g1_tts_bridge` - 文本转语音桥接
- **订阅**: ROS2 话题 `/robot/say` (要说的文本)
- **功能**: 将文本转换为语音并通过机器人播放
- **使用**: 让机器人通过语音回复

## 🚀 使用方法

### 方法 1: 使用启动脚本（推荐）

启动脚本会自动设置正确的库路径，避免 DDS 版本冲突：

```bash
cd /home/unitree/cxy/ros2_ws

# Source 环境
source /opt/ros/foxy/setup.bash
source install/setup.bash

# 运行 ASR 桥接
./src/g1_bridge/run_g1_bridge.sh asr_bridge --ros-args -p iface:=eth0

# 运行 TTS 桥接
./src/g1_bridge/run_g1_bridge.sh tts_bridge --ros-args -p iface:=eth0
```

### 方法 2: 直接运行（需手动设置库路径）

```bash
cd /home/unitree/cxy/ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash

# 设置库路径
export UNITREE_SDK_ROOT="/home/unitree/fanyh/unitree_sdk"
export LD_LIBRARY_PATH="${UNITREE_SDK_ROOT}/thirdparty/lib/aarch64:${LD_LIBRARY_PATH}"

# 运行节点
ros2 run g1_bridge g1_asr_bridge --ros-args -p iface:=eth0
ros2 run g1_bridge g1_tts_bridge --ros-args -p iface:=eth0
```

## 📝 参数说明

- `iface`: 网络接口名称（必需）
  - 示例: `eth0`, `wlan0` 等
  - 使用 `ifconfig` 或 `ip addr` 查看可用接口

## 🎮 使用示例

### 监听语音识别结果

**终端 1 - 启动 ASR 桥接：**
```bash
./src/g1_bridge/run_g1_bridge.sh asr_bridge --ros-args -p iface:=eth0
```

**终端 2 - 监听识别结果：**
```bash
ros2 topic echo /voice/prompt
```

### 让机器人说话

**终端 1 - 启动 TTS 桥接：**
```bash
./src/g1_bridge/run_g1_bridge.sh tts_bridge --ros-args -p iface:=eth0
```

**终端 2 - 发送文本：**
```bash
ros2 topic pub /robot/say std_msgs/String "data: '你好，我是G1机器人'"
```

## 🔧 故障排除

### 问题: `symbol lookup error: undefined symbol: ddsi_sertype_v0`
**根本原因**: ROS Foxy 自带的 CycloneDDS 库缺少 `ddsi_sertype_v0` 符号，而 Unitree SDK 需要这个符号。

**解决方案**: 使用 `run_g1_bridge.sh` 启动脚本，它会：
1. 使用 `LD_PRELOAD` 强制加载 Unitree SDK 的 DDS 库
2. 设置 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` 避免 ROS 加载 CycloneDDS
3. 确保库路径优先级正确

**验证方法**:
```bash
# 检查加载的库是否正确
ldd /home/unitree/cxy/ros2_ws/build/g1_bridge/g1_tts_bridge | grep ddsc
# 应该看到 unitree_sdk 的库路径，而不是 /opt/ros/foxy 的路径
```

### 问题: `bad_alloc` 错误
**可能原因**:
- AudioClient 初始化失败
- 确保机器人已连接且音频系统正常
- 检查网络接口名称是否正确

### 问题: 找不到网络接口
**解决**: 
```bash
# 查看可用网络接口
ifconfig
# 或
ip addr

# 使用正确的接口名称
ros2 run g1_bridge g1_asr_bridge --ros-args -p iface:=<your_interface>
```

## 📚 相关话题

- `/voice/prompt` (std_msgs/String): 语音识别结果
- `/robot/say` (std_msgs/String): 要转换为语音的文本

## 🛠️ 编译

```bash
cd /home/unitree/cxy/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select g1_bridge
source install/setup.bash
```
