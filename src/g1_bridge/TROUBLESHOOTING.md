# G1 Bridge 故障排除指南

## 🔍 DDS 库版本冲突问题（已修复）

### 问题现象
```
symbol lookup error: undefined symbol: ddsi_sertype_v0
```

### 根本原因
1. **ROS Foxy 的 CycloneDDS 库** (`/opt/ros/foxy/lib/aarch64-linux-gnu/libddsc.so.0`) **没有** `ddsi_sertype_v0` 符号
2. **Unitree SDK 的 CycloneDDS 库** (`/home/unitree/fanyh/unitree_sdk/thirdparty/lib/aarch64/libddsc.so.0`) **有** 这个符号
3. 程序运行时优先加载了 ROS Foxy 的库，导致符号缺失

### 验证方法

```bash
# 检查 ROS Foxy 的库（应该没有符号）
nm -D /opt/ros/foxy/lib/aarch64-linux-gnu/libddsc.so.0 | grep ddsi_sertype_v0
# 输出: (空，没有这个符号)

# 检查 Unitree SDK 的库（应该有符号）
nm -D /home/unitree/fanyh/unitree_sdk/thirdparty/lib/aarch64/libddsc.so.0 | grep ddsi_sertype_v0
# 输出: 0000000000034808 T ddsi_sertype_v0
```

### 解决方案

**使用启动脚本（推荐）**:
```bash
./src/g1_bridge/run_g1_bridge.sh tts_bridge --ros-args -p iface:=eth0
```

启动脚本会自动：
1. 使用 `LD_PRELOAD` 强制加载 Unitree SDK 的 DDS 库
2. 设置 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` 避免 ROS 加载 CycloneDDS
3. 设置正确的库路径优先级

**手动设置环境变量**:
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_PRELOAD=/home/unitree/fanyh/unitree_sdk/thirdparty/lib/aarch64/libddsc.so.0:/home/unitree/fanyh/unitree_sdk/thirdparty/lib/aarch64/libddscxx.so.0
export LD_LIBRARY_PATH=/home/unitree/fanyh/unitree_sdk/thirdparty/lib/aarch64:$LD_LIBRARY_PATH

ros2 run g1_bridge g1_tts_bridge --ros-args -p iface:=eth0
```

### 验证修复

```bash
# 检查程序加载的库
ldd /home/unitree/cxy/ros2_ws/build/g1_bridge/g1_tts_bridge | grep ddsc

# 应该看到：
# /home/unitree/fanyh/unitree_sdk/thirdparty/lib/aarch64/libddsc.so.0
# /home/unitree/fanyh/unitree_sdk/thirdparty/lib/aarch64/libddscxx.so.0

# 而不是：
# /opt/ros/foxy/lib/aarch64-linux-gnu/libddsc.so.0
```

---

## 🔍 bad_alloc 错误

### 问题现象
```
bad_alloc caught: std::bad_alloc
```

### 可能原因
1. AudioClient 初始化失败
2. 机器人未连接或音频系统未就绪
3. 网络接口配置错误

### 解决方案
1. 确保机器人已连接且电源开启
2. 检查网络接口名称是否正确：
   ```bash
   ifconfig
   # 或
   ip addr
   ```
3. 确保机器人板载程序已关闭（如果运行了其他控制程序）

---

## 🔍 其他常见问题

### 问题: 找不到网络接口
```bash
# 查看可用接口
ifconfig
ip addr

# 使用正确的接口名称
ros2 run g1_bridge g1_asr_bridge --ros-args -p iface:=eth0
```

### 问题: 程序无法启动
1. 检查是否 source 了 ROS2 环境：
   ```bash
   source /opt/ros/foxy/setup.bash
   source install/setup.bash
   ```
2. 检查可执行文件是否存在：
   ```bash
   ls -lh /home/unitree/cxy/ros2_ws/build/g1_bridge/g1_*_bridge
   ```

### 问题: 话题无数据
1. 检查节点是否正常运行：
   ```bash
   ros2 node list
   ros2 topic list
   ```
2. 检查话题数据：
   ```bash
   ros2 topic echo /voice/prompt  # ASR 输出
   ros2 topic echo /robot/say     # TTS 输入
   ```
