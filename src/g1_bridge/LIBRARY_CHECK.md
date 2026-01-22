# 库依赖检查报告

## ✅ DDS 库检查结果

### 1. DDS 库版本确认

**ROS Foxy 的 DDS 库** (`/opt/ros/foxy/lib/aarch64-linux-gnu/libddsc.so.0`):
- ❌ **没有** `ddsi_sertype_v0` 符号
- ✅ 有 `dds_create_topic` 等基本符号

**Unitree SDK 的 DDS 库** (`/home/unitree/fanyh/unitree_sdk/thirdparty/lib/aarch64/libddsc.so.0`):
- ✅ **有** `ddsi_sertype_v0` 符号
- ✅ 有 `dds_create_topic` 等符号

### 2. 当前库加载状态

**已验证**: 程序现在正确加载 Unitree SDK 的 DDS 库：
```
libddsc.so.0 => /home/unitree/fanyh/unitree_sdk/thirdparty/lib/aarch64/libddsc.so.0 ✅
libddscxx.so.0 => /home/unitree/fanyh/unitree_sdk/thirdparty/lib/aarch64/libddscxx.so.0 ✅
```

**libstdc++ 库**:
```
libstdc++.so.6 => /usr/lib/aarch64-linux-gnu/libstdc++.so.6 ✅ (系统版本，不是conda)
libgcc_s.so.1 => /usr/lib/aarch64-linux-gnu/libgcc_s.so.1 ✅ (系统版本)
```

### 3. RUNPATH 设置

已正确设置 RUNPATH，优先使用系统库：
```
RUNPATH: [/usr/lib/aarch64-linux-gnu:/home/unitree/fanyh/unitree_sdk/thirdparty/lib/aarch64:/home/unitree/fanyh/unitree_sdk/lib/aarch64:/opt/ros/foxy/lib]
```

## ⚠️ 当前问题

### bad_alloc 错误

**状态**: DDS 库问题已解决，但仍有 `bad_alloc` 错误

**可能原因**:
1. AudioClient 初始化时分配内存失败
2. 机器人未连接或音频服务未就绪
3. ChannelFactory 初始化失败
4. 其他 Unitree SDK 内部依赖问题

**建议排查步骤**:
1. 确保机器人已连接且电源开启
2. 检查网络接口是否正确
3. 确认机器人板载程序状态
4. 查看 Unitree SDK 文档中 AudioClient 的使用要求

## 📝 修复总结

### 已完成的修复

1. ✅ **DDS 库版本冲突**: 使用 `LD_PRELOAD` 强制加载 Unitree SDK 的 DDS 库
2. ✅ **libstdc++ 版本**: 设置 RUNPATH 优先使用系统库
3. ✅ **ROS2 RMW**: 设置 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` 避免 ROS 加载 CycloneDDS
4. ✅ **库路径优先级**: 正确设置 `LD_LIBRARY_PATH`

### 验证命令

```bash
# 检查 DDS 库
ldd /home/unitree/cxy/ros2_ws/build/g1_bridge/g1_tts_bridge | grep ddsc

# 检查 libstdc++
ldd /home/unitree/cxy/ros2_ws/build/g1_bridge/g1_tts_bridge | grep stdc++

# 检查 RUNPATH
readelf -d /home/unitree/cxy/ros2_ws/build/g1_bridge/g1_tts_bridge | grep RUNPATH
```
