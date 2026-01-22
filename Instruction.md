# TODO

加一下delay

改一下文字长度和delay基本保持一致





# 项目架构

三个ROS节点

g1_bridge：机器人上负责语音收发

robot_task_tools：部署在端侧，负责使用LLM来做对话与调度

rllab_deploy_ros：部署在机器人上负责切换状态机与实际的动作执行



# 编译整个项目

对于机器人的两个node，直接使用colcon来编译

cd ~/cxy/ControlUnitreeG1withROS && source /opt/ros/foxy/setup.bash && colcon build --packages-select g1_29dof_controller g1_bridge



对于端侧的一个node，使用python来构建

conda activate cap310

source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID=1



env -u HTTP_PROXY -u HTTPS_PROXY -u ALL_PROXY -u http_proxy -u https_proxy -u all_proxy \
python3 -m robot_task_tools.prompt_router_node

# 运行运行项目

运行运动模块

cd /home/unitree/cxy/ControlUnitreeG1withROS/src/rllab_deploy_ros/robots/g1_29dof

./run_g1_ctrl.sh 

运行语言模块

cd /home/unitree/cxy/ControlUnitreeG1withROS/src

./run_g1_voice.sh



运行调度模块

conda activate cap310

source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID=1

env -u HTTP_PROXY -u HTTPS_PROXY -u ALL_PROXY -u http_proxy -u https_proxy -u all_proxy \
python3 -m robot_task_tools.prompt_router_node

# 打开每个终端前

cd ~/cxy/ControlUnitreeG1withROS

*# 设置环境变量*

export UNITREE_SDK_ROOT=/home/unitree/fanyh/unitree_sdk

export ROS_DOMAIN_ID=1

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

*# Source ROS2环境*

source /opt/ros/foxy/setup.bash

source install/setup.bash



# 运动













# 被动模式

ros2 topic pub /robot/motion_code std_msgs/msg/String "data: 'Passive'"

# 固定站立

ros2 topic pub /robot/motion_code std_msgs/msg/String "data: 'FixStand'" --once

# 速度控制

ros2 topic pub /robot/motion_code std_msgs/msg/String "data: 'Velocity'"

# 舞蹈模式

ros2 topic pub /robot/motion_code std_msgs/msg/String "data: 'Mimic_Dance_102'"

# 踢腿

ros2 topic pub /robot/motion_code std_msgs/msg/String "data: 'Mimic_Kick'"

# 高尔夫挥杆

ros2 topic pub /robot/motion_code std_msgs/msg/String "data: 'Mimic_Golf'"

# 网球挥拍

ros2 topic pub /robot/motion_code std_msgs/msg/String "data: 'Mimic_tennis'"

# 热身

ros2 topic pub /robot/motion_code std_msgs/msg/String "data: 'Mimic_warm_up'"

# 拳击

ros2 topic pub /robot/motion_code std_msgs/msg/String "data: 'Mimic_Boxer'"

# 自信步伐

ros2 topic pub /robot/motion_code std_msgs/msg/String "data: 'Mimic_confident'"

# 僵尸走路

ros2 topic pub /robot/motion_code std_msgs/msg/String "data: 'Mimic_zombie'"

# 反时针圆圈

ros2 topic pub /robot/motion_code std_msgs/msg/String "data: 'Mimic_anti_clock'"

# 指挥

ros2 topic pub /robot/motion_code std_msgs/msg/String "data: 'Mimic_Conductor'"

# 警察

ros2 topic pub /robot/motion_code std_msgs/msg/String "data: 'Mimic_police'"

# 跑步

ros2 topic pub /robot/motion_code std_msgs/msg/String "data: 'Mimic_runner'"

# 窄梁行走

ros2 topic pub /robot/motion_code std_msgs/msg/String "data: 'Mimic_narrow_beam'"

# 触碰膝盖

ros2 topic pub /robot/motion_code std_msgs/msg/String "data: 'Mimic_touch_knee'"

# 江南Style

ros2 topic pub /robot/motion_code std_msgs/msg/String "data: 'Mimic_Gangnam_Style'"





# 说话模块

export ROS_DOMAIN_ID=1

 





发送简单文字

export ROS_DOMAIN_ID=1

source /opt/ros/foxy/setup.bash

ros2 topic pub /robot/say std_msgs/msg/String "data: '你好，我是G1机器人'"

发送英文

ros2 topic pub /robot/say std_msgs/msg/String "data: 'Hello, I am G1 robot'"

发送较长文本

ros2 topic pub /robot/say std_msgs/msg/String "data: '欢迎使用Unitree G1机器人系统'"



```
===================================
  G1 Voice System
===================================
Starting ASR and TTS bridges...
ROS_DOMAIN_ID: 1
===================================
Starting G1 ASR Bridge (iface: eth0)...
ASR Bridge started with PID: 10100
Starting G1 TTS Bridge (iface: eth0)...
TTS Bridge started with PID: 10127

===================================
  Voice modules started!
===================================
ASR Bridge PID: 10100
TTS Bridge PID: 10127

To stop: kill 10100 10127
Press Ctrl+C to exit
===================================
```
