cmake_minimum_required(VERSION 3.12)
project(g1_29dof_controller)

set(CMAKE_CXX_STANDARD 17)

# 忽略 conda 路径，避免库冲突
set(CMAKE_IGNORE_PATH "/home/unitree/miniconda3;/home/unitree/anaconda3")
list(APPEND CMAKE_IGNORE_PREFIX_PATH "/home/unitree/miniconda3" "/home/unitree/anaconda3")

# 使用 ament_cmake
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 显式设置 yaml-cpp（避免 find_package 找到错误版本）
set(YAML_CPP_INCLUDE_DIR "/usr/include")
set(YAML_CPP_LIBRARIES "/usr/lib/aarch64-linux-gnu/libyaml-cpp.so")

# Unitree SDK 路径
if(NOT DEFINED UNITREE_SDK_ROOT)
  set(UNITREE_SDK_ROOT "/home/unitree/fanyh/unitree_sdk" CACHE PATH "Unitree SDK root directory")
endif()
message(STATUS "UNITREE_SDK_ROOT=${UNITREE_SDK_ROOT}")

# 检测系统架构
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(ARCH_SUFFIX "aarch64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64")
  set(ARCH_SUFFIX "x86_64")
else()
  set(ARCH_SUFFIX "aarch64")
endif()
message(STATUS "Architecture: ${ARCH_SUFFIX}")

# ONNX Runtime 路径 (根据架构选择)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(ONNX_ROOT "${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-aarch64-1.22.0")
else()
  set(ONNX_ROOT "${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-x64-1.22.0")
endif()
message(STATUS "ONNX_ROOT=${ONNX_ROOT}")

include_directories(
  /usr/include/eigen3
  ${UNITREE_SDK_ROOT}/thirdparty/include
  ${UNITREE_SDK_ROOT}/thirdparty/include/ddscxx
  ${UNITREE_SDK_ROOT}/include
  include
  ${ONNX_ROOT}/include
  ${PROJECT_SOURCE_DIR}/../../include/
)

link_directories(
  ${UNITREE_SDK_ROOT}/lib/${ARCH_SUFFIX}
  ${UNITREE_SDK_ROOT}/thirdparty/lib/${ARCH_SUFFIX}
  /usr/lib/aarch64-linux-gnu
)

# 源文件
file(GLOB_RECURSE ADD_SRC_LIST
  ${PROJECT_SOURCE_DIR}/src/*.cpp
)

# 构建库
add_library(${PROJECT_NAME}_lib ${ADD_SRC_LIST})
target_link_libraries(${PROJECT_NAME}_lib
  unitree_sdk2
  ddsc
  ddscxx
  rt
  pthread
  Boost::program_options
  ${YAML_CPP_LIBRARIES}
  ${ONNX_ROOT}/lib/libonnxruntime.so.1.22.0
)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
)

# 构建可执行文件
add_executable(g1_ctrl main.cpp)
target_link_libraries(g1_ctrl
  ${PROJECT_NAME}_lib
  ${YAML_CPP_LIBRARIES}
)
ament_target_dependencies(g1_ctrl
  rclcpp
  std_msgs
)

# 设置 RPATH（确保系统库优先）
set_target_properties(g1_ctrl ${PROJECT_NAME}_lib PROPERTIES
  INSTALL_RPATH "/usr/lib/aarch64-linux-gnu:${UNITREE_SDK_ROOT}/lib/${ARCH_SUFFIX}:${UNITREE_SDK_ROOT}/thirdparty/lib/${ARCH_SUFFIX}:${ONNX_ROOT}/lib:/opt/ros/foxy/lib"
  BUILD_WITH_INSTALL_RPATH TRUE
)

# 安装
install(TARGETS
  g1_ctrl
  ${PROJECT_NAME}_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 创建符号链接，使程序能找到配置文件
# param.h 期望配置在可执行文件同目录或 config 子目录
install(CODE "
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E create_symlink
      ../../share/${PROJECT_NAME}/config
      \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/config
  )
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E create_symlink
      config/config.yaml
      \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/config.yaml
  )
")

ament_package()
