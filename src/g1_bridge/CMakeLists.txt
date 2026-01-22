cmake_minimum_required(VERSION 3.8)
project(g1_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

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
  set(ARCH_SUFFIX "aarch64")  # 默认使用 aarch64
endif()
message(STATUS "Architecture: ${ARCH_SUFFIX}")

set(UNITREE_LIBS
  unitree_sdk2
  ddsc
  ddscxx
)

add_executable(g1_asr_bridge src/g1_asr_bridge.cpp)
target_include_directories(g1_asr_bridge PRIVATE
  ${UNITREE_SDK_ROOT}/include
  ${UNITREE_SDK_ROOT}/thirdparty/include
  ${UNITREE_SDK_ROOT}/thirdparty/include/ddscxx
)
target_link_directories(g1_asr_bridge PRIVATE
  ${UNITREE_SDK_ROOT}/lib
  ${UNITREE_SDK_ROOT}/lib/${ARCH_SUFFIX}
  ${UNITREE_SDK_ROOT}/thirdparty/lib/${ARCH_SUFFIX}
  /usr/lib/aarch64-linux-gnu
)
ament_target_dependencies(g1_asr_bridge
  rclcpp
  std_msgs
)
target_link_libraries(g1_asr_bridge
  ${UNITREE_LIBS}
)
# 设置RUNPATH，避免使用conda的库
set_target_properties(g1_asr_bridge PROPERTIES
  INSTALL_RPATH "/usr/lib/aarch64-linux-gnu:${UNITREE_SDK_ROOT}/thirdparty/lib/${ARCH_SUFFIX}:${UNITREE_SDK_ROOT}/lib/${ARCH_SUFFIX}:/opt/ros/foxy/lib"
  BUILD_WITH_INSTALL_RPATH TRUE
)

add_executable(g1_tts_bridge src/g1_tts_bridge.cpp)
target_include_directories(g1_tts_bridge PRIVATE
  ${UNITREE_SDK_ROOT}/include
  ${UNITREE_SDK_ROOT}/thirdparty/include
  ${UNITREE_SDK_ROOT}/thirdparty/include/ddscxx
)
target_link_directories(g1_tts_bridge PRIVATE
  ${UNITREE_SDK_ROOT}/lib
  ${UNITREE_SDK_ROOT}/lib/${ARCH_SUFFIX}
  ${UNITREE_SDK_ROOT}/thirdparty/lib/${ARCH_SUFFIX}
  /usr/lib/aarch64-linux-gnu
)
ament_target_dependencies(g1_tts_bridge
  rclcpp
  std_msgs
)
target_link_libraries(g1_tts_bridge
  ${UNITREE_LIBS}
)
# 设置RUNPATH，避免使用conda的库
set_target_properties(g1_tts_bridge PROPERTIES
  INSTALL_RPATH "/usr/lib/aarch64-linux-gnu:${UNITREE_SDK_ROOT}/thirdparty/lib/${ARCH_SUFFIX}:${UNITREE_SDK_ROOT}/lib/${ARCH_SUFFIX}:/opt/ros/foxy/lib"
  BUILD_WITH_INSTALL_RPATH TRUE
)

install(TARGETS
  g1_asr_bridge
  g1_tts_bridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
