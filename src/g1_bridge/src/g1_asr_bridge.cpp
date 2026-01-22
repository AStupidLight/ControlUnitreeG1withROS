#include <chrono>
#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/idl/ros2/String_.hpp>

#define AUDIO_SUBSCRIBE_TOPIC "rt/audio_msg"

using namespace unitree::robot;

// 全局发布器给 Unitree 的 C 风格回调用
static rclcpp::Publisher<std_msgs::msg::String>::SharedPtr g_pub;

// 提取 ASR 文字（你原来那套）
static std::string extract_text(const std::string& json_str) {
  std::string key = "\"text\":\"";
  size_t start = json_str.find(key);
  if (start == std::string::npos) return "";
  start += key.length();
  size_t end = json_str.find("\"", start);
  if (end == std::string::npos) return "";
  return json_str.substr(start, end - start);
}

// Unitree DDS 回调：收到人声识别 JSON -> 发布 /voice/prompt
static void asr_handler(const void* msg) {
  auto* resMsg = (std_msgs::msg::dds_::String_*)msg;
  std::string text = extract_text(resMsg->data());
  if (text.empty()) return;

  std_msgs::msg::String out;
  out.data = text;
  if (g_pub) g_pub->publish(out);

  std::cout << "[ASR] " << text << std::endl;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("g1_asr_bridge");

  // iface 参数：用来初始化 Unitree ChannelFactory
  // 用法：--ros-args -p iface:=eth0
  node->declare_parameter<std::string>("iface", "");
  std::string iface = node->get_parameter("iface").as_string();

  if (iface.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Parameter 'iface' is empty. Run with: --ros-args -p iface:=<net_ifname>");
    return 1;
  }

  // 初始化 Unitree SDK ChannelFactory
  ChannelFactory::Instance()->Init(0, iface);

  // ROS2 publisher
  g_pub = node->create_publisher<std_msgs::msg::String>("/voice/prompt", 10);

  // 订阅 Unitree 自身的 ASR 话题
  ChannelSubscriber<std_msgs::msg::dds_::String_> subscriber(AUDIO_SUBSCRIBE_TOPIC);
  subscriber.InitChannel(asr_handler);

  RCLCPP_INFO(node->get_logger(), "g1_asr_bridge ready. Unitree topic=%s -> ROS2 /voice/prompt", AUDIO_SUBSCRIBE_TOPIC);

  // ROS2 spin：保持进程常驻
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

