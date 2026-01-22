#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/g1/audio/g1_audio_client.hpp>

using namespace unitree::robot;
using namespace unitree::robot::g1;

static AudioClient* g_client = nullptr;

class TtsBridgeNode : public rclcpp::Node {
public:
  TtsBridgeNode(const std::string& iface) : Node("g1_tts_bridge") {
    if (iface.empty()) {
      throw std::runtime_error("iface is empty");
    }

    RCLCPP_INFO(this->get_logger(), "Initializing with iface: %s", iface.c_str());
    ChannelFactory::Instance()->Init(0, iface);

    g_client = new AudioClient();
    g_client->Init();
    g_client->SetTimeout(10.0f);
    g_client->SetVolume(100);

    sub_ = this->create_subscription<std_msgs::msg::String>(
      "/robot/say", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        const std::string& text = msg->data;
        if (text.empty()) return;

        std::cout << "[TTS] " << text << std::endl;
        if (g_client) g_client->TtsMaker(text, 0);
      }
    );

    RCLCPP_INFO(this->get_logger(), "g1_tts_bridge ready. ROS2 /robot/say -> AudioClient::TtsMaker");
  }

  ~TtsBridgeNode() override {
    if (g_client) {
      delete g_client;
      g_client = nullptr;
    }
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  std::string iface;
  {
    // 使用代码块确保临时节点在创建 TtsBridgeNode 之前被销毁
    auto tmp = std::make_shared<rclcpp::Node>("g1_tts_bridge_args");
    tmp->declare_parameter<std::string>("iface", "");
    iface = tmp->get_parameter("iface").as_string();
  }
  // tmp 节点在这里已经被销毁

  if (iface.empty()) {
    std::cerr << "Parameter 'iface' is empty. Run with: --ros-args -p iface:=<net_ifname>\n";
    return 1;
  }

  try {
    auto node = std::make_shared<TtsBridgeNode>(iface);
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    std::cerr << "g1_tts_bridge failed: " << e.what() << "\n";
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}

