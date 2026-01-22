// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#include "motion_node.h"

#include <algorithm>
#include <cctype>

#include <yaml-cpp/yaml.h>

#include "FSM/MotionCommandRouter.h"
#include "param.h"

namespace
{
std::string trim_copy(const std::string& input)
{
    size_t start = 0;
    while (start < input.size() && std::isspace(static_cast<unsigned char>(input[start]))) {
        ++start;
    }
    size_t end = input.size();
    while (end > start && std::isspace(static_cast<unsigned char>(input[end - 1]))) {
        --end;
    }
    return input.substr(start, end - start);
}

bool starts_with(const std::string& text, const std::string& prefix)
{
    return text.rfind(prefix, 0) == 0;
}
} // namespace

MotionNode::MotionNode() : Node("motion_node")
{
    load_command_map();
    sub_ = create_subscription<std_msgs::msg::String>(
        "/robot/motion_code",
        10,
        std::bind(&MotionNode::on_message, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "motion_node subscribed to /robot/motion_code");
}

void MotionNode::on_message(const std_msgs::msg::String::SharedPtr msg)
{
    const std::string raw_cmd = normalize_command(msg->data);
    if (raw_cmd.empty()) {
        return;
    }

    const std::string key = normalize_key(raw_cmd);
    auto it = command_map_.find(key);
    if (it != command_map_.end()) {
        MotionCommandRouter::RequestStateByName(it->second);
        RCLCPP_INFO(get_logger(), "motion_code '%s' -> %s", raw_cmd.c_str(), it->second.c_str());
        return;
    }

    // Try with original case first (for state names)
    if (MotionCommandRouter::RequestStateByName(msg->data)) {
        RCLCPP_INFO(get_logger(), "motion_code '%s' -> %s", msg->data.c_str(), msg->data.c_str());
        return;
    }

    // Fallback to lowercase version
    if (MotionCommandRouter::RequestStateByName(raw_cmd)) {
        RCLCPP_INFO(get_logger(), "motion_code '%s' -> %s", raw_cmd.c_str(), raw_cmd.c_str());
        return;
    }

    if (starts_with(key, "walk")) {
        MotionCommandRouter::RequestStateByName("Velocity");
        RCLCPP_INFO(get_logger(), "motion_code '%s' -> Velocity", raw_cmd.c_str());
        return;
    }
    if (starts_with(key, "stand")) {
        MotionCommandRouter::RequestStateByName("FixStand");
        RCLCPP_INFO(get_logger(), "motion_code '%s' -> FixStand", raw_cmd.c_str());
        return;
    }
    if (starts_with(key, "stop")) {
        MotionCommandRouter::RequestStateByName("Passive");
        RCLCPP_INFO(get_logger(), "motion_code '%s' -> Passive", raw_cmd.c_str());
        return;
    }

    RCLCPP_WARN(get_logger(), "motion_code '%s' not mapped", raw_cmd.c_str());
}

std::string MotionNode::normalize_command(const std::string& input)
{
    std::string out = trim_copy(input);
    if (out.size() >= 2 && out.substr(out.size() - 2) == "()") {
        out = out.substr(0, out.size() - 2);
    }
    std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return out;
}

std::string MotionNode::normalize_key(const std::string& input)
{
    std::string out = input;
    std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return out;
}

void MotionNode::load_command_map()
{
    auto map_node = param::config["MotionCommands"];
    if (!map_node) {
        map_node = param::config["motion_commands"];
    }
    if (!map_node) {
        return;
    }

    try {
        for (auto it = map_node.begin(); it != map_node.end(); ++it) {
            std::string key = normalize_key(it->first.as<std::string>());
            std::string value = it->second.as<std::string>();
            command_map_[key] = value;
        }
        RCLCPP_INFO(get_logger(), "motion_node loaded %zu command mappings", command_map_.size());
    } catch (const YAML::Exception& e) {
        RCLCPP_WARN(get_logger(), "motion_node failed to load MotionCommands: %s", e.what());
    }
}
