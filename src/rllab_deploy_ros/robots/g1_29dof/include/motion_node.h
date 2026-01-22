// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <unordered_map>
#include <string>

class MotionNode : public rclcpp::Node
{
public:
    MotionNode();

private:
    void on_message(const std_msgs::msg::String::SharedPtr msg);
    static std::string normalize_command(const std::string& input);
    static std::string normalize_key(const std::string& input);
    void load_command_map();

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    std::unordered_map<std::string, std::string> command_map_;
};
