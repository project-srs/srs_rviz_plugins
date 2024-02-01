#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace srs_rviz_plugins {

class StringRosHandler {
public:
  StringRosHandler(const rclcpp::Node::SharedPtr& node) {
    node_ = node;
  }

  void startRosConnetion(const std::string& topic_name) {
    string_publisher_ =
      node_->create_publisher<std_msgs::msg::String>(topic_name, rclcpp::QoS(10));
  }

  void endRosConnetion(void) {
    string_publisher_.reset();
  }

  void sendRosCommand(const std::string& content) {
    if (string_publisher_) {
      std_msgs::msg::String msg;
      msg.data = content;
      string_publisher_->publish(msg);
    }
  }

  std::vector<std::string> getStringTopicList(void) const {
    return getTopicList("std_msgs/msg/String");
  }

private:
  std::vector<std::string> getTopicList(const std::string type_name) const {
    std::map<std::string, std::vector<std::string>> topic_map = node_->get_topic_names_and_types();

    std::vector<std::string> output;
    for (auto pair : topic_map) {
      for (auto s : pair.second) {
        if (s == type_name) {
          output.push_back(pair.first);
          break;
        }
      }
    }
    return output;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_publisher_;
};

} // namespace srs_rviz_plugins
