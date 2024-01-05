#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace srs_rviz_plugins {

class StringPanelRosIf {
public:
  StringPanelRosIf(void);
  void setNodePtr(rclcpp::Node::SharedPtr nh);
  void startRosConnetion(std::string topic_name);
  void endRosConnetion(void);
  void sendRosCommand(std::string content);

private:
  rclcpp::Node::SharedPtr nh_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_publisher_;
};

} // namespace srs_rviz_plugins
