#include "string_panel_ros_if.hpp"

namespace srs_rviz_plugins {
StringPanelRosIf::StringPanelRosIf(void) {}

void StringPanelRosIf::setNodePtr(rclcpp::Node::SharedPtr nh) { nh_ = nh; }

void StringPanelRosIf::startRosConnetion(std::string topic_name) {
  string_publisher_ =
      nh_->create_publisher<std_msgs::msg::String>(topic_name, rclcpp::QoS(10));
}

void StringPanelRosIf::endRosConnetion(void) { string_publisher_.reset(); }

void StringPanelRosIf::sendRosCommand(std::string content) {
  std_msgs::msg::String msg;
  msg.data = content;
  string_publisher_->publish(msg);
}

} // namespace srs_rviz_plugins