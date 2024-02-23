#pragma once

#include "string_panel_qt_if.hpp"
#include "string_ros_handler.hpp"

#ifndef Q_MOC_RUN
#include <QtWidgets>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#endif
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace srs_rviz_plugins
{

class StringPanel : public StringPanelQtIf
{
  Q_OBJECT

public:
  StringPanel(QWidget * parent = nullptr);
  // void updateTopicList(void);

  // from Qt
  void onInitialize() override;
  void onStartConnection(std::string topic_name);
  void onEndConnection(void);
  void onCommandMsg(std::string content);

private:
  std::shared_ptr<StringRosHandler> string_ros_handler_;
};

} // namespace srs_rviz_plugins
