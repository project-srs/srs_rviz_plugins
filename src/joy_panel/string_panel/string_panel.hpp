#pragma once

#include "string_panel_qt_if.hpp"
#include "string_panel_ros_if.hpp"

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

class StringPanel : public StringPanelQtIf, public StringPanelRosIf
{
  Q_OBJECT

public:
  StringPanel(QWidget * parent = nullptr);

  void StartConnection(std::string topic_name) override;

  void EndConnection(void) override;

  void CommandFromUi(std::string content) override;
};

} // namespace srs_rviz_plugins
