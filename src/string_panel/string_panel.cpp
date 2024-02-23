#include "string_panel.hpp"

#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>

#include <QMouseEvent>
#include <QPainter>
#include <QSizePolicy>

namespace srs_rviz_plugins
{
StringPanel::StringPanel(QWidget * parent)
: StringPanelQtIf(parent) {}

void StringPanel::onInitialize()
{
  string_ros_handler_ = std::make_shared<StringRosHandler>(getNodePtrFromRviz());
  updateTopicList(string_ros_handler_->getStringTopicList());
}

void StringPanel::onStartConnection(std::string topic_name)
{
  string_ros_handler_->startRosConnetion(topic_name);
}

void StringPanel::onEndConnection(void)
{
  string_ros_handler_->endRosConnetion();
  updateTopicList(string_ros_handler_->getStringTopicList());
}

void StringPanel::onCommandMsg(std::string content)
{
  string_ros_handler_->sendRosCommand(content);
}

} // namespace srs_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(srs_rviz_plugins::StringPanel, rviz_common::Panel)
