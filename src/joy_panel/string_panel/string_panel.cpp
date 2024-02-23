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

void StringPanel::StartConnection(std::string topic_name)
{
  setNodePtr(getNodePtrFromRviz());
  startRosConnetion(topic_name);
}

void StringPanel::EndConnection(void) {endRosConnetion();}

void StringPanel::CommandFromUi(std::string content)
{
  sendRosCommand(content);
}

} // namespace srs_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(srs_rviz_plugins::StringPanel, rviz_common::Panel)
