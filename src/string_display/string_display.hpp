#pragma once

#ifndef Q_MOC_RUN
  #include <rviz_common/properties/enum_property.hpp>
  #include <rviz_common/properties/int_property.hpp>
  #include <rviz_common/ros_topic_display.hpp>
#endif

#include <std_msgs/msg/string.hpp>
#include "overlay_text.hpp"

namespace srs_rviz_plugins
{
class StringDisplay : public rviz_common::RosTopicDisplay<std_msgs::msg::String>
{
  Q_OBJECT

public:
  StringDisplay();
  ~StringDisplay();

protected:
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

  rviz_common::properties::EnumProperty * hor_alignment_property_;
  rviz_common::properties::IntProperty * text_size_property_;

protected Q_SLOTS:
  void updateHorizontalAlignment();
  void updateTextSize();

private:
  void processMessage(std_msgs::msg::String::ConstSharedPtr msg) override;

  std::shared_ptr<OverlayText> overlay_text_ptr_{};
  OverlayTextData current_text_data_{};
  OverlayTextData last_text_data_{};
};
} // namespace srs_rviz_plugins
