#include "string_display.hpp"
#include <rviz_rendering/render_system.hpp>

namespace srs_rviz_plugins
{
StringDisplay::StringDisplay()
{
  text_size_property_ =
    new rviz_common::properties::IntProperty(
    "text size", 12, "text size", this,
    SLOT(updateTextSize()));
  text_size_property_->setMin(0);        hor_alignment_property_ =
    new rviz_common::properties::EnumProperty(
    "hor_alignment", "left", "horizontal alignment of the overlay",
    this, SLOT(updateHorizontalAlignment()));
  hor_alignment_property_->addOption("left", (int)HorizontalAlignment::LEFT);
  hor_alignment_property_->addOption("right", (int)HorizontalAlignment::RIGHT);

  updateHorizontalAlignment();
  updateTextSize();
  current_text_data_.text = "None";
  onEnable();
}

StringDisplay::~StringDisplay()
{
}

void StringDisplay::onEnable()
{
  current_text_data_.visible = true;
}

void StringDisplay::onDisable()
{
  current_text_data_.visible = false;
}

void StringDisplay::onInitialize()
{
  RTDClass::onInitialize();
  rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);

  static int count = 0;
  std::stringstream ss;
  ss << "StringDisplayObject" << count++;
  overlay_text_ptr_ = std::make_shared<OverlayText>(ss.str());
}

void StringDisplay::update(float wall_dt, float ros_dt)
{
  (void)wall_dt;
  (void)ros_dt;
  if (current_text_data_ != last_text_data_) {
    overlay_text_ptr_->update(current_text_data_);
    last_text_data_ = current_text_data_;
  }
}

void StringDisplay::reset()
{
  RTDClass::reset();
  overlay_text_ptr_->update(current_text_data_);
}

void StringDisplay::processMessage(std_msgs::msg::String::ConstSharedPtr msg)
{
  current_text_data_.text = msg->data;
}

void StringDisplay::updateHorizontalAlignment()
{
  current_text_data_.horizontal_alignment =
    HorizontalAlignment{(int)hor_alignment_property_->getOptionInt()};
}

void StringDisplay::updateTextSize()
{
  current_text_data_.height = text_size_property_->getInt();
}

} // namespace rviz_2d_overlay_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(srs_rviz_plugins::StringDisplay, rviz_common::Display)
