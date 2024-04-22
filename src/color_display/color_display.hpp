#pragma once

#ifndef Q_MOC_RUN
  #include <rviz_common/properties/float_property.hpp>
  #include <rviz_common/properties/tf_frame_property.hpp>
  #include <rviz_common/properties/vector_property.hpp>
  #include <rviz_common/ros_topic_display.hpp>
#endif

#include <std_msgs/msg/color_rgba.hpp>
#include <rviz_rendering/objects/shape.hpp>

namespace srs_rviz_plugins
{

struct SphereProperty {
  bool enable{false};
  float radius{0.2f};
  std::string frame_id{"map"};
  Ogre::Vector3 offset{};

  bool operator==(const SphereProperty & other)
  {
    return (enable == other.enable && 
      radius == other.radius &&
      frame_id == other.frame_id &&
      offset == other.offset);
  }

  bool operator!=(const SphereProperty & other)
  {
    return !(*this == other);
  }
};

struct OgrePose {
  Ogre::Vector3 position{};
  Ogre::Quaternion orientation{};

  bool operator==(const OgrePose & other)
  {
    return position == other.position && orientation == other.orientation;
  }

  bool operator!=(const OgrePose & other)
  {
    return !(*this == other);
  }
};

class ColorDisplay : public rviz_common::RosTopicDisplay<std_msgs::msg::ColorRGBA>
{
  Q_OBJECT

public:
  ColorDisplay();
  ~ColorDisplay();

protected:
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

  rviz_common::properties::FloatProperty * radius_property_;
  rviz_common::properties::TfFrameProperty* tf_frame_property_;
  rviz_common::properties::VectorProperty * offset_property_;

private:
  void processMessage(std_msgs::msg::ColorRGBA::ConstSharedPtr msg) override;

  Ogre::SceneNode* frame_node_;
  std::shared_ptr<rviz_rendering::Shape> vis_shape_shere_;

  std::optional<std_msgs::msg::ColorRGBA> current_msg_opt_{std::nullopt};
  std::optional<std_msgs::msg::ColorRGBA> last_msg_opt_{std::nullopt};
  std::optional<SphereProperty> last_sphere_property_{std::nullopt};
  std::optional<OgrePose> last_base_frame_pose_{std::nullopt};
};
} // namespace srs_rviz_plugins
