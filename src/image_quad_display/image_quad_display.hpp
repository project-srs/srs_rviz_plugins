#pragma once

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/vector_property.hpp>
#include <rviz_common/properties/quaternion_property.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace srs_rviz_plugins {

struct QuadProperty {
  float width{0.5f};
  float height{0.5f};
  Ogre::Vector3 offset{Ogre::Vector3::ZERO};
  Ogre::Quaternion rotation{Ogre::Quaternion::IDENTITY};

  bool operator==(const QuadProperty & other)
  {
    return (width == other.width && 
      height == other.height &&
      offset == other.offset &&
      rotation == other.rotation);
  }

  bool operator!=(const QuadProperty & other)
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

class ImageQuadDisplay
    : public rviz_common::MessageFilterDisplay<sensor_msgs::msg::Image> {

public:
  ImageQuadDisplay();
  ~ImageQuadDisplay();

private:
  void processMessage(sensor_msgs::msg::Image::ConstSharedPtr msg) override;
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

  static Ogre::TexturePtr createTexture(const std::string & name, const size_t width, const size_t height);
  static Ogre::MaterialPtr createMaterial(const std::string & name,const std::string & texture_name);
  static int cvTypeFromEncoding(const std::string &encoding);

  Ogre::SceneNode* frame_node_;
  Ogre::ManualObject *image_object_ = nullptr;
  Ogre::TexturePtr texture_;
  Ogre::MaterialPtr material_;

  // properties
  rviz_common::properties::FloatProperty * width_size_property_;
  rviz_common::properties::FloatProperty * height_size_property_;
  rviz_common::properties::VectorProperty * offset_property_;
  rviz_common::properties::QuaternionProperty * rotation_property_; 
  std::optional<QuadProperty> last_property_;

  // msg
  std::optional<sensor_msgs::msg::Image> current_msg_opt_;
  std::optional<sensor_msgs::msg::Image> last_msg_opt_;
  
  // tf pose
  std::optional<OgrePose> last_base_frame_pose_;

  // counter
  int counter_;
};

} // namespace srs_rviz_plugins
