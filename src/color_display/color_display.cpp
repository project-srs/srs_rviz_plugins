#include "color_display.hpp"
#include <rviz_rendering/render_system.hpp>

namespace srs_rviz_plugins
{
ColorDisplay::ColorDisplay()
{
  radius_property_ = new rviz_common::properties::FloatProperty("Radius", 0.2f, "radius", this);
  tf_frame_property_ = new rviz_common::properties::TfFrameProperty("Base Frame", "map", "base tf frame", this, 0, true);
  offset_property_ = new rviz_common::properties::VectorProperty("offset", Ogre::Vector3::ZERO, "offset_z", this);
}

ColorDisplay::~ColorDisplay()
{
  delete radius_property_;
  delete tf_frame_property_;
  delete offset_property_;

  delete frame_node_;
}

void ColorDisplay::onEnable()
{
}

void ColorDisplay::onDisable()
{
}

void ColorDisplay::onInitialize()
{
  tf_frame_property_->setFrameManager(context_->getFrameManager());

  frame_node_ = scene_node_->createChildSceneNode();
  vis_shape_shere_.reset(new rviz_rendering::Shape(rviz_rendering::Shape::Type::Sphere, scene_manager_, frame_node_));

  Ogre::Vector3 zero_scale{0.0f, 0.0f, 0.0f};
  vis_shape_shere_->setScale(zero_scale); // set invisible

  RTDClass::onInitialize();
}

void ColorDisplay::update(float wall_dt, float ros_dt)
{
  (void)wall_dt;
  (void)ros_dt;

  if(!current_msg_opt_.has_value()){
    // not yet
    setStatus(rviz_common::properties::StatusProperty::Level::Warn, QString("msg"), QString("not received"));
    return;
  }
  setStatus(rviz_common::properties::StatusProperty::Level::Ok, QString("msg"), QString("ok"));
  const bool msg_updated = !last_msg_opt_.has_value() || current_msg_opt_.value() != last_msg_opt_.value();

  SphereProperty current_sphere_property;
  current_sphere_property.radius = radius_property_->	getFloat();
  current_sphere_property.frame_id = tf_frame_property_->getFrameStd();
  current_sphere_property.offset = offset_property_->getVector();
  const bool property_updated = !last_sphere_property_.has_value() || current_sphere_property != last_sphere_property_.value();

  OgrePose current_base_frame_pose;
  if (!context_->getFrameManager()->getTransform(current_sphere_property.frame_id, current_base_frame_pose.position, current_base_frame_pose.orientation)){
    setStatus(rviz_common::properties::StatusProperty::Level::Warn, QString("tf"), QString("not found"));
    return;
  }
  setStatus(rviz_common::properties::StatusProperty::Level::Ok, QString("tf"), QString("OK"));
  const bool pose_updated = !last_base_frame_pose_.has_value() || current_base_frame_pose != last_base_frame_pose_.value();

  if (msg_updated || property_updated || pose_updated) {
    frame_node_->setPosition(current_base_frame_pose.position);
    frame_node_->setOrientation(current_base_frame_pose.orientation);

    const auto color = current_msg_opt_.value();
    vis_shape_shere_->setColor(color.r, color.g, color.b, color.a);

    Ogre::Vector3 radius_scale{current_sphere_property.radius, current_sphere_property.radius, current_sphere_property.radius};
    vis_shape_shere_->setScale(radius_scale); // set invisible
    vis_shape_shere_->setPosition(current_sphere_property.offset);

    last_msg_opt_ = current_msg_opt_;
    last_sphere_property_ = current_sphere_property;
    last_base_frame_pose_ = current_base_frame_pose;
  }
}

void ColorDisplay::reset()
{
  RTDClass::reset();
}

void ColorDisplay::processMessage(std_msgs::msg::ColorRGBA::ConstSharedPtr msg)
{
  current_msg_opt_ = *msg; 
}

} // namespace srs_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(srs_rviz_plugins::ColorDisplay, rviz_common::Display)
