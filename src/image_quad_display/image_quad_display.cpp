#include "image_quad_display.hpp"
#include <rviz_rendering/material_manager.hpp>
#include <OgreTextureManager.h>
#include <OgreTechnique.h>
#include <OgreHardwarePixelBuffer.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.hpp>

constexpr auto RESOURCEGROUP_NAME = "rviz_rendering";

namespace srs_rviz_plugins
{
ImageQuadDisplay::ImageQuadDisplay()
{
  width_size_property_ = new rviz_common::properties::FloatProperty("width", 0.2f, "quad width", this);
  height_size_property_ = new rviz_common::properties::FloatProperty("height", 0.2f, "quad height", this);
  offset_property_ = new rviz_common::properties::VectorProperty("offset", Ogre::Vector3::ZERO, "offset", this);
  rotation_property_ = new rviz_common::properties::QuaternionProperty("rotation", Ogre::Quaternion::IDENTITY, "rotation", this);
}

ImageQuadDisplay::~ImageQuadDisplay()
{
  scene_manager_->destroyManualObject(image_object_);
}

void ImageQuadDisplay::onEnable()
{
}

void ImageQuadDisplay::onDisable()
{
}

void ImageQuadDisplay::onInitialize()
{
  frame_node_ = scene_node_->createChildSceneNode();

  image_object_ = scene_manager_->createManualObject();
  image_object_->setDynamic(true);
  frame_node_->attachObject(image_object_);
  MFDClass::onInitialize();
}

void ImageQuadDisplay::update(float wall_dt, float ros_dt)
{
  (void)wall_dt;
  (void)ros_dt;

  // check msg
  if(!current_msg_opt_.has_value()){
    // not yet
    setStatus(rviz_common::properties::StatusProperty::Level::Warn, QString("msg"), QString("not received"));
    return;
  }
  setStatus(rviz_common::properties::StatusProperty::Level::Ok, QString("msg"), QString("ok"));

  // update texture & material
  bool texture_size_updated = (texture_ == nullptr) || (texture_->getWidth() != current_msg_opt_.value().width || texture_->getHeight() != current_msg_opt_.value().height);
  if (texture_size_updated) {
    printf("new texture\n");
    texture_ = createTexture("srs_rviz_plugins_image_quad_texture_0", current_msg_opt_.value().width, current_msg_opt_.value().height);

    if (material_ == nullptr) {
      printf("new material\n");
      material_ = createMaterial("srs_rviz_plugins_image_quad_material_0", texture_->getName());
    }
  }

  bool texture_content_updated = !last_msg_opt_.has_value() || last_msg_opt_.value().data != current_msg_opt_.value().data;
  if(texture_content_updated) {
    Ogre::HardwarePixelBufferSharedPtr pixelBuffer = texture_->getBuffer();

    // Lock the pixel buffer and get a pixel box
    pixelBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
    const Ogre::PixelBox &pixelBox = pixelBuffer->getCurrentLock();

    cv::Mat input(current_msg_opt_.value().height, current_msg_opt_.value().width, cvTypeFromEncoding(current_msg_opt_.value().encoding), (void *) current_msg_opt_.value().data.data(), current_msg_opt_.value().step);

    cv::Mat textureMat(current_msg_opt_.value().height, current_msg_opt_.value().width, CV_8UC4, (void *) pixelBox.data);

    if (sensor_msgs::image_encodings::numChannels(current_msg_opt_.value().encoding) == 1) {
        cv::cvtColor(input, textureMat, cv::COLOR_GRAY2BGRA, 4);
    } else if (current_msg_opt_.value().encoding.rfind("rgb", 0) == 0) {
        cv::cvtColor(input, textureMat, cv::COLOR_RGB2BGRA, 4);
    } else if (current_msg_opt_.value().encoding.rfind("bgr", 0) == 0) {
        cv::cvtColor(input, textureMat, cv::COLOR_BGR2BGRA, 4);
    } else {
        throw std::runtime_error{"Unknown encoding: " + current_msg_opt_.value().encoding};
    }
    // Unlock the pixel buffer
    pixelBuffer->unlock();
  }

  // check property
  QuadProperty current_property;
  current_property.width = width_size_property_->getFloat();
  current_property.height = height_size_property_->getFloat();
  current_property.offset = offset_property_->getVector();
  current_property.rotation = rotation_property_->getQuaternion();
  const bool property_updated = !last_property_.has_value() || current_property != last_property_.value();

  // check tf
  OgrePose current_base_frame_pose;
  if (!context_->getFrameManager()->getTransform(current_msg_opt_.value().header.frame_id, current_base_frame_pose.position, current_base_frame_pose.orientation)){
    setStatus(rviz_common::properties::StatusProperty::Level::Warn, QString("tf"), QString("not found"));
    return;
  }
  setStatus(rviz_common::properties::StatusProperty::Level::Ok, QString("tf"), QString("OK"));
  const bool pose_updated = !last_base_frame_pose_.has_value() || current_base_frame_pose != last_base_frame_pose_.value();

  if (property_updated || pose_updated) {
    printf("update quad\n");

    scene_node_->setPosition(current_base_frame_pose.position);
    scene_node_->setOrientation(current_base_frame_pose.orientation);

    frame_node_->setPosition(current_property.offset);
    frame_node_->setOrientation(current_property.rotation);

    image_object_->clear();
    image_object_->estimateVertexCount(4);
    image_object_->begin(material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_FAN, RESOURCEGROUP_NAME);

    image_object_->position(-current_property.width/2, -current_property.height/2, 0);
    image_object_->textureCoord(0, 0);

    // 1
    image_object_->position(current_property.width/2, -current_property.height/2, 0);
    image_object_->textureCoord(1, 0);

    // 2
    image_object_->position(current_property.width/2, current_property.height/2, 0);
    image_object_->textureCoord(1, 1);

    // 3
    image_object_->position(-current_property.width/2, current_property.height/2, 0);
    image_object_->textureCoord(0, 1);
    image_object_->end();
  }

  last_property_ = current_property;
  last_msg_opt_ = current_msg_opt_;
  last_base_frame_pose_ = current_base_frame_pose;
}

void ImageQuadDisplay::reset()
{
  image_object_->clear();
  MFDClass::reset();
}

void ImageQuadDisplay::processMessage(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  current_msg_opt_ = *msg;
}

Ogre::TexturePtr ImageQuadDisplay::createTexture(const std::string & name, const size_t width, const size_t height)
{
  Ogre::TexturePtr texture = Ogre::TextureManager::getSingleton().createManual(
    name,       // name
    RESOURCEGROUP_NAME,
    Ogre::TEX_TYPE_2D,       // type
    width, height,           // width & height of the render window
    0,                       // number of mipmaps
    Ogre::PF_A8R8G8B8,       // pixel format chosen to match a format Qt can use
    Ogre::TU_DEFAULT         // usage
  );
  return texture;
}

Ogre::MaterialPtr ImageQuadDisplay::createMaterial(const std::string & name,const std::string & texture_name)
{
  Ogre::MaterialPtr material = rviz_rendering::MaterialManager::createMaterialWithNoLighting(name);
  auto rpass = material->getTechniques()[0]->getPasses()[0];
  rpass->createTextureUnitState(texture_name);
  rpass->setCullingMode(Ogre::CULL_NONE);
  rpass->setEmissive(Ogre::ColourValue::White);
  rpass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  return material;
}

int ImageQuadDisplay::cvTypeFromEncoding(const std::string &encoding) {
    using namespace sensor_msgs::image_encodings;
    if (encoding.rfind("32FC", 0) == 0) {
        return CV_32FC(numChannels(encoding));
    }
    if (bitDepth(encoding) == 8) {
        // This case covers all bgr/rgb/bayer 8bit cases. OpenCV does not differentiate between them.
        return CV_8UC(numChannels(encoding));
    }

    throw std::invalid_argument{"OpenCV Type for encoding could not be found"};
}

} // namespace srs_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(srs_rviz_plugins::ImageQuadDisplay, rviz_common::Display)
