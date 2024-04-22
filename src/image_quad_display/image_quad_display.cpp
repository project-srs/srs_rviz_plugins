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
}

void ImageQuadDisplay::reset()
{
  image_object_->clear();
  MFDClass::reset();
}

void ImageQuadDisplay::processMessage(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(msg->header.frame_id, msg->header.stamp,
                                                  position, orientation)) {
      return;
  }

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);

  frame_node_->setPosition(offset_property_->getVector());
  frame_node_->setOrientation(rotation_property_->getQuaternion());

  bool need_new_texture = (texture_ == nullptr) || (texture_->getWidth() != msg->width || texture_->getHeight() != msg->height);
  if (need_new_texture) {
    printf("new_texture\n");
    texture_ = createTexture("srs_rviz_plugins_image_quad_texture_0", msg->width, msg->height);
    material_ = createMaterial("srs_rviz_plugins_image_quad_material_0", "srs_rviz_plugins_image_quad_texture_0");
  }

  image_object_->clear();
  image_object_->estimateVertexCount(4);
  image_object_->begin(material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_FAN, RESOURCEGROUP_NAME);

  float width = width_size_property_->getFloat();
  float height = height_size_property_->getFloat();

  image_object_->position(-width/2, -height/2, 0);
  image_object_->textureCoord(0, 0);

  // 1
  image_object_->position(width/2, -height/2, 0);
  image_object_->textureCoord(1, 0);

  // 2
  image_object_->position(width/2, height/2, 0);
  image_object_->textureCoord(1, 1);

  // 3
  image_object_->position(-width/2, height/2, 0);
  image_object_->textureCoord(0, 1);
  image_object_->end();

  Ogre::HardwarePixelBufferSharedPtr pixelBuffer = texture_->getBuffer();

  // Lock the pixel buffer and get a pixel box
  pixelBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
  const Ogre::PixelBox &pixelBox = pixelBuffer->getCurrentLock();

  cv::Mat input(msg->height, msg->width, cvTypeFromEncoding(msg->encoding), (void *) msg->data.data(), msg->step);

  cv::Mat textureMat(msg->height, msg->width, CV_8UC4, (void *) pixelBox.data);

  if (sensor_msgs::image_encodings::numChannels(msg->encoding) == 1) {
      cv::cvtColor(input, textureMat, cv::COLOR_GRAY2BGRA, 4);
  } else if (msg->encoding.rfind("rgb", 0) == 0) {
      cv::cvtColor(input, textureMat, cv::COLOR_RGB2BGRA, 4);
  } else if (msg->encoding.rfind("bgr", 0) == 0) {
      cv::cvtColor(input, textureMat, cv::COLOR_BGR2BGRA, 4);
  } else {
      throw std::runtime_error{"Unknown encoding: " + msg->encoding};
  }
  // Unlock the pixel buffer
  pixelBuffer->unlock(); 
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
