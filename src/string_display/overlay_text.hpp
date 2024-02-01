#pragma once

#ifndef Q_MOC_RUN
  #include <OgreColourValue.h>
  #include <OgreMaterial.h>
  #include <OgreHardwarePixelBuffer.h>
  #include <OgreMaterialManager.h>
  #include <OgreTexture.h>

  #include <OgreHardwarePixelBuffer.h>
  #include <OgreMaterialManager.h>
  #include <OgreTechnique.h>
  #include <OgreTexture.h>
  #include <OgreTextureManager.h>
  #include <Overlay/OgreOverlayProfileSessionListener.h>
  #include <Overlay/OgreOverlay.h>
  #include <Overlay/OgreOverlayContainer.h>
  #include <Overlay/OgreOverlayElement.h>
  #include <Overlay/OgreOverlayManager.h>
  #include <Overlay/OgrePanelOverlayElement.h>
  #include <Overlay/OgreTextAreaOverlayElement.h>
  #include <Overlay/OgreFontManager.h>

  #include <QColor>
  #include <QImage>
  #include <memory>
  #include <string>

  #include <rviz_common/properties/bool_property.hpp>
  #include <rviz_common/properties/color_property.hpp>
  #include <rviz_common/properties/enum_property.hpp>
  #include <rviz_common/properties/float_property.hpp>
  #include <rviz_common/properties/int_property.hpp>
  #include <rviz_common/properties/ros_topic_property.hpp>
  #include <rviz_common/ros_topic_display.hpp>
#endif

#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>

namespace srs_rviz_plugins {

enum class HorizontalAlignment {
  LEFT,
  CENTER,
  RIGHT,
};

struct OverlayTextData {
  bool visible{true};
  std::string text{""};
  int height{16};
  HorizontalAlignment horizontal_alignment{HorizontalAlignment::LEFT};

  bool operator==(const OverlayTextData &other) {
    if (visible == other.visible && text == other.text && height == other.height && horizontal_alignment == other.horizontal_alignment) {
      return true;
    }
    return false;
  }

  bool operator!=(const OverlayTextData &other) {
    return !(*this == other);
  }
};

class OverlayText {
public:
  OverlayText(const std::string& name) {
    Ogre::OverlayManager *mOverlayMgr = Ogre::OverlayManager::getSingletonPtr();
    overlay_ = mOverlayMgr->create(name);
    
    texture_ = createTexture(name + "Texture", QColor(200, 200, 200, 100), 100, 100);
    panel_material_ = createMaterial(name + "Material", texture_->getName());

    const int temp_size = 32;
    back_ground_panel_ = createPanel(name + "Panel", 0, 0, temp_size, temp_size, panel_material_->getName());
    overlay_->add2D(back_ground_panel_);

    const std::string temp_text = "TEST";
    text_element_ = createText(name + "TextArea", temp_text, 100, 100, temp_size);
    text_container_ = createContainer(name + "TextPanel", 0, 0, text_element_);
    overlay_->add2D(text_container_);

    overlay_->show();
    OverlayTextData data{};
    update(data);
  }

  ~OverlayText() {
    Ogre::OverlayManager *mOverlayMgr = Ogre::OverlayManager::getSingletonPtr();
      mOverlayMgr->destroyOverlayElement(text_element_);
      mOverlayMgr->destroyOverlayElement(text_container_);

      mOverlayMgr->destroyOverlayElement(back_ground_panel_);
      panel_material_->unload();
      Ogre::MaterialManager::getSingleton().remove(panel_material_->getName());

      mOverlayMgr->destroy(overlay_);      
  }

  void update(const OverlayTextData& data) {
    const float width_rate = getWidthRate(data.text);
    const float panel_width = data.height * width_rate;

    Ogre::OverlayManager *mOverlayMgr = Ogre::OverlayManager::getSingletonPtr();
    float screen_width = mOverlayMgr->getViewportWidth();

    text_element_->setCaption(data.text);
    text_element_->setCharHeight(data.height);

    if (data.horizontal_alignment == HorizontalAlignment::LEFT) {
      back_ground_panel_->setPosition(0, 0);
      text_element_->setPosition(0, 1);
    } else { // right
      back_ground_panel_->setPosition(screen_width - panel_width, 0);
      text_element_->setPosition(screen_width - panel_width, 1);
    }
    back_ground_panel_->setDimensions(panel_width, data.height);

    if (data.visible && !data.text.empty()) {
      overlay_->show();
    } else {
      overlay_->hide();
    }
  }

private:
  static Ogre::TexturePtr createTexture(const std::string& name, const QColor& qcolor, const size_t width, const size_t height) {
    Ogre::TexturePtr texture = Ogre::TextureManager::getSingleton().createManual(
            name, // name
            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
            Ogre::TEX_TYPE_2D, // type
            width, height,     // width & height of the render window
            0,                 // number of mipmaps
            Ogre::PF_A8R8G8B8, // pixel format chosen to match a format Qt can use
            Ogre::TU_DEFAULT   // usage
    );

    Ogre::HardwarePixelBufferSharedPtr pixel_buffer = texture->getBuffer();
    pixel_buffer->lock(Ogre::HardwareBuffer::HBL_NORMAL);
    const Ogre::PixelBox &pixelBox = pixel_buffer->getCurrentLock();

    Ogre::uint8 *pDest = static_cast<Ogre::uint8 *>(pixelBox.data);
    memset(pDest, 0, width * height);
    QImage Hud = QImage(pDest, width, height, QImage::Format_ARGB32);

    for (unsigned int i = 0; i < width; i++) {
      for (unsigned int j = 0; j < height; j++) {
        Hud.setPixel(i, j, qcolor.rgba());
      }
    }
    pixel_buffer->unlock();
    return texture;
  }

  static Ogre::MaterialPtr createMaterial(const std::string& name, const std::string& texture_name) {
    Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
      name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material->getTechnique(0)->getPass(0)->createTextureUnitState(texture_name);
    material->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    return material;
  }
  
  static Ogre::PanelOverlayElement* createPanel(const std::string& base_name, const int x, const int y, const int width, const int height, const std::string& material_name) {
    Ogre::OverlayManager *mOverlayMgr = Ogre::OverlayManager::getSingletonPtr();
    Ogre::PanelOverlayElement* panel = static_cast<Ogre::PanelOverlayElement *>(mOverlayMgr->createOverlayElement("Panel", base_name + "Panel"));
    panel->setMetricsMode(Ogre::GMM_PIXELS);
    panel->setMaterialName(material_name);
    panel->setPosition(x, y);
    panel->setDimensions(width, height);
    return panel;
  }

  static Ogre::TextAreaOverlayElement* createText(const std::string& name, const std::string content, const int x, const int y, const int height) {
    Ogre::OverlayManager *mOverlayMgr = Ogre::OverlayManager::getSingletonPtr();
    Ogre::TextAreaOverlayElement *text = (Ogre::TextAreaOverlayElement*)(mOverlayMgr->createOverlayElement("TextArea", name));
    text->setMetricsMode(Ogre::GuiMetricsMode::GMM_PIXELS);
    text->setVerticalAlignment(Ogre::GuiVerticalAlignment::GVA_TOP);
    text->setHorizontalAlignment(Ogre::GuiHorizontalAlignment::GHA_LEFT);
    text->setColour(Ogre::ColourValue::White);
    text->setFontName(font_name_);
    text->setCaption(content);
    text->setCharHeight(height);
    text->setPosition(x, y);
    return text;
  }

  static Ogre::OverlayContainer* createContainer(const std::string name, const int x, const int y, Ogre::TextAreaOverlayElement* text) {
    Ogre::OverlayContainer* panel =
        (Ogre::OverlayContainer*)Ogre::OverlayManager::getSingleton().createOverlayElement("Panel", name);
    panel->addChild(text);
    panel->setPosition(x, y);
    return panel;
  }

  static float getWidthRate(const std::string text) {
    Ogre::FontPtr font = Ogre::FontManager::getSingleton().getByName(font_name_, "rviz_rendering");
    float width_rate = 0;
    for (auto c : text) {
      float ratio = 1.0f;
      if (c == ' ') {
        ratio = 0.5f;
      } else {
        ratio = font->getGlyphAspectRatio(c);
      }
      width_rate += ratio;
    }
    return width_rate;
  }

  Ogre::Overlay *overlay_;
  Ogre::PanelOverlayElement * back_ground_panel_;
  Ogre::MaterialPtr panel_material_;
  Ogre::TexturePtr texture_;
  Ogre::OverlayContainer * text_container_;
  Ogre::TextAreaOverlayElement* text_element_;
  static constexpr char font_name_[] = "Liberation Sans";
};

} // namespace srs_rviz_plugins
