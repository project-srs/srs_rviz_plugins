#pragma once

#ifndef Q_MOC_RUN
#include <QtWidgets>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#endif
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace srs_rviz_plugins
{

class StringPanelQtIf : public rviz_common::Panel
{
  Q_OBJECT

public:
  StringPanelQtIf(QWidget * parent = nullptr);

  // void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

  rclcpp::Node::SharedPtr getNodePtrFromRviz(void);
  void updateTopicList(const std::vector<std::string> topic_list);

public Q_SLOTS:
  void onCheckChange(int state);
  void onClickA();
  void onClickB();
  void onClickC();

protected:
  virtual void onStartConnection(std::string topic_name) = 0;
  virtual void onEndConnection(void) = 0;
  virtual void onCommandMsg(std::string content) = 0;

private:
  QCheckBox * enable_check_;
  QComboBox * topic_combo_;

  QLineEdit * content_a_edit_;
  QLineEdit * content_b_edit_;
  QLineEdit * content_c_edit_;
  QPushButton * content_a_button_;
  QPushButton * content_b_button_;
  QPushButton * content_c_button_;
};

} // namespace srs_rviz_plugins
