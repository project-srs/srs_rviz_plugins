#include "string_panel_qt_if.hpp"

#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>

#include <QMouseEvent>
#include <QPainter>
#include <QSizePolicy>

namespace srs_rviz_plugins {
StringPanelQtIf::StringPanelQtIf(QWidget *parent) : rviz_common::Panel(parent) {
  QVBoxLayout *layout = new QVBoxLayout;

  QHBoxLayout *layout_1st = new QHBoxLayout;
  enable_check_ = new QCheckBox("Topic");
  layout_1st->addWidget(enable_check_);
  topic_combo_ = new QComboBox();
  topic_combo_->setEditable(true);
  layout_1st->addWidget(topic_combo_);
  layout->addLayout(layout_1st);

  QHBoxLayout *layout_2nd = new QHBoxLayout;
  content_a_edit_ = new QLineEdit("");
  content_a_button_ = new QPushButton("A");
  layout_2nd->addWidget(content_a_edit_);
  layout_2nd->addWidget(content_a_button_);
  layout->addLayout(layout_2nd);

  QHBoxLayout *layout_3rd = new QHBoxLayout;
  content_b_edit_ = new QLineEdit("");
  content_b_button_ = new QPushButton("B");
  layout_3rd->addWidget(content_b_edit_);
  layout_3rd->addWidget(content_b_button_);
  layout->addLayout(layout_3rd);

  QHBoxLayout *layout_4th = new QHBoxLayout;
  content_c_edit_ = new QLineEdit("");
  content_c_button_ = new QPushButton("C");
  layout_4th->addWidget(content_c_edit_);
  layout_4th->addWidget(content_c_button_);
  layout->addLayout(layout_4th);

  setLayout(layout);

  connect(enable_check_, &QCheckBox::stateChanged, this,
          &StringPanelQtIf::onCheckChange);
  connect(content_a_button_, &QPushButton::clicked, this,
          &StringPanelQtIf::onClickA);
  connect(content_b_button_, &QPushButton::clicked, this,
          &StringPanelQtIf::onClickB);
  connect(content_c_button_, &QPushButton::clicked, this,
          &StringPanelQtIf::onClickC);

  content_a_button_->setEnabled(false);
  content_b_button_->setEnabled(false);
  content_c_button_->setEnabled(false);
}

void StringPanelQtIf::onCheckChange(int state) {
  if (state == Qt::Checked) {
    std::string topic_name = topic_combo_->currentText().toStdString();
    if (topic_name == "") {
      printf("topic name is_empty\n");
      return;
    }

    onStartConnection(topic_name);
    topic_combo_->setEnabled(false);
    content_a_button_->setEnabled(true);
    content_b_button_->setEnabled(true);
    content_c_button_->setEnabled(true);
  } else {
    onEndConnection();
    topic_combo_->setEnabled(true);
    content_a_button_->setEnabled(false);
    content_b_button_->setEnabled(false);
    content_c_button_->setEnabled(false);
  }
}

void StringPanelQtIf::updateTopicList(const std::vector<std::string> topic_list) {
  std::string previous_topic_name = topic_combo_->currentText().toStdString();
  topic_combo_->clear();
  int same_topic_index = -1;
  for (auto t : topic_list)
  {
    topic_combo_->addItem(t.c_str());
    if (t == previous_topic_name)
    {
      same_topic_index = topic_combo_->count() - 1;
    }
  }

  if (previous_topic_name != "")
  {
    if (same_topic_index < 0)
    {
      topic_combo_->addItem(previous_topic_name.c_str());
      same_topic_index = topic_combo_->count() - 1;
    }
    topic_combo_->setCurrentIndex(same_topic_index);
  }
}

void StringPanelQtIf::onClickA() {
  if (!enable_check_->isChecked()) {
    printf("no connection\n");
    return;
  }

  std::string content = content_a_edit_->text().toStdString();
  onCommandMsg(content);
}

void StringPanelQtIf::onClickB() {
  if (!enable_check_->isChecked()) {
    printf("no connection\n");
    return;
  }

  std::string content = content_b_edit_->text().toStdString();
  onCommandMsg(content);
}

void StringPanelQtIf::onClickC() {
  if (!enable_check_->isChecked()) {
    printf("no connection\n");
    return;
  }

  std::string content = content_c_edit_->text().toStdString();
  onCommandMsg(content);
}

void StringPanelQtIf::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
  config.mapSetValue("BaseTopic", topic_combo_->currentText());
  config.mapSetValue("Checked", enable_check_->isChecked());

  config.mapSetValue("ContentA", content_a_edit_->text());
  config.mapSetValue("ContentB", content_b_edit_->text());
  config.mapSetValue("ContentC", content_c_edit_->text());
}

void StringPanelQtIf::load(const rviz_common::Config &config) {
  rviz_common::Panel::load(config);
  QString tmp_text;
  bool tmp_bool;
  if (config.mapGetString("BaseTopic", &tmp_text)) {
    topic_combo_->setCurrentText(tmp_text);
  }
  if (config.mapGetBool("Checked", &tmp_bool)) {
    enable_check_->setChecked(tmp_bool);
  }

  if (config.mapGetString("ContentA", &tmp_text)) {
    content_a_edit_->setText(tmp_text);
  }
  if (config.mapGetString("ContentB", &tmp_text)) {
    content_b_edit_->setText(tmp_text);
  }
  if (config.mapGetString("ContentC", &tmp_text)) {
    content_c_edit_->setText(tmp_text);
  }
}

rclcpp::Node::SharedPtr StringPanelQtIf::getNodePtrFromRviz(void) {
  return this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
}

} // namespace srs_rviz_plugins
