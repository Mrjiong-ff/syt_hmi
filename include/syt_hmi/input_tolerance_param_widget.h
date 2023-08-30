#pragma once

#include "syt_msgs/msg/cloth_style.hpp"
#include "utils/utils.h"
#include <QWidget>

namespace Ui {
class InputToleranceParamWidget;
}

class InputToleranceParamWidget : public QWidget {
  Q_OBJECT

public:
  explicit InputToleranceParamWidget(QWidget *parent = nullptr);
  ~InputToleranceParamWidget();

  syt_msgs::msg::ClothStyle getClothStyle();

private:
  Ui::InputToleranceParamWidget *ui;
};
