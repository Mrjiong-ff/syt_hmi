#pragma once

#include "syt_msgs/msg/cloth_style.hpp"
#include <QRegExpValidator>
#include <QWidget>

namespace Ui {
class InputLengthParamWidget;
}

class InputLengthParamWidget : public QWidget
{
  Q_OBJECT

public:
  explicit InputLengthParamWidget(QWidget *parent = nullptr);
  ~InputLengthParamWidget();

  bool filled();
  syt_msgs::msg::ClothStyle getClothStyle();

private:
  Ui::InputLengthParamWidget *ui;
};

