#pragma once

#include "syt_msgs/msg/cloth_style.hpp"
#include "utils/utils.h"
#include <QColorDialog>
#include <QMap>
#include <QWidget>

namespace Ui {
class InputExtraParamWidget;
}

class InputExtraParamWidget : public QWidget {
  Q_OBJECT

public:
  explicit InputExtraParamWidget(QWidget *parent = nullptr);
  ~InputExtraParamWidget();

  void setClothType(int cloth_type);
  syt_msgs::msg::ClothStyle getClothStyle();

private:
  Ui::InputExtraParamWidget *ui;

private slots:
  void updateColorButton();
};
