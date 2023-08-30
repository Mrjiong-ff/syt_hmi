#pragma once

#include "syt_msgs/msg/cloth_style.hpp"
#include "utils/utils.h"

#include <QColorDialog>
#include <QWidget>

namespace Ui {
class StyleDisplayWidget;
}

class StyleDisplayWidget : public QWidget {
  Q_OBJECT

public:
  explicit StyleDisplayWidget(QWidget *parent = nullptr);
  ~StyleDisplayWidget();

  void setClothStyle(syt_msgs::msg::ClothStyle cloth_style);
  syt_msgs::msg::ClothStyle getClothStyle();

private:
  Ui::StyleDisplayWidget *ui;

private slots:
  void updateColorButton();
};
