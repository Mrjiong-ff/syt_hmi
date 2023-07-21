#pragma once

#include "syt_msgs/msg/cloth_style.hpp"
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

  QMap<QString, int> elasticity_map_;
  QMap<QString, int> type_map_;
  QMap<QString, int> thickness_map_;
  QMap<QString, int> size_map_;
  QMap<QString, int> glossiness_map_;

private slots:
  void updateColorButton();
};
