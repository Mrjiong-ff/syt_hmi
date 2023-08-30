#include "syt_hmi/input_tolerance_param_widget.h"
#include "ui_input_tolerance_param_widget.h"

InputToleranceParamWidget::InputToleranceParamWidget(QWidget *parent) : QWidget(parent),
                                                                        ui(new Ui::InputToleranceParamWidget) {
  ui->setupUi(this);
}

InputToleranceParamWidget::~InputToleranceParamWidget() {
  delete ui;
}

syt_msgs::msg::ClothStyle InputToleranceParamWidget::getClothStyle() {
  syt_msgs::msg::ClothStyle cloth_style;
  cloth_style.cloth_length_tolerance  = ui->cloth_length_tolerance_spinbox->value();
  cloth_style.bottom_length_tolerance = ui->bottom_length_tolerance_spinbox->value();
  cloth_style.oxter_length_tolerance  = ui->oxter_length_tolerance_spinbox->value();
  cloth_style.matching_level          = style_id_map.value(ui->matching_level_combo_box->currentText());
  return cloth_style;
}
