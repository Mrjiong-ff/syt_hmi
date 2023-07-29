#include "syt_hmi/input_length_param_widget.h"
#include "ui_input_length_param_widget.h"

InputLengthParamWidget::InputLengthParamWidget(QWidget *parent) : QWidget(parent),
                                                                  ui(new Ui::InputLengthParamWidget) {
  ui->setupUi(this);

  QRegExp exp("^(?!0\\d\\d?\\d?$)\\d{1,3}(?:\\.\\d{1,3})?|1000(?:\\.00?)?$");
  QValidator *validator = new QRegExpValidator(exp);
  ui->cloth_length_line_edit->setValidator(validator);
  ui->bottom_length_line_edit->setValidator(validator);
  ui->oxter_length_line_edit->setValidator(validator);
  ui->shoulder_length_line_edit->setValidator(validator);
  ui->side_length_line_edit->setValidator(validator);
}

InputLengthParamWidget::~InputLengthParamWidget() {
  delete ui;
}

bool InputLengthParamWidget::filled() {
  if (ui->cloth_length_line_edit->text().isEmpty()) {
    return false;
  }
  if (ui->bottom_length_line_edit->text().isEmpty()) {
    return false;
  }
  if (ui->oxter_length_line_edit->text().isEmpty()) {
    return false;
  }
  if (ui->shoulder_length_line_edit->text().isEmpty()) {
    return false;
  }
  if (ui->side_length_line_edit->text().isEmpty()) {
    return false;
  }
  return true;
}

syt_msgs::msg::ClothStyle InputLengthParamWidget::getClothStyle() {
  syt_msgs::msg::ClothStyle cloth_style;
  cloth_style.cloth_length    = ui->cloth_length_line_edit->text().toFloat();
  cloth_style.bottom_length   = ui->bottom_length_line_edit->text().toFloat();
  cloth_style.oxter_length    = ui->oxter_length_line_edit->text().toFloat();
  cloth_style.shoulder_length = ui->shoulder_length_line_edit->text().toFloat();
  cloth_style.side_length     = ui->side_length_line_edit->text().toFloat();
  return cloth_style;
}
