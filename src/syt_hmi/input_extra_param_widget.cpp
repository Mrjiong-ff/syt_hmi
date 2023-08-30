#include "syt_hmi/input_extra_param_widget.h"
#include "ui_input_extra_param_widget.h"

InputExtraParamWidget::InputExtraParamWidget(QWidget *parent) : QWidget(parent),
                                                                ui(new Ui::InputExtraParamWidget) {
  ui->setupUi(this);
  ui->cloth_type_combo_box->setDisabled(true);

  updateColorButton();

  void (QSpinBox::*spin_box_signal)(int)             = &QSpinBox::valueChanged;
  void (InputExtraParamWidget::*update_color_slot)() = &InputExtraParamWidget::updateColorButton;
  connect(ui->red_spin_box, spin_box_signal, this, update_color_slot);
  connect(ui->green_spin_box, spin_box_signal, this, update_color_slot);
  connect(ui->blue_spin_box, spin_box_signal, this, update_color_slot);

  connect(ui->cloth_color_btn, &QPushButton::clicked, this, [=]() {
    QColor color = QColorDialog::getColor(QColor(255, 0, 0));
    ui->cloth_color_btn->setStyleSheet(QString("background-color: rgb(%1, %2, %3);").arg(color.red()).arg(color.green()).arg(color.blue()));
    ui->red_spin_box->setValue(color.red());
    ui->green_spin_box->setValue(color.green());
    ui->blue_spin_box->setValue(color.blue());
  });
}

InputExtraParamWidget::~InputExtraParamWidget() {
  delete ui;
}

void InputExtraParamWidget::setClothType(int cloth_type) {
  ui->cloth_type_combo_box->setCurrentText(cloth_type ? "后片" : "前片");
}

syt_msgs::msg::ClothStyle InputExtraParamWidget::getClothStyle() {
  syt_msgs::msg::ClothStyle cloth_style;
  cloth_style.elasticity_level = style_id_map.value(ui->cloth_elesticity_combo_box->currentText());
  cloth_style.thickness_level  = style_id_map.value(ui->cloth_thickness_combo_box->currentText());
  cloth_style.cloth_type       = style_id_map.value(ui->cloth_type_combo_box->currentText());
  cloth_style.have_printings   = ui->have_printings_check_box->isChecked();
  cloth_style.cloth_color      = (uint32_t)ui->red_spin_box->value() << 16 | (uint32_t)ui->green_spin_box->value() << 8 | (uint32_t)ui->blue_spin_box->value();
  cloth_style.cloth_size       = style_id_map.value(ui->cloth_size_combo_box->currentText());
  cloth_style.glossiness_level = style_id_map.value(ui->cloth_glossiness_combo_box->currentText());
  cloth_style.cloth_weight     = ui->cloth_weight_spin_box->value();
  return cloth_style;
}

void InputExtraParamWidget::updateColorButton() {
  ui->cloth_color_btn->setStyleSheet(QString("background-color: rgb(%1, %2, %3);").arg(ui->red_spin_box->value()).arg(ui->green_spin_box->value()).arg(ui->blue_spin_box->value()));
}
