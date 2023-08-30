#include "syt_hmi/style_display_widget.h"
#include "ui_style_display_widget.h"

StyleDisplayWidget::StyleDisplayWidget(QWidget *parent) : QWidget(parent),
                                                          ui(new Ui::StyleDisplayWidget) {
  ui->setupUi(this);
  ui->cloth_type_combo_box->setEnabled(false);

  void (QSpinBox::*spin_box_signal)(int)             = &QSpinBox::valueChanged;
  void (StyleDisplayWidget::*update_color_slot)() = &StyleDisplayWidget::updateColorButton;
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

StyleDisplayWidget::~StyleDisplayWidget() {
  delete ui;
}

void StyleDisplayWidget::setClothStyle(syt_msgs::msg::ClothStyle cloth_style) {
  ui->red_spin_box->setValue((cloth_style.cloth_color & 0xff0000) >> 16);
  ui->green_spin_box->setValue((cloth_style.cloth_color & 0xff00) >> 8);
  ui->blue_spin_box->setValue((cloth_style.cloth_color & 0xff));
  ui->cloth_color_btn->setStyleSheet(QString("background-color: rgb(%1, %2, %3);").arg(ui->red_spin_box->value()).arg(ui->green_spin_box->value()).arg(ui->blue_spin_box->value()));
  ui->cloth_type_combo_box->setCurrentText(id_style_map[cloth_style.cloth_type]);
  ui->cloth_size_combo_box->setCurrentText(id_style_map[cloth_style.cloth_size]);
  ui->cloth_elesticity_combo_box->setCurrentText(id_style_map[cloth_style.elasticity_level]);
  ui->cloth_thickness_combo_box->setCurrentText(id_style_map[cloth_style.thickness_level]);
  ui->cloth_glossiness_combo_box->setCurrentText(id_style_map[cloth_style.glossiness_level]);
  ui->cloth_weight_spin_box->setValue(cloth_style.cloth_weight);
  ui->cloth_length_line_edit->setText(QString::number(cloth_style.cloth_length));
  ui->bottom_length_line_edit->setText(QString::number(cloth_style.bottom_length));
  ui->oxter_length_line_edit->setText(QString::number(cloth_style.oxter_length));
  ui->shoulder_length_line_edit->setText(QString::number(cloth_style.shoulder_length));
  ui->side_length_line_edit->setText(QString::number(cloth_style.side_length));
  ui->cloth_length_tolerance_spinbox->setValue(cloth_style.cloth_length_tolerance);
  ui->bottom_length_tolerance_spinbox->setValue(cloth_style.bottom_length_tolerance);
  ui->oxter_length_tolerance_spinbox->setValue(cloth_style.oxter_length_tolerance);
  ui->matching_level_combo_box->setCurrentText(id_style_map[cloth_style.matching_level]);
  ui->have_printings_check_box->setChecked(cloth_style.have_printings);
}

syt_msgs::msg::ClothStyle StyleDisplayWidget::getClothStyle() {
  syt_msgs::msg::ClothStyle cloth_style;
  cloth_style.elasticity_level        = style_id_map.value(ui->cloth_elesticity_combo_box->currentText());
  cloth_style.thickness_level         = style_id_map.value(ui->cloth_thickness_combo_box->currentText());
  cloth_style.cloth_type              = style_id_map.value(ui->cloth_type_combo_box->currentText());
  cloth_style.have_printings          = ui->have_printings_check_box->isChecked();
  cloth_style.cloth_color             = (uint32_t)ui->red_spin_box->value() << 16 | (uint32_t)ui->green_spin_box->value() << 8 | (uint32_t)ui->blue_spin_box->value();
  cloth_style.cloth_size              = style_id_map.value(ui->cloth_size_combo_box->currentText());
  cloth_style.glossiness_level        = style_id_map.value(ui->cloth_glossiness_combo_box->currentText());
  cloth_style.cloth_weight            = ui->cloth_weight_spin_box->value();
  cloth_style.cloth_length            = ui->cloth_length_line_edit->text().toFloat();
  cloth_style.bottom_length           = ui->bottom_length_line_edit->text().toFloat();
  cloth_style.oxter_length            = ui->oxter_length_line_edit->text().toFloat();
  cloth_style.shoulder_length         = ui->shoulder_length_line_edit->text().toFloat();
  cloth_style.side_length             = ui->side_length_line_edit->text().toFloat();
  cloth_style.cloth_length_tolerance  = ui->cloth_length_tolerance_spinbox->value();
  cloth_style.bottom_length_tolerance = ui->bottom_length_tolerance_spinbox->value();
  cloth_style.oxter_length_tolerance  = ui->oxter_length_tolerance_spinbox->value();
  cloth_style.matching_level          = style_id_map.value(ui->matching_level_combo_box->currentText());
  return cloth_style;
}

void StyleDisplayWidget::updateColorButton() {
  ui->cloth_color_btn->setStyleSheet(QString("background-color: rgb(%1, %2, %3);").arg(ui->red_spin_box->value()).arg(ui->green_spin_box->value()).arg(ui->blue_spin_box->value()));
}
