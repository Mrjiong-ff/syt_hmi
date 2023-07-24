#include "syt_hmi/input_extra_param_widget.h"
#include "ui_input_extra_param_widget.h"
#include <QDebug>

InputExtraParamWidget::InputExtraParamWidget(QWidget *parent) : QWidget(parent),
                                                                ui(new Ui::InputExtraParamWidget) {
  ui->setupUi(this);
  ui->cloth_type_combo_box->insertItem(-1, "前片");
  ui->cloth_type_combo_box->insertItem(-1, "后片");
  ui->cloth_size_combo_box->insertItem(-1, "S");
  ui->cloth_size_combo_box->insertItem(-1, "M");
  ui->cloth_size_combo_box->insertItem(-1, "L");
  ui->cloth_size_combo_box->insertItem(-1, "XL");
  ui->cloth_size_combo_box->insertItem(-1, "XXL");
  ui->cloth_size_combo_box->insertItem(-1, "XXXL");
  ui->cloth_elesticity_combo_box->insertItem(-1, "低弹性");
  ui->cloth_elesticity_combo_box->insertItem(-1, "中等弹性");
  ui->cloth_elesticity_combo_box->insertItem(-1, "高弹性");
  ui->cloth_thickness_combo_box->insertItem(-1, "薄");
  ui->cloth_thickness_combo_box->insertItem(-1, "中等");
  ui->cloth_thickness_combo_box->insertItem(-1, "厚");
  ui->cloth_glossiness_combo_box->insertItem(-1, "低光泽");
  ui->cloth_glossiness_combo_box->insertItem(-1, "中光泽");
  ui->cloth_glossiness_combo_box->insertItem(-1, "高光泽");

  elasticity_map_.insert("低弹性", 10);
  elasticity_map_.insert("中等弹性", 11);
  elasticity_map_.insert("高弹性", 12);

  type_map_.insert("前片", 0);
  type_map_.insert("后片", 1);

  thickness_map_.insert("薄", 20);
  thickness_map_.insert("中等", 21);
  thickness_map_.insert("后", 22);

  size_map_.insert("S", 30);
  size_map_.insert("M", 31);
  size_map_.insert("L", 32);
  size_map_.insert("XL", 33);
  size_map_.insert("XXL", 34);
  size_map_.insert("XXXL", 35);

  glossiness_map_.insert("低光泽", 40);
  glossiness_map_.insert("中光泽", 41);
  glossiness_map_.insert("高光泽", 42);

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
    qDebug() << (ui->red_spin_box->value() << 4 | ui->green_spin_box->value() << 2 | ui->blue_spin_box->value());
  });

  // 设置克数输入限制数字
  QRegExp     weight_exp("[0-9\\.]+$");
  QValidator *weight_validator = new QRegExpValidator(weight_exp);
  ui->cloth_weight_line_edit->setValidator(weight_validator);
}

InputExtraParamWidget::~InputExtraParamWidget() {
  delete ui;
}

void InputExtraParamWidget::setClothType(int cloth_type) {
  ui->cloth_type_combo_box->setCurrentText(cloth_type ? "后片" : "前片");
}

syt_msgs::msg::ClothStyle InputExtraParamWidget::getClothStyle() {
  syt_msgs::msg::ClothStyle cloth_style;
  cloth_style.elasticity_level = elasticity_map_.value(ui->cloth_elesticity_combo_box->currentText());
  cloth_style.thickness_level  = thickness_map_.value(ui->cloth_thickness_combo_box->currentText());
  cloth_style.cloth_type       = type_map_.value(ui->cloth_type_combo_box->currentText());
  cloth_style.have_printings   = ui->have_printings_check_box->isChecked();
  cloth_style.cloth_color      = (uint32_t)ui->red_spin_box->value() << 16 | (uint32_t)ui->green_spin_box->value() << 8 | (uint32_t)ui->blue_spin_box->value();
  cloth_style.cloth_size       = size_map_.value(ui->cloth_size_combo_box->currentText());
  cloth_style.glossiness_level = glossiness_map_.value(ui->cloth_glossiness_combo_box->currentText());
  cloth_style.cloth_weight     = ui->cloth_weight_line_edit->text().toFloat();
  return cloth_style;
}

void InputExtraParamWidget::updateColorButton() {
  ui->cloth_color_btn->setStyleSheet(QString("background-color: rgb(%1, %2, %3);").arg(ui->red_spin_box->value()).arg(ui->green_spin_box->value()).arg(ui->blue_spin_box->value()));
}