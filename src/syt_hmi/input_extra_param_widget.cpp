#include "syt_hmi/input_extra_param_widget.h"
#include "ui_input_extra_param_widget.h"

InputExtraParamWidget::InputExtraParamWidget(QWidget *parent) : QWidget(parent),
                                                                ui(new Ui::InputExtraParamWidget) {
  ui->setupUi(this);
  ui->cloth_type_combo_box->setDisabled(true);
  ui->cloth_type_combo_box->insertItem(-1, "前片");
  ui->cloth_type_combo_box->insertItem(-1, "后片");
  ui->cloth_size_combo_box->insertItem(-1, "2XS");
  ui->cloth_size_combo_box->insertItem(-1, "XS");
  ui->cloth_size_combo_box->insertItem(-1, "S");
  ui->cloth_size_combo_box->insertItem(-1, "M");
  ui->cloth_size_combo_box->insertItem(-1, "L");
  ui->cloth_size_combo_box->insertItem(-1, "XL");
  ui->cloth_size_combo_box->insertItem(-1, "2XL");
  ui->cloth_size_combo_box->insertItem(-1, "3XL");
  ui->cloth_size_combo_box->insertItem(-1, "4XL");
  ui->cloth_elesticity_combo_box->insertItem(-1, "低弹性");
  ui->cloth_elesticity_combo_box->insertItem(-1, "中弹性");
  ui->cloth_elesticity_combo_box->insertItem(-1, "高弹性");
  ui->cloth_thickness_combo_box->insertItem(-1, "薄");
  ui->cloth_thickness_combo_box->insertItem(-1, "中");
  ui->cloth_thickness_combo_box->insertItem(-1, "厚");
  ui->cloth_glossiness_combo_box->insertItem(-1, "低光泽");
  ui->cloth_glossiness_combo_box->insertItem(-1, "中光泽");
  ui->cloth_glossiness_combo_box->insertItem(-1, "高光泽");

  elasticity_map_.insert("低弹性", 10);
  elasticity_map_.insert("中弹性", 11);
  elasticity_map_.insert("高弹性", 12);

  type_map_.insert("前片", 0);
  type_map_.insert("后片", 1);

  thickness_map_.insert("薄", 20);
  thickness_map_.insert("中等", 21);
  thickness_map_.insert("后", 22);

  size_map_.insert("2XS", 30);
  size_map_.insert("XS", 31);
  size_map_.insert("S", 32);
  size_map_.insert("M", 33);
  size_map_.insert("L", 34);
  size_map_.insert("XL", 35);
  size_map_.insert("2XL", 36);
  size_map_.insert("3XL", 37);
  size_map_.insert("4XL", 38);

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
  });

  ui->cloth_thickness_label->setToolTip(QString("0.35~0.45mm（薄）\n0.45~1mm（中等）\n大于1mm（厚）"));
  ui->cloth_thickness_combo_box->setToolTip(QString("0.35~0.45mm（薄）\n0.45~1mm（中等）\n大于1mm（厚）"));
  ui->cloth_weight_label->setToolTip(QString("范围0~200.000克"));
  ui->cloth_weight_spin_box->setToolTip(QString("范围0~200.000克"));

  connect(ui->cloth_weight_spin_box, SIGNAL(valueChanged(double)), this, SLOT(truncClothWeight(double)));
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
  cloth_style.cloth_weight     = ui->cloth_weight_spin_box->value();
  return cloth_style;
}

void InputExtraParamWidget::updateColorButton() {
  ui->cloth_color_btn->setStyleSheet(QString("background-color: rgb(%1, %2, %3);").arg(ui->red_spin_box->value()).arg(ui->green_spin_box->value()).arg(ui->blue_spin_box->value()));
}

void InputExtraParamWidget::truncClothWeight(double value) {
  if (value > 200.000) {
    ui->cloth_weight_spin_box->setValue(200.000);
  }
}
