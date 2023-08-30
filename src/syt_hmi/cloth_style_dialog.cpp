#include "syt_hmi/cloth_style_dialog.h"
#include "ui_cloth_style_dialog.h"

ClothStyleDialog::ClothStyleDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::ClothStyleDialog) {
  ui->setupUi(this);
  this->setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
  this->setModal(true); // 设置为模态 父对象控件不可选

  ui->create_from_cad_btn->setParentEnabled(true);
  ui->create_from_cad_btn->setForeEnabled(false);
  ui->create_from_cad_btn->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");
  ui->create_from_cad_btn->setStyleSheet("color: gray;"); // TODO: delete

  ui->auto_create_style_btn->setParentEnabled(true);
  ui->auto_create_style_btn->setForeEnabled(false);
  ui->auto_create_style_btn->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");

  ui->manual_input_param_btn->setParentEnabled(true);
  ui->manual_input_param_btn->setForeEnabled(false);
  ui->manual_input_param_btn->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");

  ui->create_from_source_btn->setParentEnabled(true);
  ui->create_from_source_btn->setForeEnabled(false);
  ui->create_from_source_btn->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");

  connect(ui->create_from_cad_btn, &QPushButton::clicked, [=] {
    emit signCreateFromCAD(this);
  });

  connect(ui->auto_create_style_btn, &QPushButton::clicked, [=] {
    emit signAutoCreateStyle(this);
  });

  connect(ui->manual_input_param_btn, &QPushButton::clicked, [=] {
    emit signManualInputParam(this);
  });

  connect(ui->create_from_source_btn, &QPushButton::clicked, [=] {
    emit signCreateFromSource(this);
  });

  connect(ui->min_btn, &QPushButton::clicked, [=] {
    parent->showMinimized();
  });

  connect(ui->close_btn, &QPushButton::clicked, [=] {
    this->close();
  });
}

ClothStyleDialog::~ClothStyleDialog() {
  delete ui;
}
