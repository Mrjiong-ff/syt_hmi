#include "syt_hmi/translate_dialog.h"
#include "ui_translate_dialog.h"

TranslateDialog::TranslateDialog(QWidget *parent, int index) : QDialog(parent), ui(new Ui::TranslateDialog) {
  ui->setupUi(this);
  this->setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
  this->setModal(true);

  ui->confirm_btn->setParentEnabled(true);
  ui->confirm_btn->setForeEnabled(false);

  ui->cancel_btn->setParentEnabled(true);
  ui->cancel_btn->setForeEnabled(false);

  ui->language_combo_box->setCurrentIndex(index);

  connect(ui->confirm_btn, &QPushButton::clicked, this, [=]() {
    emit confirmLanguage(ui->language_combo_box->currentIndex());
    this->accept();
  });

  connect(ui->cancel_btn, &QPushButton::clicked, [=]() {
    this->reject();
  });
}

TranslateDialog::~TranslateDialog() {
  delete ui;
}
