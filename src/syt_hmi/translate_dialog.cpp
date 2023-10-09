#include "syt_hmi/translate_dialog.h"
#include "ui_translate_dialog.h"

TranslateDialog::TranslateDialog(QWidget *parent) : QDialog(parent), ui(new Ui::TranslateDialog) {
  ui->setupUi(this);
  this->setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
  this->setModal(true);

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
