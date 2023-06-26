//
// Created by jerry on 23-4-28.
//

// You may need to build the project (run Qt uic code generator) to get "ui_tip_dialog.h" resolved

#include "syt_hmi/tip_dialog.h"


TipDialog::TipDialog(QWidget *parent) :
        QDialog(parent), ui(new Ui::TipDialog) {
    ui->setupUi(this);
}

TipDialog::~TipDialog() {
    delete ui;
}
