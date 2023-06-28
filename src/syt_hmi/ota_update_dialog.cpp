//
// Created by jerry on 23-6-28.
//

// You may need to build the project (run Qt uic code generator) to get "ui_ota_update_dialog.h" resolved

#include "syt_hmi/ota_update_dialog.h"
#include "ui_ota_update_dialog.h"


OtaUpdateDialog::OtaUpdateDialog(QWidget *parent) :
        QDialog(parent), ui(new Ui::OtaUpdateDialog) {
    ui->setupUi(this);
    this->setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
    this->setModal(true);  // 设置为模态 父对象控件不可选

    // todo
    connect(ui->pushButton, &QPushButton::clicked, [=] {
        this->close();
    });

}

OtaUpdateDialog::~OtaUpdateDialog() {
    delete ui;
}


