//
// Created by jerry on 23-6-28.
//

// You may need to build the project (run Qt uic code generator) to get "ui_ota_update_dialog.h" resolved

#include <iostream>
#include "syt_hmi/ota_update_dialog.h"
#include "ui_ota_update_dialog.h"


OtaUpdateDialog::OtaUpdateDialog(QWidget *parent) :
        QDialog(parent), ui(new Ui::OtaUpdateDialog) {
    ui->setupUi(this);
    this->setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
    this->setModal(true);  // 设置为模态 父对象控件不可选

    // todo 取消升级暂未实现接口
    ui->pushButton->setEnabled(false);

    connect(ui->pushButton, &QPushButton::clicked, [=] {
        this->reject();
    });

}

OtaUpdateDialog::~OtaUpdateDialog() {
    delete ui;
}

void OtaUpdateDialog::updateProcessValue(int val, int total) {
    auto i = (static_cast<float>(val) / static_cast<float>(total)) * 100;
    std::cout << "进度::" << i << std::endl;
    std::cout << "当前val::" << val << "总共::" << total << std::endl;
    ui->widget->setValue(i);
}

void OtaUpdateDialog::clearProcessValue() {
    ui->widget->setValue(0);
}

void OtaUpdateDialog::getDownloadRes(bool f, QString msg) {
    Q_UNUSED(msg);
    if (f) {
        this->done(9);
    } else {
        this->done(10);
    }
}


