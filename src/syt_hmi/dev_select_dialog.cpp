//
// Created by jerry on 23-7-4.
//

// You may need to build the project (run Qt uic code generator) to get "ui_dev_select_dialog.h" resolved

#include "syt_hmi/dev_select_dialog.h"
#include "ui_dev_select_dialog.h"


DevSelectDialog::DevSelectDialog(QWidget *parent) :
        QDialog(parent), ui(new Ui::DevSelectDialog) {
    ui->setupUi(this);
    this->setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
    this->setModal(true);  // 设置为模态 父对象控件不可选

    ui->visionBtn->setParentEnabled(true);
    ui->visionBtn->setForeEnabled(false);
    ui->visionBtn->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");

    ui->debugBtn->setParentEnabled(true);
    ui->debugBtn->setForeEnabled(false);
    ui->debugBtn->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");

    connect(ui->closeBtn, &QPushButton::clicked, [=] {
        this->close();
    });
}

DevSelectDialog::~DevSelectDialog() {
    delete ui;
}
