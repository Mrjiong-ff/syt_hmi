//
// Created by jerry on 23-6-29.
//

// You may need to build the project (run Qt uic code generator) to get "ui_user_opt_dialog.h" resolved

#include "syt_hmi/user_opt_dialog.h"
#include "ui_user_opt_dialog.h"


UserOptDialog::UserOptDialog(QWidget *parent) :
        QDialog(parent), ui(new Ui::UserOptDialog) {
    ui->setupUi(this);
    this->setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
    this->setModal(true);  // 设置为模态 父对象控件不可选

    ui->pushButton_3->setParentEnabled(true);
    ui->pushButton_3->setForeEnabled(false);
    ui->pushButton_3->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");

    ui->backPushButton->setParentEnabled(true);
    ui->backPushButton->setForeEnabled(false);
    ui->backPushButton->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");

    ui->confirmPushButton->setParentEnabled(true);
    ui->confirmPushButton->setForeEnabled(false);
    ui->confirmPushButton->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");

    ui->label->setPixmap(QPixmap(":m_icon/icon/opt.png").scaled(70, 70, Qt::AspectRatioMode::KeepAspectRatio,
                                                                Qt::TransformationMode::SmoothTransformation));

    connect(ui->pushButton_3, &QPushButton::clicked, [=] {
        // todo 打开选取

    });

    connect(ui->confirmPushButton, &QPushButton::clicked, [=] {
       // todo 确认 emit给mainwindow

    });

    connect(ui->backPushButton, &QPushButton::clicked, [=] {
        close();
    });
}

UserOptDialog::~UserOptDialog() {
    delete ui;
}
