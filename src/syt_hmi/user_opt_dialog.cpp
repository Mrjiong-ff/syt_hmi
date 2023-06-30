//
// Created by jerry on 23-6-29.
//

// You may need to build the project (run Qt uic code generator) to get "ui_user_opt_dialog.h" resolved

#include <iostream>
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

    ui->label->setPixmap(QPixmap(":m_icon/icon/opt.png").scaled(80, 80, Qt::AspectRatioMode::KeepAspectRatio,
                                                                Qt::TransformationMode::SmoothTransformation));

    // dxf save dir


    QStringList colorStrList;
    colorStrList << "黑色" << "白色" << "红色" << "绿色" << "蓝色" << "紫色";
    ui->comboBox_3->addItems(colorStrList);

    QStringList sizeStrList;
    sizeStrList << "S" << "M" << "L" << "XL" << "XXL";
    ui->comboBox_4->addItems(sizeStrList);

    connect(ui->pushButton_3, &QPushButton::clicked, [=] {
        QString fileName = QFileDialog::getOpenFileName(this, "选择cad文件", "", "DXF Files (*.dxf)");
        if (!fileName.isEmpty()) {
            ui->lineEdit_2->setText(fileName);
        } else {
            return;
        }
    });

    connect(ui->confirmPushButton, &QPushButton::clicked, [=] {
        // todo 确认 emit给 rcl comm
        auto cad_path = ui->lineEdit_2->text();
        if (cad_path.isEmpty()) {
            showMessageBox(this, WARN, "CAD裁片文件路径不能为空", 1, {"返回"});
            return;
        }

        emit systemStart();
        // todo emit to rclcomm
        this->accept();

    });

    connect(ui->backPushButton, &QPushButton::clicked, [=] {
        close();
    });
}

UserOptDialog::~UserOptDialog() {
    delete ui;
}
