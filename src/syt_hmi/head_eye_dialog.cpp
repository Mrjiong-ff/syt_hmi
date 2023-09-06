//
// Created by jerry on 23-7-4.
//

// You may need to build the project (run Qt uic code generator) to get "ui_head_eye_dialog.h" resolved

#include "syt_hmi/head_eye_dialog.h"
#include "ui_head_eye_dialog.h"

HeadEyeDialog::HeadEyeDialog(QWidget *parent) : QDialog(parent), ui(new Ui::HeadEyeDialog) {
  ui->setupUi(this);
  this->setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
  this->setModal(true); // 设置为模态 父对象控件不可选

  ui->composerMachineBtn->setParentEnabled(true);
  ui->composerMachineBtn->setForeEnabled(false);
  ui->composerMachineBtn->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");

  ui->sewingMachineBtn->setParentEnabled(true);
  ui->sewingMachineBtn->setForeEnabled(false);
  ui->sewingMachineBtn->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");

  //    ui->sewingMachineBtn->setIcon(QPixmap::fromImage(
  //            QImage(":m_icon/icon/sewing.png").scaled(ui->sewingMachineBtn->width() / 2,
  //                                                     ui->sewingMachineBtn->height() / 2,
  //                                                     Qt::AspectRatioMode::KeepAspectRatio,
  //                                                     Qt::TransformationMode::SmoothTransformation)));
  //    ui->composerMachineBtn->setIcon(QPixmap::fromImage(
  //            QImage(":m_icon/icon/comp.png").scaled(ui->composerMachineBtn->width() / 2,
  //                                                   ui->composerMachineBtn->height() / 2,
  //                                                   Qt::AspectRatioMode::KeepAspectRatio,
  //                                                   Qt::TransformationMode::SmoothTransformation)));

  connect(ui->composerMachineBtn, &QPushButton::clicked, [=] {
    QString tip = "<html>"
                  "<head/><b>注意:</b>\n<body>"
                  "<p>即将启动<font color=\"red\"><b>合片台</b></font>标定.</p>"
                  "</body></html>";

    auto res = showMessageBox(this, WARN, tip, 2, {"确定", "返回"});
    switch (res) {
    case 0:
      qDebug("合片台标定-确认");
      emit signCompStart();
      this->accept();
    case 1:
      return;
    default:
      return;
    }
  });

  connect(ui->sewingMachineBtn, &QPushButton::clicked, [=] {
    QString tip = "<html>"
                  "<head/><b>注意:</b>\n<body>"
                  "<p>即将启动<font color=\"red\"><b>缝纫台</b></font>的标定.</p>"
                  "</body></html>";

    auto res = showMessageBox(this, WARN, tip, 2, {"确定", "返回"});
    switch (res) {
    case 0:
      qDebug("缝纫台标定-确认");
      emit signSewingStart();
      this->accept();
    case 1:
      return;
    default:
      return;
    }
  });

  connect(ui->closeBtn, &QPushButton::clicked, [=] {
    this->close();
  });
}

HeadEyeDialog::~HeadEyeDialog() {
  delete ui;
}
