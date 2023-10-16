#include "syt_hmi/hand_eye_dialog.h"
#include "ui_hand_eye_dialog.h"

HandEyeDialog::HandEyeDialog(QWidget *parent) : QDialog(parent), ui(new Ui::HandEyeDialog) {
  ui->setupUi(this);
  waiting_spinner_widget_ = new WaitingSpinnerWidget(this);
  this->setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
  this->setModal(true); // 设置为模态 父对象控件不可选

  ui->calib_compose_btn->setParentEnabled(true);
  ui->calib_compose_btn->setForeEnabled(false);
  ui->calib_compose_btn->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");

  ui->calib_sewing_btn->setParentEnabled(true);
  ui->calib_sewing_btn->setForeEnabled(false);
  ui->calib_sewing_btn->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");

  connect(ui->calib_compose_btn, &QPushButton::clicked, [=] {
    QString tip = tr("<html>"
                     "<head/><b>注意:</b>\n<body>"
                     "<p>即将启动<font color=\"red\"><b>合片台</b></font>标定。</p>"
                     "</body></html>");

    auto res = showMessageBox(this, WARN, tip, 2, {tr("确定"), tr("返回")});
    switch (res) {
    case 0:
      waiting_spinner_widget_->start();
      emit signCompStart();
    case 1:
      return;
    default:
      return;
    }
  });

  connect(ui->calib_sewing_btn, &QPushButton::clicked, [=] {
    QString tip = tr("<html>"
                     "<head/><b>注意:</b>\n<body>"
                     "<p>即将启动<font color=\"red\"><b>缝纫台</b></font>的标定。</p>"
                     "</body></html>");

    auto res = showMessageBox(this, WARN, tip, 2, {tr("确定"), tr("返回")});
    switch (res) {
    case 0:
      waiting_spinner_widget_->start();
      emit signSewingStart();
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

HandEyeDialog::~HandEyeDialog() {
  delete ui;
}

void HandEyeDialog::slotCompCalibRes(bool result) {
  waiting_spinner_widget_->stop();
  if (result) {
    showMessageBox(this, SUCCESS, tr("合片台标定成功"), 1, {tr("确认")});
    return;
  } else {
    showMessageBox(this, ERROR, tr("合片台标定失败,请联系相关人员"), 1, {tr("确认")});
    return;
  }
}

void HandEyeDialog::slotSewingCalibRes(bool result) {
  waiting_spinner_widget_->stop();
  if (result) {
    showMessageBox(this, SUCCESS, tr("缝纫台标定成功"), 1, {tr("确认")});
    return;
  } else {
    showMessageBox(this, ERROR, tr("缝纫台标定失败,请联系相关人员"), 1, {tr("确认")});
    return;
  }
}
