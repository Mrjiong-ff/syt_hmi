#include "syt_hmi/lock_dialog.h"
#include "ui_lock_dialog.h"

LockDialog::LockDialog(QWidget *parent) : QDialog(parent), ui(new Ui::LockDialog) {
  ui->setupUi(this);
  this->setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
  this->setAttribute(Qt::WA_TranslucentBackground); // 设置窗口背景透明
  this->setModal(true);                             // 设置为模态 父对象控件不可选
  this->setFixedSize(parent->width(), parent->height());
  ui->label->setPixmap(QPixmap::fromImage(QImage(":m_icon/icon/locking.png")));
  ui->pushButton->setIcon(QIcon(":m_icon/icon/unlock.svg"));
  ui->pushButton->setAlign(Qt::Alignment::enum_type::AlignCenter);
  connect(ui->pushButton, &QPushButton::clicked, [=] {
    this->close();
  });
}

LockDialog::~LockDialog() {
  delete ui;
}
