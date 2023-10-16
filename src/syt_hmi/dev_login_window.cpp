#include "syt_hmi/dev_login_window.h"

DevLoginWindow::DevLoginWindow(QWidget *parent) : QDialog(parent), ui(new Ui::DevLoginWindow) {
  ui->setupUi(this);

  setFixedSize(640, 480);

  // 设置模态窗口的方式阻止用户操作父窗口
  setModal(true);
  // dialog必须要加 Qt::Dialog，不然背景会变成透明
  setWindowFlags(Qt::FramelessWindowHint | Qt::Dialog);

  m_closeBtn_ = new WinCloseButton(this);
  m_closeBtn_->setFixedSize(30, 30);

  ui->sytLoginTitHorizontalLayout->addWidget(m_closeBtn_);

  ui->pushButton->setBgColor(QColor(180, 180, 180, 100));
  ui->pushButton->setRadius(10, 5);
  ui->pushButton->setChokingProp(0.0018);

  ui->pixLabel->setPixmap(QPixmap(":m_icon/icon/login.png"));
  ui->pixLabel->setAlignment(Qt::AlignCenter);

  connect(ui->pwd_le, &QLineEdit::returnPressed, ui->pushButton, &QPushButton::click, Qt::UniqueConnection);
  connect(m_closeBtn_, &QPushButton::clicked, [=] { close(); });
  // 登录按钮
  connect(ui->pushButton, &QPushButton::clicked, [=] {
    QString token;
    token.append('a');
    //token.append('s');
    //token.append('e');
    //token.append('w');
    //token.append('i');
    //token.append('n');
    //token.append('g');
    //token.append('t');
    //token.append('e');
    //token.append('c');
    //token.append('h');
    //token.append('0');
    //token.append('3');
    //token.append('1');
    //token.append('0');
    //token.append('6');
    //token.append('6');
    //token.append('6');
    //token.append('6');
    // 当前默认密码
    if (ui->pwd_le->text() == token) {
      emit signDevMode(true);
      hide();
    } else {
      ui->pwd_le->setText("");
      showMessageBox(this, STATE::ERROR, tr("密码输入错误，请重新输入或联系管理员!"), 1, {tr("确认")});
    }
  });
}

DevLoginWindow::~DevLoginWindow() {
  delete ui;
}
