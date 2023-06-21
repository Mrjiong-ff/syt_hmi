//
// Created by jerry on 23-4-28.
//

// You may need to build the project (run Qt uic code generator) to get "ui_dev_login_window.h" resolved

#include "syt_hmi/dev_login_window.h"


DevLoginWindow::DevLoginWindow(QWidget *parent) :
        QDialog(parent), ui(new Ui::DevLoginWindow) {
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
//    ui->pushButton->setBgColor(Qt::transparent, QColor(255, 255, 255, 100));
    ui->pushButton->setRadius(10, 5);
    ui->pushButton->setChokingProp(0.0018);


    ui->pixLabel->setPixmap(QPixmap(":m_icon/icon/Login-Form.png"));
    ui->pixLabel->setAlignment(Qt::AlignCenter);

    connect(m_closeBtn_, &QPushButton::clicked, [=] { close(); });
    // 登录按钮
    connect(ui->pushButton, &QPushButton::clicked, [=] {
        // todo 当前默认密码123
        if (ui->pwd_le->text() == "123") {
            emit signDevMode(true);
            hide();
        } else {
            show_message_box(this, "密码输入错误,请重新输入或联系管理员!", 1, {"确认"});
//            emit signDevMode(false);
        }
    });
}

DevLoginWindow::~DevLoginWindow() {
    delete ui;
}

//void DevLoginWindow::closeEvent(QCloseEvent *e) {
////    Q_UNUSED(e);
//    delete ui;
//}
