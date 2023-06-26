//
// Created by jerry on 23-4-26.
//

// You may need to build the project (run Qt uic code generator) to get "ui_main_window.h" resolved

#include "syt_hmi/main_window.h"


MainWindow::MainWindow(QWidget *parent) :
        QMainWindow(parent), ui(new Ui::MainWindow) {

    ui->setupUi(this);

    initNode();

    initWidget();

    settingConnection();
}

MainWindow::~MainWindow() {
    delete rclcomm;
    delete ui;
}

void MainWindow::settingConnection() {
    // main window右上角4个按钮
    connect(m_closeBtn_, &WinCloseButton::clicked, this, &MainWindow::close);
    connect(m_minBtn_, &WinMaxButton::clicked, this, &MainWindow::showMinimized);
    connect(m_maxBtn_, &WinMaxButton::clicked, this, &MainWindow::slotMaxBtnClicked);

    connect(m_menuBtn_, &WinMenuButton::toggled, [=] {
        m_menu_->show();
        m_menu_->exec();
    });

    // 工具栏按钮
    connect(ui->sytDevModeBtn, &QPushButton::clicked, this, &MainWindow::slotShowDevLoginWindow);
    connect(ui->sytLockScreenBtn, &QPushButton::clicked, this, &MainWindow::slotLockScreen);

    // todo 帮助
    connect(ui->sytHelpPushButton, &QPushButton::clicked, [=] {
        auto workCursor = ui->sytPlainTextEdit->textCursor();
        workCursor.insertText(QString("test test test"));
        workCursor.insertBlock();
        workCursor.movePosition(QTextCursor::End);

//        //移动滚动条到底部
//        QScrollBar *scrollbar = ui->sytPlainTextEdit->verticalScrollBar();
//        if (scrollbar) {
//            scrollbar->setSliderPosition(scrollbar->maximum());
//        }
    });

    // todo log filter btn
//    connect(ui->sytFilterPushButton,&QPushButton::clicked,)
    connect(ui->sytClearPushButton, &QPushButton::clicked, [=] { ui->sytPlainTextEdit->clear(); });

    // action
    connect(minAct_, &QAction::triggered, this, &MainWindow::showMinimized);
    connect(maxAct_, &QAction::triggered, this, &MainWindow::slotMaxBtnClicked);
    connect(fullAct_, &QAction::triggered, this, &MainWindow::showFullScreen);
    connect(closeAct_, &QAction::triggered, this, &MainWindow::close);

    connect(devAct_, &QAction::triggered, this, &MainWindow::slotShowDevLoginWindow);

    connect(prev_btn, &QPushButton::clicked, this, &MainWindow::slotPrevPage);
    connect(next_btn_, &QPushButton::clicked, this, &MainWindow::slotNextPage);

    // 主界面3个程序按钮
    connect(ui->sytResetPushButton, &QPushButton::clicked, this, &MainWindow::resetBtnClicked);
    connect(ui->sytStartPushButton, &QPushButton::clicked, this, &MainWindow::startBtnClicked);
    connect(ui->sytStopPushButton, &QPushButton::clicked, this, &MainWindow::stopBtnClicked);
}

void MainWindow::initWidget() {
    // 用于捕获鼠标移动事件
    // 注意：mainwindow及其之类都要设置mouse track，不然不生效
    setMouseTracking(true);
    ui->centralwidget->setMouseTracking(true);

//    setFocusPolicy(Qt::FocusPolicy::StrongFocus);

    // 隐藏默认标题栏
    this->setWindowFlags(Qt::FramelessWindowHint);

    this->setWindowIcon(QIcon(":m_logo/logo/bg_logo.png"));

    // todo add logo title

    // 移动到中心
    QScreen *desktop = QApplication::screenAt(QCursor::pos());
    QRect rect = desktop->availableGeometry();
    move(rect.left() + (rect.width() - width()) / 2, (rect.height() - height()) / 2);

    m_menuBtn_ = new WinMenuButton(this);
    m_menuBtn_->setFixedSize(30, 30);
    m_minBtn_ = new WinMinButton(this);
    m_minBtn_->setFixedSize(30, 30);
    m_maxBtn_ = new WinMaxButton(this);
    m_maxBtn_->setFixedSize(30, 30);
    m_closeBtn_ = new WinCloseButton(this);
    m_closeBtn_->setFixedSize(30, 30);
    ui->mainLayout->addWidget(m_menuBtn_);
    ui->mainLayout->addWidget(m_minBtn_);
    ui->mainLayout->addWidget(m_maxBtn_);
    ui->mainLayout->addWidget(m_closeBtn_);

    // 为title widget设置上下文菜单
    ui->sytMainTitleWidget->setContextMenuPolicy(Qt::CustomContextMenu);
    m_titleMenu_ = new QMenu(this);
    minAct_ = new QAction(this);
    maxAct_ = new QAction(this);
    fullAct_ = new QAction(this);
    closeAct_ = new QAction(this);
    minAct_->setText("最小化");
    maxAct_->setText("最大化");
    fullAct_->setText("全屏窗口化");
    closeAct_->setText("关闭");
    m_titleMenu_->addAction(minAct_);
    m_titleMenu_->addAction(maxAct_);
    m_titleMenu_->addAction(fullAct_);
    m_titleMenu_->addAction(closeAct_);

    // menu btn上下文
    m_menu_ = new QMenu(this);
    devAct_ = new QAction(this);
    helpAct_ = new QAction(this);
    aboutAct_ = new QAction(this);
    devAct_->setText("开发者模式");
    helpAct_->setText("帮助文档");
    aboutAct_->setText("关于速英");
    m_menu_->addAction(devAct_);
    m_menu_->addAction(helpAct_);
    m_menu_->addAction(aboutAct_);
    m_menuBtn_->setMenu(m_menu_);

    // tool bar btn
    ui->sytDevModeBtn->setIcon(QIcon(":m_icon/icon/dev-mode.png"));  // 开发者模式按钮
    ui->sytDevModeBtn->setToolTip(QString("开发者模式"));

    ui->sytLockScreenBtn->setIcon(QIcon(":m_icon/icon/lock.png"));  // 屏幕上锁/解锁按钮
    ui->sytLockScreenBtn->setToolTip(QString("界面上锁/解锁"));

    ui->sytHelpPushButton->setIcon(QIcon(":m_icon/icon/help.png"));
    ui->sytHelpPushButton->setToolTip(QString("帮助说明"));

    // 调整分割位置
    ui->mainHSplitter->setStretchFactor(0, 5);
    ui->mainHSplitter->setStretchFactor(1, 2);

    ui->mainVSplitter->setStretchFactor(0, 3);
    ui->mainVSplitter->setStretchFactor(1, 1);

    // main tab btn
    ui->sytResetPushButton->setParentEnabled(true);
    ui->sytResetPushButton->setForeEnabled(false);
    ui->sytResetPushButton->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");

    ui->sytStartPushButton->setParentEnabled(true);
    ui->sytStartPushButton->setForeEnabled(false);
    ui->sytStartPushButton->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");

    ui->sytStopPushButton->setParentEnabled(true);
    ui->sytStopPushButton->setForeEnabled(false);
    ui->sytStopPushButton->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");

    // log view btn
    ui->sytFilterPushButton->setIcon(QIcon(":m_icon/icon/filter-records.png"));
    ui->sytFilterPushButton->setToolTip(QString("日志过滤筛选"));

    ui->sytClearPushButton->setIcon(QIcon(":m_icon/icon/clear.png"));
    ui->sytClearPushButton->setToolTip(QString("日志清除"));


    // 添加左右翻页按钮
    int init_page_btn_w = this->width() / page_btn_w_scale_;
    int init_page_btn_h = this->height() / page_btn_h_scale_;

    prev_btn = new InteractiveButtonBase(this);
    prev_btn->setIcon(QIcon(":m_icon/icon/l-page.png"));
    prev_btn->setParentEnabled(true);
    prev_btn->setForeEnabled(false);
    prev_btn->setStyleSheet("background-color: rgba(150,150,150, 0.3);qproperty-press_color: rgba(0,0,100,0.3);");
//    prev_btn->setToolTip(QString("上一页,快捷键A"));

    next_btn_ = new InteractiveButtonBase(this);
    next_btn_->setIcon(QIcon(":m_icon/icon/r-page.png"));
    next_btn_->setParentEnabled(true);
    next_btn_->setForeEnabled(false);
    next_btn_->setStyleSheet("background-color:  rgba(150,150,150, 0.3);qproperty-press_color: rgba(0,0,100,0.3);");
//    next_btn_->setToolTip(QString("下一页,快捷键D"));

    prev_btn->setGeometry(0, this->height() / 2 - init_page_btn_h / 2, init_page_btn_w, init_page_btn_h);
    next_btn_->setGeometry(this->width() - init_page_btn_w, this->height() / 2 - init_page_btn_h / 2,
                           init_page_btn_w,
                           init_page_btn_h);

    // todo rviz
//    rviz_widget_ = new QRviz(ui->rviz_verticalLayout, "syt_hmi/");
//    rviz_widget_->GetDisplayTreeModel();
//    QMap<QString, QVariant> namevalue;
//    namevalue.insert("Line Style", "Billboards");
//    namevalue.insert("Color", QColor(150, 150, 150));
//    namevalue.insert("Plane Cell Count", 10);
//    rviz_widget_->DisplayInit(RVIZ_DISPLAY_GRID, "Grid", true, namevalue);
////    rviz_widget_->createDisplay(RVIZ_DISPLAY_MARKER,)
////    rviz_widget_->DisplayInit(RVIZ_DISPLAY_AXES, "Axes", true, {});

    // 事件过滤
    ui->sytMainTitleWidget->installEventFilter(this);
}

void MainWindow::slotMaxBtnClicked() {
    if (is_normal_size_) {
        showMaximized();
    } else {
        showNormal();
    }
    is_normal_size_ = !is_normal_size_;
}

void MainWindow::mousePressEvent(QMouseEvent *event) {
    switch (event->button()) {
        case Qt::LeftButton:
            is_mouse_left_press_down_ = true;

            if (dir_ != NONE) {
                this->mouseGrabber(); //返回当前抓取鼠标输入的窗口
            } else {
                m_mousePos_ = event->globalPos() - this->frameGeometry().topLeft();
            }
            break;
        case Qt::RightButton:
//            this->setWindowState(Qt::WindowMinimized);
            break;
        default:
            return;
    }
}

void MainWindow::mouseMoveEvent(QMouseEvent *event) {
    QPoint globalPoint = event->globalPos();   //鼠标全局坐标
    QRect rect = this->rect();  //rect == QRect(0,0 1280x720)
    QPoint topLeft = mapToGlobal(rect.topLeft());
    QPoint bottomRight = mapToGlobal(rect.bottomRight());

    if (this->windowState() != Qt::WindowMaximized) {
        if (!is_mouse_left_press_down_)  //没有按下左键时
        {
            this->region(globalPoint); //窗口大小的改变——判断鼠标位置，改变光标形状
        } else {
            if (dir_ != NONE) {
                QRect newRect(topLeft, bottomRight); //定义一个矩形  拖动后最大1000*1618

                switch (dir_) {
                    case LEFT:

                        if (bottomRight.x() - globalPoint.x() <= this->minimumWidth()) {
                            newRect.setLeft(topLeft.x());  //小于界面的最小宽度时，设置为左上角横坐标为窗口x
                            //只改变左边界
                        } else {
                            newRect.setLeft(globalPoint.x());
                        }
                        break;
                    case RIGHT:
                        newRect.setWidth(globalPoint.x() - topLeft.x());  //只能改变右边界
                        break;
                    case UP:
                        if (bottomRight.y() - globalPoint.y() <= this->minimumHeight()) {
                            newRect.setY(topLeft.y());
                        } else {
                            newRect.setY(globalPoint.y());
                        }
                        break;
                    case DOWN:
                        newRect.setHeight(globalPoint.y() - topLeft.y());
                        break;
                    case LEFTTOP:
                        if (bottomRight.x() - globalPoint.x() <= this->minimumWidth()) {
                            newRect.setX(topLeft.x());
                        } else {
                            newRect.setX(globalPoint.x());
                        }

                        if (bottomRight.y() - globalPoint.y() <= this->minimumHeight()) {
                            newRect.setY(topLeft.y());
                        } else {
                            newRect.setY(globalPoint.y());
                        }
                        break;
                    case RIGHTTOP:
                        if (globalPoint.x() - topLeft.x() >= this->minimumWidth()) {
                            newRect.setWidth(globalPoint.x() - topLeft.x());
                        } else {
                            newRect.setWidth(bottomRight.x() - topLeft.x());
                        }
                        if (bottomRight.y() - globalPoint.y() >= this->minimumHeight()) {
                            newRect.setY(globalPoint.y());
                        } else {
                            newRect.setY(topLeft.y());
                        }
                        break;
                    case LEFTBOTTOM:
                        if (bottomRight.x() - globalPoint.x() >= this->minimumWidth()) {
                            newRect.setX(globalPoint.x());
                        } else {
                            newRect.setX(topLeft.x());
                        }
                        if (globalPoint.y() - topLeft.y() >= this->minimumHeight()) {
                            newRect.setHeight(globalPoint.y() - topLeft.y());
                        } else {
                            newRect.setHeight(bottomRight.y() - topLeft.y());
                        }
                        break;
                    case RIGHTBOTTOM:
                        newRect.setWidth(globalPoint.x() - topLeft.x());
                        newRect.setHeight(globalPoint.y() - topLeft.y());
                        break;
                    default:
                        break;
                }
                this->setGeometry(newRect);
            } else {
                move(event->globalPos() - m_mousePos_); //移动窗口
                event->accept();

            }
        }
    }

}


void MainWindow::mouseReleaseEvent(QMouseEvent *event) {
    if (event->button() == Qt::LeftButton) {
        is_mouse_left_press_down_ = false;
        if (dir_ != NONE) {
            this->releaseMouse(); //释放鼠标抓取
            this->setCursor(QCursor(Qt::ArrowCursor));
            dir_ = NONE;
        }
    } else {
        return;
    }
}

void MainWindow::region(const QPoint &currentGlobalPoint) {
// 获取窗体在屏幕上的位置区域，topLeft为坐上角点，rightButton为右下角点
    QRect rect = this->rect();

    QPoint topLeft = this->mapToGlobal(rect.topLeft()); //将左上角的(0,0)转化为全局坐标
    QPoint rightButton = this->mapToGlobal(rect.bottomRight());

    int x = currentGlobalPoint.x(); //当前鼠标的坐标
    int y = currentGlobalPoint.y();

    if (((topLeft.x() + PADDING >= x) && (topLeft.x() <= x))
        && ((topLeft.y() + PADDING >= y) && (topLeft.y() <= y))) {
        // 左上角
        dir_ = LEFTTOP;
        this->setCursor(QCursor(Qt::SizeFDiagCursor));  // 设置光标形状
    } else if (((x >= rightButton.x() - PADDING) && (x <= rightButton.x()))
               && ((y >= rightButton.y() - PADDING) && (y <= rightButton.y()))) {
        // 右下角
        dir_ = RIGHTBOTTOM;
        this->setCursor(QCursor(Qt::SizeFDiagCursor));
    } else if (((x <= topLeft.x() + PADDING) && (x >= topLeft.x()))
               && ((y >= rightButton.y() - PADDING) && (y <= rightButton.y()))) {
        //左下角
        dir_ = LEFTBOTTOM;
        this->setCursor(QCursor(Qt::SizeBDiagCursor));
    } else if (((x <= rightButton.x()) && (x >= rightButton.x() - PADDING))
               && ((y >= topLeft.y()) && (y <= topLeft.y() + PADDING))) {
        // 右上角
        dir_ = RIGHTTOP;
        this->setCursor(QCursor(Qt::SizeBDiagCursor));
    } else if ((x <= topLeft.x() + PADDING) && (x >= topLeft.x())) {
        // 左边
        dir_ = LEFT;
        this->setCursor(QCursor(Qt::SizeHorCursor));
    } else if ((x <= rightButton.x()) && (x >= rightButton.x() - PADDING)) {
        // 右边
        dir_ = RIGHT;
        this->setCursor(QCursor(Qt::SizeHorCursor));
    } else if ((y >= topLeft.y()) && (y <= topLeft.y() + PADDING)) {
        // 上边
        dir_ = UP;
        this->setCursor(QCursor(Qt::SizeVerCursor));
    } else if ((y <= rightButton.y()) && (y >= rightButton.y() - PADDING)) {
        // 下边
        dir_ = DOWN;
        this->setCursor(QCursor(Qt::SizeVerCursor));
    } else {
        // 默认
        dir_ = NONE;
        this->setCursor(QCursor(Qt::ArrowCursor));
    }
}

bool MainWindow::eventFilter(QObject *obj, QEvent *ev) {
    // 双击title栏，最大化/正常化
    if (obj == ui->sytMainTitleWidget) {
        if (ev->type() == QEvent::MouseButtonDblClick) {
            emit slotMaxBtnClicked();
            return true;
        } else if (ev->type() == QEvent::ContextMenu) {
            m_titleMenu_->exec(QCursor::pos());
            return true;
        }
        return false;

    }
    return QObject::eventFilter(obj, ev);
}

void MainWindow::slotShowDevLoginWindow() {
    dev_login_window_ = new DevLoginWindow(this);
    connect(dev_login_window_, &DevLoginWindow::signDevMode, this, &MainWindow::slotDevWindow);
    dev_login_window_->show();
}

void MainWindow::slotLockScreen() {
    is_lock_screen_ = !is_lock_screen_;
    if (is_lock_screen_) {
        ui->sytLockScreenBtn->setIcon(QIcon(":m_icon/icon/unlock.png"));
        setAllButtonsEnabled(this, false, ui->sytLockScreenBtn);
    } else {
        ui->sytLockScreenBtn->setIcon(QIcon(":m_icon/icon/lock.png"));
        setAllButtonsEnabled(this, true, ui->sytLockScreenBtn);
    }
}

void MainWindow::setAllButtonsEnabled(QWidget *parent, bool enabled, QPushButton *excludedButton) {
    // 遍历父控件下的所有子控件,禁用所有btn，仅保留lock btn
    QList<QWidget *> childWidgets = parent->findChildren<QWidget *>();
    for (auto widget: childWidgets) {
        // 如果是QPushButton类型并且不是排除的按钮，则设置其enabled状态
        QPushButton *button = qobject_cast<QPushButton *>(widget);
        if (button && button != excludedButton) {
            button->setEnabled(enabled);
        }
    }
}


void MainWindow::slotDevWindow() {
    this->hide();

    dev_login_window_->deleteLater();
    dev_login_window_ = nullptr;

    // dev main window
    auto dev_window = new DevWindow(this);
    dev_window->setAttribute(Qt::WA_DeleteOnClose);
    connect(dev_window, &DevWindow::closeDevWindow, [=] { this->show(); });
    connect(dev_window, &DevWindow::destroyed, [=] { dev_window->deleteLater(); });
    dev_window->show();
}

void MainWindow::resizeEvent(QResizeEvent *event) {
    if (event->type() == QResizeEvent::Resize) {
        // 翻页按钮
        int init_page_btn_w = this->width() / page_btn_w_scale_;
        int init_page_btn_h = this->height() / page_btn_h_scale_;

        prev_btn->setGeometry(0, this->height() / 2 - init_page_btn_h / 2, init_page_btn_w, init_page_btn_h);
        next_btn_->setGeometry(this->width() - init_page_btn_w, this->height() / 2 - init_page_btn_h / 2,
                               init_page_btn_w,
                               init_page_btn_h);

    }

//    QWidget::resizeEvent(event);
}

void MainWindow::slotPrevPage() {
    int currentIndex = ui->stackedWidget->currentIndex();
    if (currentIndex == 0) {
        ui->stackedWidget->setCurrentIndex(ui->stackedWidget->count() - 1);
    } else {
        ui->stackedWidget->setCurrentIndex(currentIndex - 1);
    }
}

void MainWindow::slotNextPage() {
    int currentIndex = ui->stackedWidget->currentIndex();
    if (currentIndex == ui->stackedWidget->count() - 1) {
        ui->stackedWidget->setCurrentIndex(0);
    } else {
        ui->stackedWidget->setCurrentIndex(currentIndex + 1);
    }
}

void MainWindow::keyPressEvent(QKeyEvent *event) {
    switch (event->key()) {
        case Qt::Key_A:
            emit prev_btn->click();
            break;

        case Qt::Key_D:
            emit next_btn_->click();
            break;

        default:
            break;
    }
}

void MainWindow::initNode() {
    rclcomm = new SytRclComm();

}

void MainWindow::resetBtnClicked() {
    bool res = isFastClick(ui->sytResetPushButton, 1000);
    if (!res) {
        return;
    }
    qDebug("点击重置按钮");

}

void MainWindow::startBtnClicked() {
    bool res = isFastClick(ui->sytStartPushButton, 1000);
    if (!res) {
        return;
    }
    qDebug("点击开始按钮");

}

void MainWindow::stopBtnClicked() {
    bool res = isFastClick(ui->sytStopPushButton, 1000);
    if (!res) {
        return;
    }
    qDebug("点击停止按钮");

}
