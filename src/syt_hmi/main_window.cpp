#include "syt_hmi/main_window.h"
#include <memory>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow) {

  ui->setupUi(this);

  // 初始化节点
  initNode();

  // 初始化控件
  initWidget();

  // 初始化其他（主要是配置相关）
  initOther();

  // 信号槽
  settingConnection();

  // todo 自动启动launch下所有节点(得根据实际情况来)
  bool res = rclcomm_->initAllNodes();
  if (!res) {
    if (rclcomm_ != nullptr) {
      delete rclcomm_;
    }
    delete ui;
    exit(-1);
  }
}

MainWindow::~MainWindow() {
  this->deleteAll();
}

void MainWindow::region(const QPoint &currentGlobalPoint) {
  // 获取窗体在屏幕上的位置区域，topLeft为坐上角点，rightButton为右下角点
  QRect rect = this->rect();

  QPoint topLeft     = this->mapToGlobal(rect.topLeft()); // 将左上角的(0,0)转化为全局坐标
  QPoint rightButton = this->mapToGlobal(rect.bottomRight());

  int x = currentGlobalPoint.x(); // 当前鼠标的坐标
  int y = currentGlobalPoint.y();

  if (((topLeft.x() + PADDING >= x) && (topLeft.x() <= x)) && ((topLeft.y() + PADDING >= y) && (topLeft.y() <= y))) {
    // 左上角
    dir_ = LEFTTOP;
    this->setCursor(QCursor(Qt::SizeFDiagCursor)); // 设置光标形状
  } else if (((x >= rightButton.x() - PADDING) && (x <= rightButton.x())) && ((y >= rightButton.y() - PADDING) && (y <= rightButton.y()))) {
    // 右下角
    dir_ = RIGHTBOTTOM;
    this->setCursor(QCursor(Qt::SizeFDiagCursor));
  } else if (((x <= topLeft.x() + PADDING) && (x >= topLeft.x())) && ((y >= rightButton.y() - PADDING) && (y <= rightButton.y()))) {
    // 左下角
    dir_ = LEFTBOTTOM;
    this->setCursor(QCursor(Qt::SizeBDiagCursor));
  } else if (((x <= rightButton.x()) && (x >= rightButton.x() - PADDING)) && ((y >= topLeft.y()) && (y <= topLeft.y() + PADDING))) {
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

void MainWindow::mousePressEvent(QMouseEvent *event) {
  switch (event->button()) {
  case Qt::LeftButton:
    is_mouse_left_press_down_ = true;

    if (dir_ != NONE) {
      this->mouseGrabber(); // 返回当前抓取鼠标输入的窗口
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
  QPoint globalPoint = event->globalPos(); // 鼠标全局坐标
  QRect rect         = this->rect();       // rect == QRect(0,0 1280x720)
  QPoint topLeft     = mapToGlobal(rect.topLeft());
  QPoint bottomRight = mapToGlobal(rect.bottomRight());

  if (this->windowState() != Qt::WindowMaximized) {
    if (!is_mouse_left_press_down_) // 没有按下左键时
    {
      this->region(globalPoint); // 窗口大小的改变——判断鼠标位置，改变光标形状
    } else {
      if (dir_ != NONE) {
        QRect newRect(topLeft, bottomRight); // 定义一个矩形  拖动后最大1000*1618

        switch (dir_) {
        case LEFT:
          if (bottomRight.x() - globalPoint.x() <= this->minimumWidth()) {
            newRect.setLeft(topLeft.x()); // 小于界面的最小宽度时，设置为左上角横坐标为窗口x
            // 只改变左边界
          } else {
            newRect.setLeft(globalPoint.x());
          }
          break;
        case RIGHT:
          newRect.setWidth(globalPoint.x() - topLeft.x()); // 只能改变右边界
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
        move(event->globalPos() - m_mousePos_); // 移动窗口
        event->accept();
      }
    }
  }
}

void MainWindow::mouseReleaseEvent(QMouseEvent *event) {
  if (event->button() == Qt::LeftButton) {
    is_mouse_left_press_down_ = false;
    if (dir_ != NONE) {
      this->releaseMouse(); // 释放鼠标抓取
      this->setCursor(QCursor(Qt::ArrowCursor));
      dir_ = NONE;
    }
  } else {
    return;
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

void MainWindow::resizeEvent(QResizeEvent *event) {
  if (event->type() == QResizeEvent::Resize) {
    // 翻页按钮
    int init_page_btn_w = this->width() / 50;
    int init_page_btn_h = this->height() / 9;

    prev_btn->setGeometry(0, this->height() / 2 - init_page_btn_h / 2, init_page_btn_w, init_page_btn_h);
    next_btn_->setGeometry(this->width() - init_page_btn_w, this->height() / 2 - init_page_btn_h / 2,
                           init_page_btn_w,
                           init_page_btn_h);
    // todo 应该有一些根据界面大小 控件resize的逻辑 不然单纯靠spacer做 界面布局比较丑
    // todo 主程序按钮

    // todo 任务进度条大小
    //        ui->processWidget->setOutterBarWidth(this->width()/20);
    //        ui->processWidget->setInnerBarWidth(this->width()/20);
    //        updateGeometry();
    //        qDebug("resize");
  }

  //    QWidget::resizeEvent(event);
}

void MainWindow::keyPressEvent(QKeyEvent *event) {
  // 一些主界面的快捷键
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
  // rcl comm 负责处理业务 所有ros相关的接口调用应该由它实现
  rclcomm_ = new SytRclComm();
}

void MainWindow::initWidget() {
  // 用于捕获鼠标移动事件
  // 注意：mainwindow及其之类都要设置mouse track，不然不生效
  setMouseTracking(true);
  ui->centralwidget->setMouseTracking(true);

  // 隐藏默认标题栏
  this->setWindowFlags(Qt::FramelessWindowHint);

  // icon
  this->setWindowIcon(QIcon(":m_logo/logo/bg_logo.png"));

  // title logo
  auto tit_logo = QPixmap(":m_logo/logo/logo2.png");
  tit_logo      = tit_logo.scaled(ui->sytLogoLabel->width(), ui->sytLogoLabel->height(),
                                  Qt::AspectRatioMode::KeepAspectRatio, Qt::TransformationMode::SmoothTransformation);
  ui->sytLogoLabel->setPixmap(tit_logo);

  // 移动到中心
  QScreen *desktop = QApplication::screenAt(QCursor::pos());
  QRect rect       = desktop->availableGeometry();
  move(rect.left() + (rect.width() - width()) / 2, (rect.height() - height()) / 2);

  int btn_size = 40;
  m_menuBtn_   = new WinMenuButton(this);
  m_menuBtn_->setFixedSize(btn_size, btn_size);
  m_hideBtn_ = new WinMinButton(this);
  m_hideBtn_->setFixedSize(btn_size, btn_size);
  m_maxBtn_ = new WinMaxButton(this);
  m_maxBtn_->setFixedSize(btn_size, btn_size);
  m_closeBtn_ = new WinCloseButton(this);
  m_closeBtn_->setFixedSize(btn_size, btn_size);
  ui->mainLayout->addWidget(m_menuBtn_);
  ui->mainLayout->addWidget(m_hideBtn_);
  ui->mainLayout->addWidget(m_maxBtn_);
  ui->mainLayout->addWidget(m_closeBtn_);

  // 为title widget设置上下文菜单
  ui->sytMainTitleWidget->setContextMenuPolicy(Qt::CustomContextMenu);
  m_titleMenu_ = new QMenu(this);
  minAct_      = new QAction(this);
  maxAct_      = new QAction(this);
  fullAct_     = new QAction(this);
  closeAct_    = new QAction(this);
  minAct_->setText("最小化");
  maxAct_->setText("最大化");
  fullAct_->setText("全屏窗口化");
  closeAct_->setText("关闭");
  m_titleMenu_->addAction(minAct_);
  m_titleMenu_->addAction(maxAct_);
  m_titleMenu_->addAction(fullAct_);
  m_titleMenu_->addAction(closeAct_);

  // menu btn上下文
  m_menu_    = new QMenu(this);
  updateAct_ = new QAction(this);
  helpAct_   = new QAction(this);
  aboutAct_  = new QAction(this);
  updateAct_->setText("检查更新");
  updateAct_->setIcon(QIcon(":m_icon/icon/update.png"));
  helpAct_->setText("帮助文档");
  helpAct_->setIcon(QIcon(":m_icon/icon/help.png"));
  aboutAct_->setText("关于速英");
  aboutAct_->setIcon(QIcon(":m_icon/icon/about.png"));
  m_menu_->addAction(updateAct_);
  m_menu_->addAction(helpAct_);
  m_menu_->addAction(aboutAct_);
  m_menuBtn_->setMenu(m_menu_);

  // tool bar btn
  ui->developer_mode_btn->setIcon(QIcon(":m_icon/icon/dev-mode.png")); // 开发者模式按钮
  ui->developer_mode_btn->setToolTip(QString("开发者调试模式"));

  ui->head_eye_calibration_btn->setIcon(QIcon(":m_icon/icon/handeye.png")); // 手眼标定模式按钮
  ui->head_eye_calibration_btn->setToolTip(QString("机器人眼手标定"));

  ui->create_style_btn->setIcon(QIcon(":m_icon/icon/shirt-line.png")); // 创建衣服样式按钮
  ui->create_style_btn->setToolTip(QString("创建衣服样式"));

  ui->lock_screen_btn->setIcon(QIcon(":m_icon/icon/lock.png")); // 屏幕上锁/解锁按钮
  ui->lock_screen_btn->setToolTip(QString("锁屏"));

  ui->help_btn->setIcon(QIcon(":m_icon/icon/help.png"));
  ui->help_btn->setToolTip(QString("帮助说明"));

  // main tab btn
  ui->reset_btn->setParentEnabled(true);
  ui->reset_btn->setForeEnabled(false);
  ui->reset_btn->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");

  ui->start_btn->setParentEnabled(true);
  ui->start_btn->setForeEnabled(false);
  ui->start_btn->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");

  ui->stop_btn->setParentEnabled(true);
  ui->stop_btn->setForeEnabled(false);
  ui->stop_btn->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");

  ui->choose_style_btn->setParentEnabled(true);
  ui->choose_style_btn->setForeEnabled(false);
  ui->choose_style_btn->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");

  // log view btn
  ui->sytFilterPushButton->setIcon(QIcon(":m_icon/icon/filter-records.png"));
  ui->sytFilterPushButton->setToolTip(QString("日志过滤筛选"));

  ui->sytClearPushButton->setIcon(QIcon(":m_icon/icon/clear.png"));
  ui->sytClearPushButton->setToolTip(QString("日志清除"));

  // 添加左右翻页按钮
  int init_page_btn_w = this->width() / 50;
  int init_page_btn_h = this->height() / 9;

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
  next_btn_->setGeometry(this->width() - init_page_btn_w, this->height() / 2 - init_page_btn_h / 2, init_page_btn_w, init_page_btn_h);

  // 可视化的两个按钮
  ui->loadClothVisableBtn->setIcon(QIcon(":m_icon/icon/unvisable.png"));
  ui->loadClothVisableBtn->setText("隐藏");
  ui->compositeClothVisableBtn->setIcon(QIcon(":m_icon/icon/unvisable.png"));
  ui->compositeClothVisableBtn->setText("隐藏");

  // 等待动画初始化
  waiting_spinner_widget_ = new WaitingSpinnerWidget(this);

  // todo 初始状态下，开始和停止无法使用
  this->btnControl({ui->reset_btn}, {ui->start_btn, ui->stop_btn});

  // todo 初始状态下，亮灯
  this->setMutuallyLight(YELLOW);
  ui->msg_widget->setToolTip("系统暂停");

  // 初始状态下主界面显示的任务进度条
  ui->processWidget->setValue(100);

  // 移动至末尾
  ui->moveToEndBtn->setIcon(QIcon(":m_icon/icon/end.png"));
  ui->moveToEndBtn->setToolTip("移至日志末尾");
  // 日志只读
  ui->sytPlainTextEdit->setReadOnly(true);

  // todo 测试进度条，根据逻辑做对应的改变
  test_timer = new QTimer(this);
  test_timer->setInterval(500);
  connect(test_timer, &QTimer::timeout, [=] {
    value += 1;
    if (value == 100) {
      // todo
      //            ui->processWidget->setValue(value);
      value = 0;
    }
    ui->processWidget->setValue(value);
    // todo
    //        if (value == 100) {
    //            test_timer->stop();
    //            emit processSuccessful();
    //            return;
    //        }
  });

  // 样式树形列表
  ui->cloth_style_tree_widget->header()->resizeSection(0, 200);
  ui->cloth_style_tree_widget->setAlternatingRowColors(true);
  ui->cloth_style_tree_widget->setAnimated(true);
  ui->cloth_style_tree_widget->setUniformRowHeights(true);
  ui->cloth_style_tree_widget->setHeaderLabels(QStringList() << "属性"
                                                             << "值");

  // 设置样式line edit只读
  ui->choose_style_line_edit->setReadOnly(true);

  // 事件过滤
  ui->sytMainTitleWidget->installEventFilter(this);
}

void MainWindow::settingConnection() {
  // main window右上角按钮 放大 缩小 菜单等
  connect(m_closeBtn_, &WinCloseButton::clicked, this, &MainWindow::close);
  connect(m_hideBtn_, &WinMaxButton::clicked, this, &MainWindow::showMinimized);
  connect(m_maxBtn_, &WinMaxButton::clicked, this, &MainWindow::slotMaxBtnClicked);
  connect(m_menuBtn_, &WinMenuButton::toggled, [=] {
    m_menu_->show();
    m_menu_->exec();
  });

  // main window工具栏按钮
  connect(ui->developer_mode_btn, &QPushButton::clicked, this, &MainWindow::slotShowDevLoginWindow);
  connect(ui->head_eye_calibration_btn, &QPushButton::clicked, this, &MainWindow::slotStartHeadEyeWindow);
  connect(ui->create_style_btn, &QPushButton::clicked, this, &MainWindow::slotStartClothStyleWindow);
  connect(ui->lock_screen_btn, &QPushButton::clicked, this, &MainWindow::slotLockScreen);
  // todo 帮助未实现，应该是个文档弹出的dialog
  connect(ui->help_btn, &QPushButton::clicked, [=] {
    // todo test
    showMessageBox(this, ERROR, "干巴爹弟兄们😆", 1, {"返回"});
    return;
  });

  // main window 日志栏的按钮
  // todo log filter btn 日志过滤
  connect(ui->sytFilterPushButton, &QPushButton::clicked, [=] {
    // todo
    showMessageBox(this, ERROR, "没做,搞它😵", 1, {"返回"});
  });
  // 日志清除
  connect(ui->sytClearPushButton, &QPushButton::clicked, [=] { ui->sytPlainTextEdit->clear(); });
  // 日志定位到最后一行 继续滚动
  connect(ui->moveToEndBtn, &QPushButton::clicked, [=] {
    QTextCursor cursor = ui->sytPlainTextEdit->textCursor();
    cursor.movePosition(QTextCursor::End);
    ui->sytPlainTextEdit->setTextCursor(cursor);
    // 自动滚动到末尾
    QScrollBar *scrollBar = ui->sytPlainTextEdit->verticalScrollBar();
    scrollBar->setValue(scrollBar->maximum());
  });

  // action 相关
  connect(minAct_, &QAction::triggered, this, &MainWindow::showMinimized);
  connect(maxAct_, &QAction::triggered, this, &MainWindow::slotMaxBtnClicked);
  connect(fullAct_, &QAction::triggered, this, &MainWindow::showFullScreen);
  connect(closeAct_, &QAction::triggered, this, &MainWindow::close);
  connect(helpAct_, &QAction::triggered, this, [=] { ui->help_btn->clicked(true); });
  connect(updateAct_, &QAction::triggered, this, &MainWindow::triggeredOTAUpdate);
  // todo 关于sewing action，应该是个模态dialog，期望他能跳转到sewing的官网等等
  connect(aboutAct_, &QAction::triggered, this, [=] {
    showMessageBox(this, ERROR, "耗子尾汁🙃", 1, {"返回"});
    ;
    return;
  });

  // main window的 翻页按钮
  connect(prev_btn, &QPushButton::clicked, this, &MainWindow::slotPrevPage);
  connect(next_btn_, &QPushButton::clicked, this, &MainWindow::slotNextPage);

  // 主界面接界面 用于交互的3个程序按钮 开始停止复位
  connect(ui->reset_btn, &QPushButton::clicked, this, &MainWindow::resetBtnClicked);
  connect(ui->start_btn, &QPushButton::clicked, this, &MainWindow::startBtnClicked);
  connect(ui->stop_btn, &QPushButton::clicked, this, &MainWindow::stopBtnClicked);

  // 可视化两个按钮
  connect(ui->loadClothVisableBtn, &QPushButton::clicked, [=] {
    is_load_cloth_on_ = !is_load_cloth_on_;
    if (is_load_cloth_on_) {
      qDebug("打开 上料视觉显示");
      ui->loadClothVisableBtn->setIcon(QIcon(":m_icon/icon/unvisable.png"));
      ui->loadClothVisableBtn->setText("隐藏");
    } else {
      qDebug("关闭 上料视觉显示");
      ui->loadClothVisableBtn->setIcon(QIcon(":m_icon/icon/visable.png"));
      ui->loadClothVisableBtn->setText("显示");
      ui->leftLeftVisualLabel->clear();
      ui->leftRightVisualLabel->clear();
      ui->rightLeftVisualLabel->clear();
      ui->rightRightVisualLabel->clear();
      ui->leftLeftVisualLabel->setText("NO IMAGE");
      ui->leftRightVisualLabel->setText("NO IMAGE");
      ui->rightLeftVisualLabel->setText("NO IMAGE");
      ui->rightRightVisualLabel->setText("NO IMAGE");
    }
  });
  connect(ui->compositeClothVisableBtn, &QPushButton::clicked, [=] {
    is_comp_cloth_on_ = !is_comp_cloth_on_;
    if (is_comp_cloth_on_) {
      qDebug("打开 合片视觉显示");
      ui->compositeClothVisableBtn->setIcon(QIcon(":m_icon/icon/unvisable.png"));
      ui->compositeClothVisableBtn->setText("隐藏");
    } else {
      qDebug("关闭 合片视觉显示");
      ui->compositeClothVisableBtn->setIcon(QIcon(":m_icon/icon/visable.png"));
      ui->compositeClothVisableBtn->setText("显示");
      ui->leftCompLabel->clear();
      ui->rightCompLabel->clear();
      ui->leftCompLabel->setText("NO IMAGE");
      ui->rightCompLabel->setText("NO IMAGE");
    }
  });

  // 一些节点相关的报错槽
  connect(rclcomm_, &SytRclComm::errorNodeMsgSign, this, &MainWindow::errorNodeMsgSlot);

  // ota停止
  connect(rclcomm_, &SytRclComm::waitUpdateResultSuccess, this, &MainWindow::otaResultShow, Qt::QueuedConnection);

  // ota安装
  connect(rclcomm_, &SytRclComm::installRes, this, &MainWindow::otaInstallSuccess, Qt::QueuedConnection);

  // head eye dialog 信号槽
  connect(this, &MainWindow::signHeadEyeWindowShow, [=] {
    auto head_eye_dialog = new HeadEyeDialog(this);
    connect(head_eye_dialog, &HeadEyeDialog::signCompStart, this, &MainWindow::slotCompCalibStart, Qt::ConnectionType::QueuedConnection);
    connect(head_eye_dialog, &HeadEyeDialog::signSewingStart, this, &MainWindow::slotSewingCalibStart, Qt::ConnectionType::QueuedConnection);
    head_eye_dialog->show();
    head_eye_dialog->setAttribute(Qt::WA_DeleteOnClose);
  });

  // 选择样式信号槽
  connect(
      ui->choose_style_btn, &QPushButton::clicked, this, [=] {
        auto choose_style_dialog = new ChooseStyleDialog(this);
        connect(choose_style_dialog, &ChooseStyleDialog::signChoseStyle, this, &MainWindow::slotChoseStyleFile);

        choose_style_dialog->show();
        choose_style_dialog->setAttribute(Qt::WA_DeleteOnClose);
      },
      Qt::ConnectionType::UniqueConnection);

  // 创建样式信号槽
  connect(this, &MainWindow::signClothStyleWindowShow, [=] {
    auto cloth_style_dialog = new ClothStyleDialog(this);

    void (ClothStyleDialog::*create_from_cad_signal)(ClothStyleDialog *parent) = &ClothStyleDialog::signCreateFromCAD;
    void (MainWindow::*create_from_cad_slot)(ClothStyleDialog *parent)         = &MainWindow::slotCreateFromCAD;
    connect(cloth_style_dialog, create_from_cad_signal, this, create_from_cad_slot, Qt::ConnectionType::QueuedConnection);

    void (ClothStyleDialog::*auto_create_style_signal)(ClothStyleDialog *parent) = &ClothStyleDialog::signAutoCreateStyle;
    void (MainWindow::*auto_create_style_slot)(ClothStyleDialog *parent)         = &MainWindow::slotAutoCreateStyle;
    connect(cloth_style_dialog, auto_create_style_signal, this, auto_create_style_slot, Qt::ConnectionType::QueuedConnection);

    void (ClothStyleDialog::*manual_input_param_signal)(ClothStyleDialog *parent) = &ClothStyleDialog::signManualInputParam;
    void (MainWindow::*manual_input_param_slot)(ClothStyleDialog *parent)         = &MainWindow::slotManualInputParam;
    connect(cloth_style_dialog, manual_input_param_signal, this, manual_input_param_slot, Qt::ConnectionType::QueuedConnection);

    void (ClothStyleDialog::*create_from_source_signal)(ClothStyleDialog *parent) = &ClothStyleDialog::signCreateFromSource;
    void (MainWindow::*create_from_source_slot)(ClothStyleDialog *parent)         = &MainWindow::slotCreateFromSource;
    connect(cloth_style_dialog, create_from_source_signal, this, create_from_source_slot, Qt::ConnectionType::QueuedConnection);

    cloth_style_dialog->show();
    cloth_style_dialog->setAttribute(Qt::WA_DeleteOnClose);
  });

  // 标定相关
  connect(rclcomm_, &SytRclComm::compCalibRes, this, &MainWindow::slotCompCalibRes);
  connect(rclcomm_, &SytRclComm::sewingCalibRes, this, &MainWindow::slotSewingCalibRes);

  // ros2 rosout回调消息的槽函数
  connect(rclcomm_, &SytRclComm::signLogPub, this, &MainWindow::slotLogShow, Qt::ConnectionType::QueuedConnection);

  // 上料机可视化相关槽函数
  connect(rclcomm_, &SytRclComm::visualLoadClothRes, this, &MainWindow::slotVisualLoadCloth);

  // todo 合片机可视化相关槽函数

  // todo 任务完成后按钮的逻辑
  connect(this, &MainWindow::processSuccessful, [=] {
    this->btnControl({ui->reset_btn}, {ui->start_btn, ui->stop_btn});
    showMessageBox(this, SUCCESS, "当前批次任务完成,请手动完成上料后继续开始", 1, {"确认"});
  });

  // 状态label显示
  connect(this, &MainWindow::signUpdateLabelState, [=](QString text) {
    ui->stateLabel->setText(text);
  });
}

void MainWindow::setMutuallyLight(LIGHT_COLOR c) {
  std::map<LIGHT_COLOR, std::string> m;
  m[RED]    = "background-color: rgb(238, 99, 99);border: 3px solid black;border-radius: 15px;";
  m[YELLOW] = "background-color: rgb(255 ,215, 0);border: 3px solid black;border-radius: 15px;";
  m[GREEN]  = "background-color: rgb(0 ,255, 150);border: 3px solid black;border-radius: 15px;";
  m[GRAY]   = "background-color: gray;border: 3px solid black;border-radius: 15px;";
  switch (c) {
  case RED:
    ui->red_label->setStyleSheet(m[RED].data());
    ui->yellow_label->setStyleSheet(m[GRAY].data());
    ui->green_label->setStyleSheet(m[GRAY].data());
    break;
  case YELLOW:
    ui->red_label->setStyleSheet(m[GRAY].data());
    ui->yellow_label->setStyleSheet(m[YELLOW].data());
    ui->green_label->setStyleSheet(m[GRAY].data());
    break;
  case GREEN:
    ui->red_label->setStyleSheet(m[GRAY].data());
    ui->yellow_label->setStyleSheet(m[GRAY].data());
    ui->green_label->setStyleSheet(m[GREEN].data());
    break;
  default:
    break;
  }
}

void MainWindow::deleteAll() {
  if (this->future_.isRunning()) {
    this->future_.cancel();
    this->future_.waitForFinished();
  }
  delete rclcomm_;
  delete ui;
}

void MainWindow::initOther() {
  // 如未存在,创建所有配置文件
  checkConfigsExist();
}

void MainWindow::btnControl(std::vector<QPushButton *> enables, std::vector<QPushButton *> unables) {
  for (auto i : enables) {
    i->setEnabled(true);
    i->setStyleSheet("");
  }
  for (auto i : unables) {
    i->setEnabled(false);
    i->setStyleSheet("color: gray;");
  }
}

void MainWindow::slotMaxBtnClicked() {
  if (this->isMaximized()) {
    this->showNormal();
  } else {
    this->showMaximized();
  }
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

void MainWindow::resetBtnClicked() {
  emit signUpdateLabelState("重置中");

  bool res = isFastClick(ui->reset_btn, 1000);
  if (!res) {
    return;
  }
  // todo 重置完成后可用
  ui->start_btn->setEnabled(true);
  ui->start_btn->setStyleSheet("");
  ui->stop_btn->setEnabled(true);
  ui->stop_btn->setStyleSheet("");
  this->setMutuallyLight(YELLOW);
  // todo 进度条清0
  value = 0;
  ui->processWidget->setValue(value);
  // todo
  future_ = QtConcurrent::run([=] {
    rclcomm_->resetCmd();
  });

  emit signUpdateLabelState("重置完成");
}

void MainWindow::startBtnClicked() {
  bool res = isFastClick(ui->start_btn, 1000);
  if (!res) {
    return;
  }

  this->btnControl({ui->stop_btn}, {ui->start_btn, ui->reset_btn});

  test_timer->start();
  setMutuallyLight(GREEN);

  emit signUpdateLabelState("运行中");

  // 发布开始指令
  future_ = QtConcurrent::run([=] {
    rclcomm_->startCmd();
  });
}

void MainWindow::stopBtnClicked() {
  bool res = isFastClick(ui->stop_btn, 1000);
  if (!res) {
    return;
  }
  qDebug("点击停止按钮");

  // todo
  this->btnControl({ui->reset_btn, ui->start_btn}, {ui->stop_btn});

  // todo test
  ui->msg_widget->setToolTip("系统异常");
  test_timer->stop();

  this->setMutuallyLight(YELLOW);

  emit signUpdateLabelState("停止中");
}

void MainWindow::errorNodeMsgSlot(QString msg) {
  showMessageBox(this, STATE::ERROR, msg, 1, {"退出"});
}

void MainWindow::triggeredOTAUpdate() {
  qDebug("update");
  waiting_spinner_widget_->start();
  future_ = QtConcurrent::run([=] {
    rclcomm_->otaUpdate();
  });
}

void MainWindow::otaResultShow(bool res, QString msg) {
  waiting_spinner_widget_->stop();
  if (res) {
    // todo show ota res
    auto res = showMessageBox(this, STATE::SUCCESS, "检测到远端存在新安装包,请选择是否升级", 2,
                              {"一键升级", "取消升级"});
    if (res == 0) {
      future_ = QtConcurrent::run([=] {
        rclcomm_->otaDownload();
      });
      //            future2.waitForFinished();

    } else if (res == 1) {
      qDebug("取消更新");
      return;
    }

    // todo 完成后的
    auto ota_dialog = new OtaUpdateDialog(this);
    connect(rclcomm_, &SytRclComm::processZero, ota_dialog, &OtaUpdateDialog::clearProcessValue, Qt::ConnectionType::QueuedConnection);
    connect(rclcomm_, &SytRclComm::updateProcess, ota_dialog, &OtaUpdateDialog::updateProcessValue, Qt::ConnectionType::QueuedConnection);
    connect(rclcomm_, &SytRclComm::downloadRes, ota_dialog, &OtaUpdateDialog::getDownloadRes);
    ota_dialog->show();
    auto res_ = ota_dialog->exec();

    if (res_ == QDialog::Rejected) {
      showMessageBox(this, STATE::WARN, "取消升级", 1, {"退出"});
      return;
    } else if (res_ == 9) {
      showMessageBox(this, STATE::SUCCESS, "下载完成,请点击以下按钮进行软件安装", 1, {"安装"});
      // todo call server
      waiting_spinner_widget_->start();
      future_ = QtConcurrent::run([=] {
        rclcomm_->otaInstall();
      });
    } else if (res_ == 10) {
      showMessageBox(this, STATE::ERROR, "升级失败,请检查网络是否异常", 1, {"退出"});
      return;
    }

    delete ota_dialog;

  } else {
    showMessageBox(this, STATE::ERROR, msg, 1, {"退出"});
  }
}

void MainWindow::otaInstallSuccess(bool res, QString msg) {
  waiting_spinner_widget_->stop();
  if (!res) {
    showMessageBox(this, ERROR, msg, 1, {"返回"});
    return;
  }
  showMessageBox(this, SUCCESS, msg, 1, {"重启"});
  this->deleteAll();
  exit(0);
}

void MainWindow::slotVisualLoadCloth(int machine_id, int cam_id, QImage image) {
  if (!is_load_cloth_on_) {
    return;
  }
  auto pix = QPixmap::fromImage(
      image.scaled(ui->leftLeftVisualLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
  if (machine_id == 0) {
    if (cam_id == 0) {
      ui->leftLeftVisualLabel->clear();
      ui->leftLeftVisualLabel->setPixmap(pix);
      ui->leftLeftVisualLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    } else if (cam_id == 1) {
      ui->leftRightVisualLabel->clear();
      ui->leftRightVisualLabel->setPixmap(pix);
      ui->leftRightVisualLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    } else {
      return;
    }

  } else if (machine_id == 1) {
    if (cam_id == 0) {
      ui->rightLeftVisualLabel->clear();
      ui->rightLeftVisualLabel->setPixmap(pix);
      ui->rightLeftVisualLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    } else if (cam_id == 1) {
      ui->rightRightVisualLabel->clear();
      ui->rightRightVisualLabel->setPixmap(pix);
      ui->rightRightVisualLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    } else {
      return;
    }
  } else {
    return;
  }
}

////////////////////////// 工具栏函数 //////////////////////////
void MainWindow::slotShowDevLoginWindow() {
  dev_login_window_ = new DevLoginWindow(this);
  connect(dev_login_window_, &DevLoginWindow::signDevMode, this, &MainWindow::slotDevWindow);
  dev_login_window_->show();
}

void MainWindow::slotLockScreen() {
  auto lock_dialog = new LockDialog(this);
  lock_dialog->show();
  lock_dialog->setAttribute(Qt::WA_DeleteOnClose);
}

void MainWindow::slotStartHeadEyeWindow() {
  qDebug("启动眼手标定");

  QString tip =
      "<html>"
      "<head/><b>启动机器人眼手标定</b>\n<body>"
      "<b>注意:</b>"
      "<p>1.标定过程中,<font color=\"red\"><b>禁止靠近机台</b></font>;\n</p>"
      "<p>2.请确认机器人处在一个<font color=\"red\"><b>良好的位置</b></font>,避免启动时发生碰撞;\n</p>"
      "<p>3.请时刻<font color=\"red\"><b>保持专注</b></font>,并<font color=\"red\"><b>手持急停开关</b></font>,务必保证危险时刻能按下;\n</p>"
      "<p>4.确保机器人当前为<font color=\"red\"><b>停止状态</b></font>;\n</p>"
      "</body></html>";

  auto res = showMessageBox(this, WARN, tip, 2, {"确认", "返回"});

  switch (res) {
  case 0:
    emit signHeadEyeWindowShow();
    return;
  case 1:
    return;
  default:
    return;
  }
}

void MainWindow::slotStartClothStyleWindow() {
  emit signClothStyleWindowShow();
}

void MainWindow::slotDevWindow() {
  dev_login_window_->deleteLater();
  dev_login_window_ = nullptr;
  // dev main window
  //    auto dev_window = new DevWindow(this);
  //    dev_window->setAttribute(Qt::WA_DeleteOnClose);
  //    connect(dev_window, &DevWindow::closeDevWindow, [=] { this->show(); });
  //    connect(dev_window, &DevWindow::destroyed, [=] { dev_window->deleteLater(); });
  //    dev_window->show();
  auto dev_sel_dialog = new DevSelectDialog(this);
  dev_sel_dialog->show();
  dev_sel_dialog->setAttribute(Qt::WA_DeleteOnClose);
}

////////////////////////// 标定槽函数 //////////////////////////
void MainWindow::slotCompCalibRes(bool f) {
  waiting_spinner_widget_->stop();
  if (f) {
    showMessageBox(this, SUCCESS, "合片台标定成功", 1, {"退出"});
    return;
  } else {
    showMessageBox(this, SUCCESS, "合片台标定失败,请联系相关人员", 1, {"退出"});
    return;
  }
}

void MainWindow::slotSewingCalibRes(bool f) {
  waiting_spinner_widget_->stop();
  if (f) {
    showMessageBox(this, SUCCESS, "缝纫台标定成功", 1, {"退出"});
    return;
  } else {
    showMessageBox(this, SUCCESS, "缝纫台标定失败,请联系相关人员", 1, {"退出"});
    return;
  }
}

void MainWindow::slotCompCalibStart() {
  waiting_spinner_widget_->start();
  future_ = QtConcurrent::run([=] {
    rclcomm_->compCalib();
  });
}

void MainWindow::slotSewingCalibStart() {
  waiting_spinner_widget_->start();
  future_ = QtConcurrent::run([=] {
    rclcomm_->sewingCalib();
  });
}

////////////////////////// 显示log槽函数 //////////////////////////
void MainWindow::slotLogShow(QString time, QString level, QString location, QString func, QString msg) {
  Q_UNUSED(func)
  QString htmlText;
  if (level == "DEBUG") {
    htmlText = QString("<span style=\"background-color: green; color: white; font-weight: bold;\">【 %1 】 【 %2 】 【 %3 】：  %4\n</span>").arg(level).arg(time).arg(location).arg(msg);
  } else if (level == "INFO") {
    htmlText = QString("<span style=\"background-color: white; color: black; font-weight: bold;\">【 %1 】 【 %2 】 【 %3 】：  %4\n</span>").arg(level).arg(time).arg(location).arg(msg);
  } else if (level == "WARN") {
    htmlText = QString("<span style=\"background-color: orange; color: white; font-weight: bold;\">【 %1 】 【 %2 】 【 %3 】：  %4\n</span>").arg(level).arg(time).arg(location).arg(msg);
  } else if (level == "ERROR") {
    htmlText = QString("<span style=\"background-color: darkred; color: white; font-weight: bold;\">【 %1 】 【 %2 】 【 %3 】：  %4\n</span>").arg(level).arg(time).arg(location).arg(msg);
  } else if (level == "FATAL") {
    htmlText = QString("<span style=\"background-color: red; color: white; font-weight: bold;\">【 %1 】 【 %2 】 【 %3 】：  %4\n</span>").arg(level).arg(time).arg(location).arg(msg);
  } else {
    qDebug("敲尼玛？");
    return;
  }
  ui->sytPlainTextEdit->appendHtml(htmlText);
}

////////////////////////// 选择设置样式槽函数 //////////////////////////
void MainWindow::slotChoseStyleFile(QString prefix, QString file_name) {
  style_file_prefix_ = prefix;
  style_file_name_   = file_name;
  ui->choose_style_line_edit->setText(prefix + QDir::separator() + file_name);

  // 开始设置并获取当前样式
  waiting_spinner_widget_->start();
  bool success = rclcomm_->setCurrentStyle(prefix, file_name);
  if (!success) {
    showMessageBox(this, WARN, "设置当前样式错误", 1, {"确认"});
  }

  success = rclcomm_->getClothStyle(prefix, file_name, cloth_style_front_, cloth_style_back_);
  if (!success) {
    showMessageBox(this, WARN, "获取当前样式错误", 1, {"确认"});
  }

  ui->cloth_style_tree_widget->clear();

  // 设置信息到treewidget中
  QTreeWidgetItem *front_item = new QTreeWidgetItem(QStringList() << "前片");
  QTreeWidgetItem *back_item  = new QTreeWidgetItem(QStringList() << "后片");

  ui->cloth_style_tree_widget->addTopLevelItem(front_item);
  ui->cloth_style_tree_widget->addTopLevelItem(back_item);

  auto fillTreeWidget = [&](QTreeWidgetItem *top_item, syt_msgs::msg::ClothStyle cloth_style) {
    top_item->addChild(new QTreeWidgetItem(QStringList() << "衣长" << QString::number(cloth_style.cloth_length)));
    top_item->addChild(new QTreeWidgetItem(QStringList() << "下摆长" << QString::number(cloth_style.bottom_length)));
    top_item->addChild(new QTreeWidgetItem(QStringList() << "腋下间距" << QString::number(cloth_style.oxter_length)));
    top_item->addChild(new QTreeWidgetItem(QStringList() << "肩缝长" << QString::number(cloth_style.shoulder_length)));
    top_item->addChild(new QTreeWidgetItem(QStringList() << "侧缝长" << QString::number(cloth_style.side_length)));
    top_item->addChild(new QTreeWidgetItem(QStringList() << "有无印花" << (cloth_style.have_printings ? QString("有") : QString("无"))));

    // 设置颜色预览
    QTreeWidgetItem *color_item = new QTreeWidgetItem(QStringList() << "颜色");
    top_item->addChild(color_item);
    ui->cloth_style_tree_widget->setItemWidget(color_item, 1, new ShowColorWidget(QString::number(cloth_style.cloth_color, 16)));

    top_item->addChild(new QTreeWidgetItem(QStringList() << "裁片克数" << QString::number(cloth_style.cloth_weight)));
    top_item->addChild(new QTreeWidgetItem(QStringList() << "弹性" << id_style_map.value(cloth_style.elasticity_level)));
    top_item->addChild(new QTreeWidgetItem(QStringList() << "厚度" << id_style_map.value(cloth_style.thickness_level)));
    top_item->addChild(new QTreeWidgetItem(QStringList() << "尺码" << id_style_map.value(cloth_style.cloth_size)));
    top_item->addChild(new QTreeWidgetItem(QStringList() << "光泽度" << id_style_map.value(cloth_style.glossiness_level)));
  };
  fillTreeWidget(front_item, cloth_style_front_);
  fillTreeWidget(back_item, cloth_style_back_);
  ui->cloth_style_tree_widget->expandAll(); // 展开
  waiting_spinner_widget_->stop();
}

////////////////////////// 创建衣服样式槽函数 //////////////////////////
// 从CAD创建
void MainWindow::slotCreateFromCAD(ClothStyleDialog *parent) {
  CreateFromCADWizard *create_from_cad_wizard = new CreateFromCADWizard(parent);
  create_from_cad_wizard->show();
  create_from_cad_wizard->setAttribute(Qt::WA_DeleteOnClose);
}

// 自动创建
void MainWindow::slotAutoCreateStyle(ClothStyleDialog *parent) {
  AutoCreateStyleWizard *auto_create_style_wizard = new AutoCreateStyleWizard(parent);
  connect(auto_create_style_wizard, &AutoCreateStyleWizard::signMoveHand, this, &MainWindow::slotMoveHandByAutoCreateStyle);
  connect(rclcomm_, &SytRclComm::signComposeMachineMoveHandFinish, auto_create_style_wizard, &AutoCreateStyleWizard::slotMoveHandResult);

  connect(auto_create_style_wizard, &AutoCreateStyleWizard::signDetectCloth, this, &MainWindow::slotDetectClothByAutoCreateStyle);
  connect(rclcomm_, &SytRclComm::signComposeMachineDetectClothFinish, auto_create_style_wizard, &AutoCreateStyleWizard::slotDetectClothResult);

  connect(auto_create_style_wizard, &AutoCreateStyleWizard::signCreateStyle, this, &MainWindow::slotCreateStyleByAutoCreateStyle);
  connect(rclcomm_, &SytRclComm::signCreateStyleFinish, auto_create_style_wizard, &AutoCreateStyleWizard::slotCreateStyleResult);

  connect(auto_create_style_wizard, &AutoCreateStyleWizard::signRenameClothStyle, this, &MainWindow::slotRenameClothStyleByAutoCreateStyle);
  connect(rclcomm_, &SytRclComm::signRenameClothStyleFinish, auto_create_style_wizard, &AutoCreateStyleWizard::slotRenameClothStyleResult);

  auto_create_style_wizard->show();
  auto_create_style_wizard->setAttribute(Qt::WA_DeleteOnClose);
}

// 手动输入创建
void MainWindow::slotManualInputParam(ClothStyleDialog *parent) {
  ManualInputParamWizard *manual_input_param_wizard = new ManualInputParamWizard(parent);
  manual_input_param_wizard->show();
  manual_input_param_wizard->setAttribute(Qt::WA_DeleteOnClose);
}

// 从已有文件创建
void MainWindow::slotCreateFromSource(ClothStyleDialog *parent) {
  CreateFromSourceWizard *create_from_source_wizard = new CreateFromSourceWizard(parent);
  create_from_source_wizard->show();
  create_from_source_wizard->setAttribute(Qt::WA_DeleteOnClose);
}

void MainWindow::slotMoveHandByAutoCreateStyle() {
  future_ = QtConcurrent::run([=] {
    rclcomm_->composeMachineMoveHand(0, 900, 0, 0);
  });
}

void MainWindow::slotDetectClothByAutoCreateStyle(int cloth_type) {
  future_ = QtConcurrent::run([=] {
    rclcomm_->composeMachineDetectCloth(1, cloth_type);
  });
}

void MainWindow::slotCreateStyleByAutoCreateStyle(syt_msgs::msg::ClothStyle cloth_style_front, syt_msgs::msg::ClothStyle cloth_style_back) {
  future_ = QtConcurrent::run([=] {
    rclcomm_->createStyle(cloth_style_front, cloth_style_back);
  });
}

void MainWindow::slotRenameClothStyleByAutoCreateStyle(std::string old_name, std::string new_name) {
  future_ = QtConcurrent::run([=] {
    rclcomm_->renameClothStyle(old_name, new_name);
  });
}
