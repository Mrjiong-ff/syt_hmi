#include "syt_hmi/main_window.h"

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
    next_btn_->setGeometry(this->width() - init_page_btn_w, this->height() / 2 - init_page_btn_h / 2, init_page_btn_w, init_page_btn_h);
    // todo 应该有一些根据界面大小 控件resize的逻辑 不然单纯靠spacer做 界面布局比较丑
    // todo 主程序按钮

    // todo 任务进度条大小
    //        ui->processWidget->setOutterBarWidth(this->width()/20);
    //        ui->processWidget->setInnerBarWidth(this->width()/20);
    //        updateGeometry();
    //        qDebug("resize");
  }

  // QWidget::resizeEvent(event);
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

  ui->add_cloth_btn->setParentEnabled(true);
  ui->add_cloth_btn->setForeEnabled(false);
  ui->add_cloth_btn->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");

  ui->change_board_btn->setParentEnabled(true);
  ui->change_board_btn->setForeEnabled(false);
  ui->change_board_btn->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");

  ui->choose_style_btn->setParentEnabled(true);
  ui->choose_style_btn->setForeEnabled(false);
  ui->choose_style_btn->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");

  //////////////////////////////// 日志 ///////////////////////////////////
  // log view btn
  ui->log_filter_tool_btn->setIcon(QIcon(":m_icon/icon/filter-records.png"));
  ui->log_filter_tool_btn->setToolTip(QString("日志过滤筛选"));
  set_log_level_debug_act_ = new QAction("DEBUG", this);
  set_log_level_info_act_  = new QAction("INFO", this);
  set_log_level_warn_act_  = new QAction("WARN", this);
  set_log_level_error_act_ = new QAction("ERROR", this);
  set_log_level_fatal_act_ = new QAction("FATAL", this);
  ui->log_filter_tool_btn->addAction(set_log_level_debug_act_);
  ui->log_filter_tool_btn->addAction(set_log_level_info_act_);
  ui->log_filter_tool_btn->addAction(set_log_level_warn_act_);
  ui->log_filter_tool_btn->addAction(set_log_level_error_act_);
  ui->log_filter_tool_btn->addAction(set_log_level_fatal_act_);

  ui->log_clear_btn->setIcon(QIcon(":m_icon/icon/clear.png"));
  ui->log_clear_btn->setToolTip(QString("日志清除"));
  ui->log_clear_btn->setParentEnabled(true);
  ui->log_clear_btn->setForeEnabled(false);
  ui->log_clear_btn->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");

  // 移动至末尾
  ui->log_end_btn->setIcon(QIcon(":m_icon/icon/end.png"));
  ui->log_end_btn->setToolTip("移至日志末尾");
  ui->log_end_btn->setParentEnabled(true);
  ui->log_end_btn->setForeEnabled(false);
  ui->log_end_btn->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");

  // 日志只读
  ui->log_plain_text_edit->setReadOnly(true);

  //////////////////////////////// 左右翻页 ///////////////////////////////////
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

  // todo 初始状态下按钮状态
  this->btnControl({ui->reset_btn, ui->start_btn, ui->stop_btn, ui->add_cloth_btn, ui->change_board_btn}, {});

  // 初始状态下主界面显示的任务进度条
  // ui->processWidget->setValue(100);

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

  // todo 初始状态下，亮灯
  this->setMutuallyLight(RED);

  // 开发者界面
  developer_widget_ = new DeveloperWidget(this);

  // 时间显示定时器
  time_timer_ = new QTimer(this);
  time_timer_->start(1000);
  QTime time_now = QTime::currentTime();
  ui->current_time_label->setText(QString("时间：%1").arg(time_now.toString()));
}

void MainWindow::settingConnection() {
  connect(ui->stackedWidget, &QStackedWidget::currentChanged, this, [=]() {
    if (ui->stackedWidget->currentWidget() == ui->page1) {
      if (!pix_A_left_.isNull()) {
        ui->rightLeftVisualLabel->clear();
        ui->rightLeftVisualLabel->setPixmap(pix_A_left_);
        ui->rightLeftVisualLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
      }

      if (!pix_A_right_.isNull()) {
        ui->rightRightVisualLabel->clear();
        ui->rightRightVisualLabel->setPixmap(pix_A_right_);
        ui->rightRightVisualLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
      }

      if (!pix_B_left_.isNull()) {
        ui->leftLeftVisualLabel->clear();
        ui->leftLeftVisualLabel->setPixmap(pix_B_left_);
        ui->leftLeftVisualLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
      }

      if (!pix_B_right_.isNull()) {
        ui->leftRightVisualLabel->clear();
        ui->leftRightVisualLabel->setPixmap(pix_B_right_);
        ui->leftRightVisualLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
      }
    }
  });

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
  connect(set_log_level_debug_act_, &QAction::triggered, [=] { log_level_ = LOG_DEBUG; });
  connect(set_log_level_info_act_, &QAction::triggered, [=] { log_level_ = LOG_INFO; });
  connect(set_log_level_warn_act_, &QAction::triggered, [=] { log_level_ = LOG_WARN; });
  connect(set_log_level_error_act_, &QAction::triggered, [=] { log_level_ = LOG_ERROR; });
  connect(set_log_level_fatal_act_, &QAction::triggered, [=] { log_level_ = LOG_FATAL; });

  // 日志清除
  connect(ui->log_clear_btn, &QPushButton::clicked, [=] { ui->log_plain_text_edit->clear(); });

  // 日志定位到最后一行 继续滚动
  connect(ui->log_end_btn, &QPushButton::clicked, [=] {
    QTextCursor cursor = ui->log_plain_text_edit->textCursor();
    cursor.movePosition(QTextCursor::End);
    ui->log_plain_text_edit->setTextCursor(cursor);
    // 自动滚动到末尾
    QScrollBar *scrollBar = ui->log_plain_text_edit->verticalScrollBar();
    scrollBar->setValue(scrollBar->maximum());
    ui->log_plain_text_edit->verticalScrollBar()->show();
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
    showMessageBox(this, ERROR, "帮助", 1, {"返回"});
    return;
  });

  // main window的 翻页按钮
  connect(prev_btn, &QPushButton::clicked, this, &MainWindow::slotPrevPage);
  connect(next_btn_, &QPushButton::clicked, this, &MainWindow::slotNextPage);

  // 主界面接界面 用于交互的3个程序按钮 开始停止复位
  connect(ui->reset_btn, &QPushButton::clicked, this, &MainWindow::resetBtnClicked);
  connect(ui->start_btn, &QPushButton::clicked, this, &MainWindow::startBtnClicked);
  connect(ui->stop_btn, &QPushButton::clicked, this, &MainWindow::stopBtnClicked);
  connect(ui->add_cloth_btn, &QPushButton::clicked, this, &MainWindow::addClothBtnClicked);
  connect(ui->change_board_btn, &QPushButton::clicked, this, &MainWindow::changePlateBtnClicked);

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
    connect(head_eye_dialog, &HeadEyeDialog::signCompStart, this, &MainWindow::slotCompCalibStart);
    connect(head_eye_dialog, &HeadEyeDialog::signSewingStart, this, &MainWindow::slotSewingCalibStart);
    head_eye_dialog->show();
    head_eye_dialog->setAttribute(Qt::WA_DeleteOnClose);
  });

  // 标定相关
  connect(rclcomm_, &SytRclComm::compCalibRes, this, &MainWindow::slotCompCalibRes);
  connect(rclcomm_, &SytRclComm::sewingCalibRes, this, &MainWindow::slotSewingCalibRes);

  // 选择样式信号槽
  connect(ui->choose_style_btn, &QPushButton::clicked, this, &MainWindow::slotChooseStyleFile);

  // 创建样式信号槽
  connect(this, &MainWindow::signClothStyleWindowShow, [=] {
    auto cloth_style_dialog = new ClothStyleDialog(this);

    //// 从CAD创建
    // void (ClothStyleDialog::*create_from_cad_signal)(ClothStyleDialog *parent) = &ClothStyleDialog::signCreateFromCAD;
    // void (MainWindow::*create_from_cad_slot)(ClothStyleDialog *parent)         = &MainWindow::slotCreateFromCAD;
    // connect(cloth_style_dialog, create_from_cad_signal, this, create_from_cad_slot, Qt::ConnectionType::QueuedConnection);

    // 自动创建
    void (ClothStyleDialog::*auto_create_style_signal)(ClothStyleDialog *parent) = &ClothStyleDialog::signAutoCreateStyle;
    void (MainWindow::*auto_create_style_slot)(ClothStyleDialog *parent)         = &MainWindow::slotAutoCreateStyle;
    connect(cloth_style_dialog, auto_create_style_signal, this, auto_create_style_slot, Qt::ConnectionType::QueuedConnection);

    // 手动创建
    void (ClothStyleDialog::*manual_input_param_signal)(ClothStyleDialog *parent) = &ClothStyleDialog::signManualInputParam;
    void (MainWindow::*manual_input_param_slot)(ClothStyleDialog *parent)         = &MainWindow::slotManualInputParam;
    connect(cloth_style_dialog, manual_input_param_signal, this, manual_input_param_slot, Qt::ConnectionType::QueuedConnection);

    //// 从已有文件创建
    // void (ClothStyleDialog::*create_from_source_signal)(ClothStyleDialog *parent) = &ClothStyleDialog::signCreateFromSource;
    // void (MainWindow::*create_from_source_slot)(ClothStyleDialog *parent)         = &MainWindow::slotCreateFromSource;
    // connect(cloth_style_dialog, create_from_source_signal, this, create_from_source_slot, Qt::ConnectionType::QueuedConnection);

    cloth_style_dialog->show();
    cloth_style_dialog->setAttribute(Qt::WA_DeleteOnClose);
  });

  // ros2 rosout回调消息的槽函数
  connect(rclcomm_, &SytRclComm::signLogPub, this, &MainWindow::slotLogShow, Qt::ConnectionType::QueuedConnection);

  // 补料模式结束
  connect(rclcomm_, &SytRclComm::signLoadMachineAddClothFinish, this, &MainWindow::slotAddClothResult);

  // 上料机可视化相关槽函数
  connect(rclcomm_, &SytRclComm::visualLoadClothRes, this, &MainWindow::slotVisualLoadCloth);

  // todo 合片机可视化相关槽函数

  // todo 任务完成后按钮的逻辑
  connect(rclcomm_, &SytRclComm::machineIdle, [=](bool idle) {
    if (idle) {
      this->btnControl({ui->reset_btn, ui->add_cloth_btn, ui->change_board_btn, ui->start_btn, ui->stop_btn}, {});
    }
    // showMessageBox(this, SUCCESS, "当前批次任务完成,请手动完成上料后继续开始", 1, {"确认"});
  });

  // 状态label显示
  connect(this, &MainWindow::signUpdateLabelState, [=](QString text) {
    ui->stateLabel->setText(text);
  });

  // 绑定开发者界面按钮
  bindDeveloperConnection();

  // 动态显示时间
  connect(time_timer_, &QTimer::timeout, ui->current_time_label, [=]() {
    QTime time_now = QTime::currentTime();
    ui->current_time_label->setText(QString("时间：%1").arg(time_now.toString()));
  });
}

void MainWindow::bindDeveloperConnection() {
  // 合片反馈
  qRegisterMetaType<syt_msgs::msg::ComposeMachineState>("syt_msgs::msg::ComposeMachineState");
  connect(rclcomm_, &SytRclComm::updateComposeMachineState, [=](syt_msgs::msg::ComposeMachineState state) {
    developer_widget_->setComposeMachineState(state);
  });

  connect(rclcomm_, &SytRclComm::updateSewingMachineState, [=](syt_msgs::msg::SewingMachineState state) {
    developer_widget_->setSewingMachineState(state);
  });

  // 缝纫反馈

  // 上料机-复位
  connect(developer_widget_, &DeveloperWidget::signLoadMachineReset, [=](int id) {
    QtConcurrent::run([=]() {
      rclcomm_->loadMachineReset(id);
    });
  });

  // 上料机-补料
  connect(developer_widget_, &DeveloperWidget::signLoadMachineAddCloth, [=](int id) {
    QtConcurrent::run([=]() {
      rclcomm_->loadMachineAddCloth(id);
    });
  });

  // 上料机-清台
  connect(developer_widget_, &DeveloperWidget::signLoadMachineClearTable, [=](int id) {
    QtConcurrent::run([=]() {
      rclcomm_->loadMachineClearTable(id);
    });
  });

  // 上料机-裁片尺寸
  connect(developer_widget_, &DeveloperWidget::signLoadMachineClothSize, [=](int id, uint32_t width, uint32_t length) {
    QtConcurrent::run([=]() {
      rclcomm_->loadMachineClothSize(id, width, length);
    });
  });

  // 上料机-裁片尺寸
  connect(developer_widget_, &DeveloperWidget::signLoadMachineLoadDistance, [=](int id, uint32_t distance) {
    QtConcurrent::run([=]() {
      rclcomm_->loadMachineLoadDistance(id, distance);
    });
  });

  // 上料机-上料间隔
  connect(developer_widget_, &DeveloperWidget::signLoadMachineTrayGap, [=](int id, uint32_t height) {
    QtConcurrent::run([=]() {
      rclcomm_->loadMachineTrayGap(id, height);
    });
  });

  // 上料机-上料偏移
  connect(developer_widget_, &DeveloperWidget::signLoadMachineOffset, [=](int id, int offset) {
    QtConcurrent::run([=]() {
      rclcomm_->loadMachineOffset(id, offset);
    });
  });

  // 上料机-抓住裁片
  connect(developer_widget_, &DeveloperWidget::signLoadMachineHoldCloth, [=](int id) {
    QtConcurrent::run([=]() {
      rclcomm_->loadMachineHoldCloth(id);
    });
  });

  // 上料机-上裁片
  connect(developer_widget_, &DeveloperWidget::signLoadMachineGrabCloth, [=](int id) {
    QtConcurrent::run([=]() {
      rclcomm_->loadMachineGrabCloth(id);
    });
  });

  // 合片机-复位
  connect(developer_widget_, &DeveloperWidget::signComposeMachineReset, [=]() {
    QtConcurrent::run([=]() {
      rclcomm_->composeMachineReset();
    });
  });

  // 合片机-停止
  connect(developer_widget_, &DeveloperWidget::signComposeMachineStop, [=]() {
    // QtConcurrent::run([=]() {
    // rclcomm_->composeMachineReset();
    //});
  });

  // 合片机-除褶
  connect(developer_widget_, &DeveloperWidget::signComposeMachineWipeFold, [=]() {
    QtConcurrent::run([=]() {
      rclcomm_->composeMachineWipeFold();
    });
  });

  // 合片机-出针
  connect(developer_widget_, &DeveloperWidget::signComposeMachineExtendNeedle, [=]() {
    QtConcurrent::run([=]() {
      rclcomm_->composeMachineExtendNeedle();
    });
  });

  // 合片机-收针
  connect(developer_widget_, &DeveloperWidget::signComposeMachineWithdrawNeedle, [=]() {
    QtConcurrent::run([=]() {
      rclcomm_->composeMachineWithdrawNeedle();
    });
  });

  // 合片机-吹气
  connect(developer_widget_, &DeveloperWidget::signComposeMachineBlowWind, [=]() {
    QtConcurrent::run([=]() {
      rclcomm_->composeMachineBlowWind();
    });
  });

  // 合片机-停气
  connect(developer_widget_, &DeveloperWidget::signComposeMachineStopBlow, [=]() {
    QtConcurrent::run([=]() {
      rclcomm_->composeMachineStopBlow();
    });
  });

  // 合片机-移动抓手
  connect(developer_widget_, &DeveloperWidget::signComposeMachineMoveHand, [=](float x, float y, float z, float c) {
    QtConcurrent::run([=]() {
      rclcomm_->composeMachineMoveHand(x, y, z, c);
    });
  });

  // 合片机-移动吸盘
  connect(developer_widget_, &DeveloperWidget::signComposeMachineMoveSucker, [=]() {
    QtConcurrent::run([=]() {
      rclcomm_->composeMachineMoveSucker();
    });
  });

  // 缝纫机-复位
  connect(developer_widget_, &DeveloperWidget::signSewingMachineReset, [=]() {
    // QtConcurrent::run([=]() {
    // });
  });

  // 缝纫机-移动抓手
  connect(developer_widget_, &DeveloperWidget::signSewingMachineMoveHand, [=](float x, float y, float c, bool z) {
    QtConcurrent::run([=]() {
      rclcomm_->sewingMachineMoveHand(x, y, c, z);
    });
  });

  // 缝纫机-发送关键点
  connect(developer_widget_, &DeveloperWidget::signSewingMachineSendKeypoints, [=](syt_msgs::msg::ClothKeypoints2f keypoints) {
    QtConcurrent::run([=]() {
      rclcomm_->sewingMachineSendKeypoints(keypoints);
    });
  });
}

void MainWindow::setMutuallyLight(LIGHT_COLOR c) {
  std::map<LIGHT_COLOR, std::string> m;
  m[RED]    = "background-color: rgb(255, 0, 0);border: 3px solid black;border-radius: 15px;";
  m[YELLOW] = "background-color: rgb(255 ,255, 0);border: 3px solid black;border-radius: 15px;";
  m[GREEN]  = "background-color: rgb(0 ,255, 0);border: 3px solid black;border-radius: 15px;";
  m[GRAY]   = "background-color: gray;border: 3px solid black;border-radius: 15px;";
  switch (c) {
  case RED:
    // qDebug() << "red";
    ui->red_label->setStyleSheet(m[RED].data());
    ui->yellow_label->setStyleSheet(m[GRAY].data());
    ui->green_label->setStyleSheet(m[GRAY].data());
    break;
  case YELLOW:
    // qDebug() << "yellow";
    ui->red_label->setStyleSheet(m[GRAY].data());
    ui->yellow_label->setStyleSheet(m[YELLOW].data());
    ui->green_label->setStyleSheet(m[GRAY].data());
    break;
  case GREEN:
    // qDebug() << "green";
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
  // TODO
  ui->stop_btn->setEnabled(false);
  ui->stop_btn->setStyleSheet("color: gray;");
  ui->change_board_btn->setEnabled(false);
  ui->change_board_btn->setStyleSheet("color: gray;");
  ui->reset_btn->setEnabled(false);
  ui->reset_btn->setStyleSheet("color: gray;");
}

void MainWindow::slotMaxBtnClicked() {
  if (this->isMaximized()) {
    this->showNormal();
  } else {
    this->showMaximized();
  }
}

// 导航至上一页
void MainWindow::slotPrevPage() {
  int currentIndex = ui->stackedWidget->currentIndex();
  if (currentIndex == 0) {
    ui->stackedWidget->setCurrentIndex(ui->stackedWidget->count() - 1);
  } else {
    ui->stackedWidget->setCurrentIndex(currentIndex - 1);
  }
}

// 导航至下一页
void MainWindow::slotNextPage() {
  int currentIndex = ui->stackedWidget->currentIndex();
  if (currentIndex == ui->stackedWidget->count() - 1) {
    ui->stackedWidget->setCurrentIndex(0);
  } else {
    ui->stackedWidget->setCurrentIndex(currentIndex + 1);
  }
}

// 复位按钮槽函数
void MainWindow::resetBtnClicked() {
  emit signUpdateLabelState("重置中");

  bool res = isFastClick(ui->reset_btn, 1000);
  if (!res) {
    return;
  }

  // todo 进度条清0
  value = 0;
  // ui->processWidget->setValue(value);

  this->btnControl({ui->start_btn, ui->stop_btn, ui->add_cloth_btn, ui->add_cloth_btn, ui->change_board_btn}, {});
  this->setMutuallyLight(YELLOW);
  emit signUpdateLabelState("重置完成");

  // 复位指令
  future_ = QtConcurrent::run([=] {
    rclcomm_->resetCmd();
  });
}

// 开始按钮槽函数
void MainWindow::startBtnClicked() {
  if (!is_style_seted_) {
    showMessageBox(this, WARN, "请先设置裁片样式。", 1, {"确认"});
    return;
  }
  bool res = isFastClick(ui->start_btn, 1000);
  if (!res) {
    return;
  }

  // test_timer->start();

  this->btnControl({ui->stop_btn, ui->start_btn, ui->reset_btn, ui->add_cloth_btn, ui->change_board_btn}, {});
  setMutuallyLight(GREEN);
  emit signUpdateLabelState("运行中");

  // 开始指令
  future_ = QtConcurrent::run([=] {
    rclcomm_->startCmd();
  });
}

// 停止按钮槽函数
void MainWindow::stopBtnClicked() {
  bool res = isFastClick(ui->stop_btn, 1000);
  if (!res) {
    return;
  }

  // test_timer->stop();

  // this->btnControl({ui->reset_btn, ui->stop_btn}, {ui->start_btn, ui->add_cloth_btn, ui->change_board_btn});
  this->btnControl({ui->reset_btn, ui->start_btn, ui->stop_btn, ui->add_cloth_btn, ui->change_board_btn}, {});
  this->setMutuallyLight(YELLOW);
  emit signUpdateLabelState("运行完本次流程即将停止");

  // 停止指令
  future_ = QtConcurrent::run([=] {
    rclcomm_->stopCmd();
  });
}

// 补料按钮槽函数
void MainWindow::addClothBtnClicked() {
  bool res = isFastClick(ui->stop_btn, 1000);
  if (!res) {
    return;
  }

  waiting_spinner_widget_->start();
  // test_timer->stop();

  this->btnControl({ui->reset_btn, ui->start_btn, ui->stop_btn, ui->change_board_btn, ui->add_cloth_btn}, {});
  this->setMutuallyLight(YELLOW);
  emit signUpdateLabelState("换料模式");

  future_ = QtConcurrent::run([=] {
    rclcomm_->loadMachineAddCloth(0);
  });

  QThread::msleep(100);

  future_ = QtConcurrent::run([=] {
    rclcomm_->loadMachineAddCloth(1);
  });
}

// 换板按钮槽函数
void MainWindow::changePlateBtnClicked() {
  bool res = isFastClick(ui->stop_btn, 1000);
  if (!res) {
    return;
  }
  // test_timer->stop();

  this->btnControl({ui->reset_btn, ui->start_btn, ui->stop_btn, ui->change_board_btn, ui->add_cloth_btn}, {});
  this->setMutuallyLight(YELLOW);
  emit signUpdateLabelState("换压板模式");

  // 停止指令
  future_ = QtConcurrent::run([=] {
    // rclcomm_->stopCmd(); // TODO
  });
}

// 错误提示槽函数
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
    connect(rclcomm_, &SytRclComm::processZero, ota_dialog, &OtaUpdateDialog::clearProcessValue);
    connect(rclcomm_, &SytRclComm::updateProcess, ota_dialog, &OtaUpdateDialog::updateProcessValue);
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

void MainWindow::slotAddClothResult(bool result, int id) {
  if (id == 0) {
    add_cloth_result_B_ = result;
  } else if (id == 1) {
    add_cloth_result_A_ = result;
  }

  if (++add_cloth_count_ == 2) {
    waiting_spinner_widget_->stop();
    if (add_cloth_result_A_ && add_cloth_result_B_) {
      showMessageBox(this, SUCCESS, "上料模式设置成功，请在手动补充裁片后再点击确认。", 1, {"确认"});
    } else {
      showMessageBox(this, ERROR, "上料模式设置失败", 1, {"确认"});
    }
    add_cloth_count_ = 0;
  }
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
      pix_B_left_ = pix;
    } else if (cam_id == 1) {
      ui->leftRightVisualLabel->clear();
      ui->leftRightVisualLabel->setPixmap(pix);
      ui->leftRightVisualLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
      pix_B_right_ = pix;
    } else {
      return;
    }

  } else if (machine_id == 1) {
    if (cam_id == 0) {
      ui->rightLeftVisualLabel->clear();
      ui->rightLeftVisualLabel->setPixmap(pix);
      ui->rightLeftVisualLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
      pix_A_left_ = pix;
    } else if (cam_id == 1) {
      ui->rightRightVisualLabel->clear();
      ui->rightRightVisualLabel->setPixmap(pix);
      ui->rightRightVisualLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
      pix_A_right_ = pix;
    } else {
      return;
    }
  } else {
    return;
  }
}

////////////////////////// 工具栏函数 //////////////////////////
void MainWindow::slotShowDevLoginWindow() {
  DevLoginWindow *dev_login_window = new DevLoginWindow(this);
  // 开发者界面
  connect(dev_login_window, &DevLoginWindow::signDevMode, this, &MainWindow::slotDeveloperMode);
  dev_login_window->show();
  dev_login_window->setAttribute(Qt::WA_DeleteOnClose);
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

void MainWindow::slotDeveloperMode() {
  developer_widget_->show();
}

////////////////////////// 标定槽函数 //////////////////////////
void MainWindow::slotCompCalibRes(bool result) {
  waiting_spinner_widget_->stop();
  if (result) {
    showMessageBox(this, SUCCESS, "合片台标定成功", 1, {"退出"});
    return;
  } else {
    showMessageBox(this, SUCCESS, "合片台标定失败,请联系相关人员", 1, {"退出"});
    return;
  }
}

void MainWindow::slotSewingCalibRes(bool result) {
  waiting_spinner_widget_->stop();
  if (result) {
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
void MainWindow::slotLogShow(QString time, int level, QString location, QString func, QString msg) {
  Q_UNUSED(func)

  if (level < log_level_) {
    return;
  }

  QString htmlText;
  switch (level) {
  case 10:
    htmlText = QString("<span style=\"background-color: green; color: white; font-weight: bold;\">【 %1 】 【 %2 】 【 %3 】：  %4\n</span>").arg("DEBUG").arg(time).arg(location).arg(msg);
    break;
  case 20:
    htmlText = QString("<span style=\"background-color: white; color: black; font-weight: bold;\">【 %1 】 【 %2 】 【 %3 】：  %4\n</span>").arg("INFO").arg(time).arg(location).arg(msg);
    break;
  case 30:
    htmlText = QString("<span style=\"background-color: orange; color: white; font-weight: bold;\">【 %1 】 【 %2 】 【 %3 】：  %4\n</span>").arg("WARN").arg(time).arg(location).arg(msg);
    break;
  case 40:
    htmlText = QString("<span style=\"background-color: darkred; color: white; font-weight: bold;\">【 %1 】 【 %2 】 【 %3 】：  %4\n</span>").arg("ERROR").arg(time).arg(location).arg(msg);
    break;
  case 50:
    htmlText = QString("<span style=\"background-color: red; color: white; font-weight: bold;\">【 %1 】 【 %2 】 【 %3 】：  %4\n</span>").arg("FATAL").arg(time).arg(location).arg(msg);
    break;
  default:
    break;
  }
  ui->log_plain_text_edit->appendHtml(htmlText);
}

////////////////////////// 选择设置样式槽函数 //////////////////////////
void MainWindow::slotChooseStyleFile() {
  auto choose_style_dialog = new ChooseStyleDialog(this);
  disconnect(choose_style_dialog, &ChooseStyleDialog::signSetCurrentStyle, this, &MainWindow::slotSetCurrentStyleFile);
  connect(choose_style_dialog, &ChooseStyleDialog::signSetCurrentStyle, this, &MainWindow::slotSetCurrentStyleFile);
  disconnect(rclcomm_, &SytRclComm::signSetCurrentClothStyleFinish, choose_style_dialog, &ChooseStyleDialog::slotSetCurrentStyleFinish);
  connect(rclcomm_, &SytRclComm::signSetCurrentClothStyleFinish, choose_style_dialog, &ChooseStyleDialog::slotSetCurrentStyleFinish);

  qRegisterMetaType<syt_msgs::msg::ClothStyle>("syt_msgs::msg::ClothStyle");
  connect(choose_style_dialog, &ChooseStyleDialog::accepted, this, [=] {
    emit signGetClothStyle(style_file_prefix_, style_file_name_);
  });
  disconnect(this, SIGNAL(signGetClothStyle(QString, QString)), this, SLOT(slotGetClothStyle(QString, QString)));
  connect(this, SIGNAL(signGetClothStyle(QString, QString)), this, SLOT(slotGetClothStyle(QString, QString)));
  disconnect(rclcomm_, &SytRclComm::signGetClothStyleFinish, this, &MainWindow::slotGetClothStyleFinish);
  connect(rclcomm_, &SytRclComm::signGetClothStyleFinish, this, &MainWindow::slotGetClothStyleFinish);

  choose_style_dialog->show();
  choose_style_dialog->setAttribute(Qt::WA_DeleteOnClose);
}

void MainWindow::slotSetCurrentStyleFile(QString prefix, QString file_name) {
  style_file_prefix_ = prefix;
  style_file_name_   = file_name;
  ui->choose_style_line_edit->setText(prefix + QDir::separator() + file_name);
  future_ = QtConcurrent::run([=] {
    rclcomm_->setCurrentStyle(prefix, file_name);
  });
}

void MainWindow::slotGetClothStyle(QString prefix, QString file_name) {
  waiting_spinner_widget_->start();
  future_ = QtConcurrent::run([=] {
    rclcomm_->getClothStyle(prefix, file_name);
  });
}

void MainWindow::slotGetClothStyleFinish(bool result, syt_msgs::msg::ClothStyle cloth_style_front, syt_msgs::msg::ClothStyle cloth_style_back) {
  if (result) {
    cloth_style_front_ = cloth_style_front;
    cloth_style_back_  = cloth_style_back;

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

    is_style_seted_ = true;
  } else {
    showMessageBox(this, WARN, "获取样式信息失败", 1, {"确认"});
  }
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
  connect(auto_create_style_wizard, &AutoCreateStyleWizard::signMoveHand, this, &MainWindow::slotMoveHand);
  connect(rclcomm_, &SytRclComm::signComposeMachineMoveHandFinish, auto_create_style_wizard, &AutoCreateStyleWizard::slotMoveHandResult);

  connect(auto_create_style_wizard, &AutoCreateStyleWizard::signDetectCloth, this, &MainWindow::slotDetectClothByAutoCreateStyle);
  connect(rclcomm_, &SytRclComm::signGetClothInfoFinish, auto_create_style_wizard, &AutoCreateStyleWizard::slotDetectClothResult);

  connect(auto_create_style_wizard, &AutoCreateStyleWizard::signCreateStyle, this, &MainWindow::slotCreateStyle);
  connect(rclcomm_, &SytRclComm::signCreateStyleFinish, auto_create_style_wizard, &AutoCreateStyleWizard::slotCreateStyleResult);

  connect(auto_create_style_wizard, &AutoCreateStyleWizard::signRenameClothStyle, this, &MainWindow::slotRenameClothStyle);
  connect(rclcomm_, &SytRclComm::signRenameClothStyleFinish, auto_create_style_wizard, &AutoCreateStyleWizard::slotRenameClothStyleResult);

  auto_create_style_wizard->show();
  auto_create_style_wizard->setAttribute(Qt::WA_DeleteOnClose);
}

// 手动输入创建
void MainWindow::slotManualInputParam(ClothStyleDialog *parent) {
  ManualInputParamWizard *manual_input_param_wizard = new ManualInputParamWizard(parent);

  connect(manual_input_param_wizard, &ManualInputParamWizard::signCreateStyle, this, &MainWindow::slotCreateStyle);
  connect(rclcomm_, &SytRclComm::signCreateStyleFinish, manual_input_param_wizard, &ManualInputParamWizard::slotCreateStyleResult);

  connect(manual_input_param_wizard, &ManualInputParamWizard::signRenameClothStyle, this, &MainWindow::slotRenameClothStyle);
  connect(rclcomm_, &SytRclComm::signRenameClothStyleFinish, manual_input_param_wizard, &ManualInputParamWizard::slotRenameClothStyleResult);

  manual_input_param_wizard->show();
  manual_input_param_wizard->setAttribute(Qt::WA_DeleteOnClose);
}

// 从已有文件创建
void MainWindow::slotCreateFromSource(ClothStyleDialog *parent) {
  CreateFromSourceWizard *create_from_source_wizard = new CreateFromSourceWizard(parent);
  create_from_source_wizard->show();
  create_from_source_wizard->setAttribute(Qt::WA_DeleteOnClose);
}

void MainWindow::slotMoveHand() {
  future_ = QtConcurrent::run([=] {
    rclcomm_->composeMachineMoveHand(0, 900, 0, 0);
  });
}

void MainWindow::slotDetectClothByAutoCreateStyle(int cloth_type) {
  future_ = QtConcurrent::run([=] {
    rclcomm_->getClothInfo(1, cloth_type);
  });
}

void MainWindow::slotCreateStyle(int mode, syt_msgs::msg::ClothStyle cloth_style_front, syt_msgs::msg::ClothStyle cloth_style_back) {
  future_ = QtConcurrent::run([=] {
    rclcomm_->createStyle(mode, cloth_style_front, cloth_style_back);
  });
}

void MainWindow::slotRenameClothStyle(QString old_name, QString new_name) {
  future_ = QtConcurrent::run([=] {
    rclcomm_->renameClothStyle(old_name, new_name);
  });
}
