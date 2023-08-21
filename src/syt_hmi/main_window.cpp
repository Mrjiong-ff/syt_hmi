#include "syt_hmi/main_window.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow) {
  ui->setupUi(this);

  // 初始化节点
  rclcomm_ = new SytRclComm(this);

  // 设置各组件
  setLogComponent();
  setTimeComponent();
  setToolBar();
  setMainControlButton();
  setPageJump();
  setVisualComponent();
  setBaseComponet();
  setChooseStyleComponet();
  setDeveloperWidget();

  // 初始化控件
  initWidget();

  // 初始化其他（主要是配置相关）
  // initOther();

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
    // this->setWindowState(Qt::WindowMinimized);
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
      title_menu_->exec(QCursor::pos());
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

    prev_btn_->setGeometry(0, this->height() / 2 - init_page_btn_h / 2, init_page_btn_w, init_page_btn_h);
    next_btn_->setGeometry(this->width() - init_page_btn_w, this->height() / 2 - init_page_btn_h / 2, init_page_btn_w, init_page_btn_h);
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
    emit prev_btn_->click();
    break;
  case Qt::Key_D:
    emit next_btn_->click();
    break;
  default:
    break;
  }
}

void MainWindow::initWidget() {
  setMouseTracking(true);                           // 用于捕获鼠标移动事件
  ui->centralwidget->setMouseTracking(true);        // 注意：mainwindow及其之类都要设置mouse track，不然不生效
  ui->sytMainTitleWidget->installEventFilter(this); // 事件过滤
  this->setMutuallyLight(RED);                      // 初始状态下，亮灯
  this->setWindowFlags(Qt::FramelessWindowHint);    // 隐藏默认标题栏
  this->setWindowIcon(QIcon(":m_logo/logo/bg_logo.png"));
  ui->running_state_label->setText(QString("请选择样式文件"));

  // 初始状态下按钮状态
  this->btnControl({ui->reset_btn}, {ui->start_btn, ui->stop_btn, ui->add_cloth_btn, ui->change_board_btn});

  waiting_spinner_widget_ = new WaitingSpinnerWidget(this); // 旋转条初始化
  // 初始状态下主界面显示的任务进度条
  // ui->processWidget->setValue(100);

  // title logo
  auto tit_logo = QPixmap(":m_logo/logo/logo2.png");
  tit_logo      = tit_logo.scaled(ui->sytLogoLabel->width(), ui->sytLogoLabel->height(), Qt::AspectRatioMode::KeepAspectRatio, Qt::TransformationMode::SmoothTransformation);
  ui->sytLogoLabel->setPixmap(tit_logo);

  // 移动到中心
  QScreen *desktop = QApplication::screenAt(QCursor::pos());
  QRect rect       = desktop->availableGeometry();
  move(rect.left() + (rect.width() - width()) / 2, (rect.height() - height()) / 2);
}

void MainWindow::setLogComponent() {
  // 日志过滤筛选
  ui->log_filter_tool_btn->setIcon(QIcon(":m_icon/icon/filter-records.png"));
  ui->log_filter_tool_btn->setToolTip(QString("日志过滤筛选"));
  QAction *set_log_level_debug_act = new QAction("DEBUG", this);
  QAction *set_log_level_info_act  = new QAction("INFO", this);
  QAction *set_log_level_warn_act  = new QAction("WARN", this);
  QAction *set_log_level_error_act = new QAction("ERROR", this);
  QAction *set_log_level_fatal_act = new QAction("FATAL", this);
  ui->log_filter_tool_btn->addAction(set_log_level_debug_act);
  ui->log_filter_tool_btn->addAction(set_log_level_info_act);
  ui->log_filter_tool_btn->addAction(set_log_level_warn_act);
  ui->log_filter_tool_btn->addAction(set_log_level_error_act);
  ui->log_filter_tool_btn->addAction(set_log_level_fatal_act);

  ui->log_clear_btn->setIcon(QIcon(":m_icon/icon/clear.png"));
  ui->log_clear_btn->setToolTip(QString("日志清除"));
  ui->log_clear_btn->setParentEnabled(true);
  ui->log_clear_btn->setForeEnabled(false);
  ui->log_clear_btn->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");

  // 移动至日志末尾
  ui->log_end_btn->setIcon(QIcon(":m_icon/icon/end.png"));
  ui->log_end_btn->setToolTip("移至日志末尾");
  ui->log_end_btn->setParentEnabled(true);
  ui->log_end_btn->setForeEnabled(false);
  ui->log_end_btn->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");

  // 日志只读
  ui->log_plain_text_edit->setReadOnly(true);

  // 信号绑定
  connect(set_log_level_debug_act, &QAction::triggered, [=] { log_level_ = LOG_DEBUG; });
  connect(set_log_level_info_act, &QAction::triggered, [=] { log_level_ = LOG_INFO; });
  connect(set_log_level_warn_act, &QAction::triggered, [=] { log_level_ = LOG_WARN; });
  connect(set_log_level_error_act, &QAction::triggered, [=] { log_level_ = LOG_ERROR; });
  connect(set_log_level_fatal_act, &QAction::triggered, [=] { log_level_ = LOG_FATAL; });

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
}

void MainWindow::setTimeComponent() {
  // 时间显示定时器
  time_timer_ = new QTimer(this);
  time_timer_->start(1000);
  QTime time_now = QTime::currentTime();
  ui->current_time_label->setText(QString("时间：%1").arg(time_now.toString()));

  // 动态显示时间
  connect(time_timer_, &QTimer::timeout, ui->current_time_label, [=]() {
    QTime time_now = QTime::currentTime();
    ui->current_time_label->setText(QString("时间：%1").arg(time_now.toString()));
  });
}

void MainWindow::setToolBar() {
  ui->developer_mode_btn->setIcon(QIcon(":m_icon/icon/dev-mode.png")); // 开发者模式按钮
  ui->developer_mode_btn->setToolTip(QString("开发者界面"));

  ui->head_eye_calibration_btn->setIcon(QIcon(":m_icon/icon/handeye.png")); // 手眼标定模式按钮
  ui->head_eye_calibration_btn->setToolTip(QString("相机标定标定"));

  ui->create_style_btn->setIcon(QIcon(":m_icon/icon/shirt-line.png")); // 创建衣服样式按钮
  ui->create_style_btn->setToolTip(QString("创建衣服样式"));

  ui->lock_screen_btn->setIcon(QIcon(":m_icon/icon/lock.png")); // 屏幕上锁/解锁按钮
  ui->lock_screen_btn->setToolTip(QString("锁屏"));

  ui->help_btn->setIcon(QIcon(":m_icon/icon/help.png"));
  ui->help_btn->setToolTip(QString("帮助说明"));

  // 工具栏信号槽
  connect(ui->developer_mode_btn, &QPushButton::clicked, this, &MainWindow::slotShowDevLoginWindow);
  connect(ui->head_eye_calibration_btn, &QPushButton::clicked, this, &MainWindow::slotStartHeadEyeWindow);
  connect(ui->create_style_btn, &QPushButton::clicked, this, &MainWindow::slotStartClothStyleWindow);
  connect(ui->lock_screen_btn, &QPushButton::clicked, this, &MainWindow::slotLockScreen);
  // connect(ui->help_btn, &QPushButton::clicked, [=] {
  //// TODO: 帮助文档
  // showMessageBox(this, ERROR, "", 1, {"返回"});
  // return;
  //});

  // head eye dialog 信号槽
  connect(this, &MainWindow::signHeadEyeWindowShow, [=] {
    auto head_eye_dialog = new HeadEyeDialog(this);
    connect(head_eye_dialog, &HeadEyeDialog::signCompStart, this, &MainWindow::slotCompCalibStart);
    connect(head_eye_dialog, &HeadEyeDialog::signSewingStart, this, &MainWindow::slotSewingCalibStart);
    connect(rclcomm_, &SytRclComm::compCalibRes, this, &MainWindow::slotCompCalibRes);
    connect(rclcomm_, &SytRclComm::sewingCalibRes, this, &MainWindow::slotSewingCalibRes);
    head_eye_dialog->show();
    head_eye_dialog->setAttribute(Qt::WA_DeleteOnClose);
  });

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
}

// 主体按钮
void MainWindow::setMainControlButton() {
  // 主控组件
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

  connect(ui->reset_btn, &QPushButton::clicked, this, &MainWindow::resetBtnClicked);
  connect(ui->start_btn, &QPushButton::clicked, this, &MainWindow::startBtnClicked);
  connect(ui->stop_btn, &QPushButton::clicked, this, &MainWindow::stopBtnClicked);
  connect(ui->add_cloth_btn, &QPushButton::clicked, this, &MainWindow::addClothBtnClicked);
  connect(ui->change_board_btn, &QPushButton::clicked, this, &MainWindow::changePlateBtnClicked);
}

// 添加左右翻页按钮
void MainWindow::setPageJump() {
  int init_page_btn_w = this->width() / 50;
  int init_page_btn_h = this->height() / 9;

  prev_btn_ = new InteractiveButtonBase(this);
  prev_btn_->setIcon(QIcon(":m_icon/icon/l-page.png"));
  prev_btn_->setParentEnabled(true);
  prev_btn_->setForeEnabled(false);
  prev_btn_->setStyleSheet("background-color: rgba(150,150,150, 0.3);qproperty-press_color: rgba(0,0,100,0.3);");

  next_btn_ = new InteractiveButtonBase(this);
  next_btn_->setIcon(QIcon(":m_icon/icon/r-page.png"));
  next_btn_->setParentEnabled(true);
  next_btn_->setForeEnabled(false);
  next_btn_->setStyleSheet("background-color:  rgba(150,150,150, 0.3);qproperty-press_color: rgba(0,0,100,0.3);");

  prev_btn_->setGeometry(0, this->height() / 2 - init_page_btn_h / 2, init_page_btn_w, init_page_btn_h);
  next_btn_->setGeometry(this->width() - init_page_btn_w, this->height() / 2 - init_page_btn_h / 2, init_page_btn_w, init_page_btn_h);

  connect(prev_btn_, &QPushButton::clicked, this, &MainWindow::slotPrevPage);
  connect(next_btn_, &QPushButton::clicked, this, &MainWindow::slotNextPage);
}

// 视觉功能可视化
void MainWindow::setVisualComponent() {
  // 可视化的两个按钮
  ui->load_machine_visible_btn->setIcon(QIcon(":m_icon/icon/invisible.png"));
  ui->load_machine_visible_btn->setText("隐藏");
  ui->compose_machine_visible_btn->setIcon(QIcon(":m_icon/icon/invisible.png"));
  ui->compose_machine_visible_btn->setText("隐藏");

  connect(ui->load_machine_visible_btn, &QPushButton::clicked, [=] {
    is_load_cloth_on_ = !is_load_cloth_on_;
    if (is_load_cloth_on_) {
      qDebug("打开 上料视觉显示");
      ui->load_machine_visible_btn->setIcon(QIcon(":m_icon/icon/invisible.png"));
      ui->load_machine_visible_btn->setText("隐藏");
    } else {
      qDebug("关闭 上料视觉显示");
      ui->load_machine_visible_btn->setIcon(QIcon(":m_icon/icon/visible.png"));
      ui->load_machine_visible_btn->setText("显示");
      ui->B_left_visual_label->clear();
      ui->B_right_visual_label->clear();
      ui->A_left_visual_label->clear();
      ui->A_right_visual_label->clear();
      ui->B_left_visual_label->setText("NO IMAGE");
      ui->B_right_visual_label->setText("NO IMAGE");
      ui->A_left_visual_label->setText("NO IMAGE");
      ui->A_right_visual_label->setText("NO IMAGE");
    }
  });
  connect(ui->compose_machine_visible_btn, &QPushButton::clicked, [=] {
    is_comp_cloth_on_ = !is_comp_cloth_on_;
    if (is_comp_cloth_on_) {
      qDebug("打开 合片视觉显示");
      ui->compose_machine_visible_btn->setIcon(QIcon(":m_icon/icon/invisible.png"));
      ui->compose_machine_visible_btn->setText("隐藏");
    } else {
      qDebug("关闭 合片视觉显示");
      ui->compose_machine_visible_btn->setIcon(QIcon(":m_icon/icon/visible.png"));
      ui->compose_machine_visible_btn->setText("显示");
      ui->leftCompLabel->clear();
      ui->rightCompLabel->clear();
      ui->leftCompLabel->setText("NO IMAGE");
      ui->rightCompLabel->setText("NO IMAGE");
    }
  });
}

void MainWindow::setBaseComponet() {
  // 右上角菜单按钮
  QMenu *menu = new QMenu(this);

  QAction *update_act = new QAction(this);
  QAction *about_act  = new QAction(this);

  update_act->setText("检查更新");
  update_act->setIcon(QIcon(":m_icon/icon/update.png"));
  about_act->setText("关于速英");
  about_act->setIcon(QIcon(":m_icon/icon/about.png"));

  menu->addAction(update_act);
  menu->addAction(about_act);
  ui->menu_btn->setMenu(menu);

  // 标题栏右键菜单
  ui->sytMainTitleWidget->setContextMenuPolicy(Qt::CustomContextMenu);
  title_menu_ = new QMenu(this);

  QAction *min_act   = new QAction(this);
  QAction *max_act   = new QAction(this);
  QAction *full_act  = new QAction(this);
  QAction *close_act = new QAction(this);

  min_act->setText("最小化");
  max_act->setText("最大化");
  full_act->setText("全屏窗口化");
  close_act->setText("关闭");

  title_menu_->addAction(min_act);
  title_menu_->addAction(max_act);
  title_menu_->addAction(full_act);
  title_menu_->addAction(close_act);

  // main window右上角按钮 放大 缩小 菜单等
  connect(ui->close_btn, &WinCloseButton::clicked, this, &MainWindow::close);
  connect(ui->min_btn, &WinMaxButton::clicked, this, &MainWindow::showMinimized);
  connect(ui->max_btn, &WinMaxButton::clicked, this, &MainWindow::slotMaxBtnClicked);
  connect(ui->menu_btn, &WinMenuButton::toggled, [=] { menu->exec(); });

  // action 相关
  connect(min_act, &QAction::triggered, this, &MainWindow::showMinimized);
  connect(max_act, &QAction::triggered, this, &MainWindow::slotMaxBtnClicked);
  connect(full_act, &QAction::triggered, this, &MainWindow::showFullScreen);
  connect(close_act, &QAction::triggered, this, &MainWindow::close);
  connect(update_act, &QAction::triggered, this, &MainWindow::triggeredOTAUpdate);
  // TODO: 帮助界面
  connect(about_act, &QAction::triggered, this, [=] {
    showMessageBox(this, ERROR, "帮助", 1, {"返回"});
    return;
  });
}

void MainWindow::setChooseStyleComponet() {
  // 样式树形列表
  ui->cloth_style_tree_widget->header()->resizeSection(0, 200);
  ui->cloth_style_tree_widget->setAlternatingRowColors(true);
  ui->cloth_style_tree_widget->setAnimated(true);
  ui->cloth_style_tree_widget->setUniformRowHeights(true);
  ui->cloth_style_tree_widget->setHeaderLabels(QStringList() << "属性"
                                                             << "值");
  // 设置样式line edit只读
  ui->choose_style_line_edit->setReadOnly(true);

  connect(ui->choose_style_btn, &QPushButton::clicked, this, &MainWindow::slotChooseStyleFile); // 选择样式信号槽
}

// 开发者界面
void MainWindow::setDeveloperWidget() {
  developer_widget_ = new DeveloperWidget(this);

  // 绑定机器控制按钮
  bindControlConnection();
}

void MainWindow::settingConnection() {
  // 状态label显示
  connect(this, &MainWindow::signUpdateLabelState, [=](QString text) {
    ui->running_state_label->setText(text);
  });

  // 跳转界面显示上料机监控
  connect(ui->stackedWidget, &QStackedWidget::currentChanged, this, [=]() {
    if (ui->stackedWidget->currentWidget() == ui->page1) {
      if (!pix_A_left_.isNull()) {
        ui->A_left_visual_label->clear();
        ui->A_left_visual_label->setPixmap(pix_A_left_);
        ui->A_left_visual_label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
      }

      if (!pix_A_right_.isNull()) {
        ui->A_right_visual_label->clear();
        ui->A_right_visual_label->setPixmap(pix_A_right_);
        ui->A_right_visual_label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
      }

      if (!pix_B_left_.isNull()) {
        ui->B_left_visual_label->clear();
        ui->B_left_visual_label->setPixmap(pix_B_left_);
        ui->B_left_visual_label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
      }

      if (!pix_B_right_.isNull()) {
        ui->B_right_visual_label->clear();
        ui->B_right_visual_label->setPixmap(pix_B_right_);
        ui->B_right_visual_label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
      }
    }
  });

  // 上料机可视化相关槽函数
  connect(rclcomm_, &SytRclComm::visualLoadClothRes, this, &MainWindow::slotVisualLoadCloth);

  // TODO 合片机可视化相关槽函数

  // 一些节点相关的报错槽
  connect(rclcomm_, &SytRclComm::errorNodeMsgSign, this, &MainWindow::errorNodeMsgSlot);                            // 错误提示
  connect(rclcomm_, &SytRclComm::waitUpdateResultSuccess, this, &MainWindow::otaResultShow, Qt::QueuedConnection);  // ota停止
  connect(rclcomm_, &SytRclComm::installRes, this, &MainWindow::otaInstallSuccess, Qt::QueuedConnection);           // ota安装
  connect(rclcomm_, &SytRclComm::signLogPub, this, &MainWindow::slotLogShow, Qt::ConnectionType::QueuedConnection); // rosout回调消息

  // 补料模式结束
  connect(rclcomm_, &SytRclComm::signLoadMachineAddClothFinish, this, &MainWindow::slotAddClothResult);

  // 成功率统计
  connect(rclcomm_, &SytRclComm::machineIdle, [=]() {
    this->btnControl({ui->reset_btn, ui->add_cloth_btn, ui->change_board_btn, ui->start_btn}, {ui->stop_btn});
    ++success_count_;
    ++round_count_; // TODO: 增加异常次数计数，统计成功率
    ui->processWidget->setRange(0, round_count_);
    ui->processWidget->setValue(success_count_);
  });

  connect(rclcomm_, &SytRclComm::finishOneRound, [=]() {
    ++success_count_;
    ++round_count_;
    ui->processWidget->setRange(0, round_count_);
    ui->processWidget->setValue(success_count_);
  });
}

void MainWindow::bindControlConnection() {
  // 切换模式
  connect(developer_widget_, &DeveloperWidget::signChooseMode, [=](int mode) {
    rclcomm_->changeMode(mode);
  });

  // 合片反馈
  qRegisterMetaType<syt_msgs::msg::ComposeMachineState>("syt_msgs::msg::ComposeMachineState");
  connect(rclcomm_, &SytRclComm::updateComposeMachineState, [=](syt_msgs::msg::ComposeMachineState state) {
    developer_widget_->setComposeMachineState(state);
  });

  // 缝纫反馈
  connect(rclcomm_, &SytRclComm::updateSewingMachineState, [=](syt_msgs::msg::SewingMachineState state) {
    developer_widget_->setSewingMachineState(state);
  });

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

  // 上料机-预备设置
  connect(developer_widget_, &DeveloperWidget::signLoadMachinePreSetup, [=](int id) {
    QtConcurrent::run([=]() {
      rclcomm_->loadMachinePreSetup(id);
    });
  });

  // 上料机-视觉对位
  connect(developer_widget_, &DeveloperWidget::signLoadMachinePreSetup, [=](int id) {
    QtConcurrent::run([=]() {
      rclcomm_->loadMachineVisualAlign(id);
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
  connect(developer_widget_, &DeveloperWidget::signComposeMachineMoveSucker, [=](syt_msgs::msg::ComposeMachineSuckerStates sucker_states) {
    QtConcurrent::run([=]() {
      rclcomm_->composeMachineMoveSucker(sucker_states);
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

void MainWindow::btnControl(std::vector<QPushButton *> enables, std::vector<QPushButton *> disables) {
  for (auto i : enables) {
    i->setEnabled(true);
    i->setStyleSheet("color: black;");
  }
  for (auto i : disables) {
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
  if (!is_style_seted_) {
    showMessageBox(this, WARN, "请先设置裁片样式。", 1, {"确认"});
    return;
  }

  bool res = isFastClick(ui->reset_btn, 1000);
  if (!res) {
    return;
  }
  this->setMutuallyLight(YELLOW);

  // 清空可视化
  ui->B_left_visual_label->clear();
  ui->B_right_visual_label->clear();
  ui->A_left_visual_label->clear();
  ui->A_right_visual_label->clear();
  ui->B_left_visual_label->setText("NO IMAGE");
  ui->B_right_visual_label->setText("NO IMAGE");
  ui->A_left_visual_label->setText("NO IMAGE");
  ui->A_right_visual_label->setText("NO IMAGE");

  round_count_   = 0;
  success_count_ = 0;

  // 复位指令
  emit signUpdateLabelState("复位中");
  rclcomm_->resetCmd();
  emit signUpdateLabelState("复位完成");

  this->btnControl({ui->start_btn, ui->stop_btn, ui->add_cloth_btn, ui->change_board_btn}, {ui->reset_btn});
}

// 开始按钮槽函数
void MainWindow::startBtnClicked() {
  bool res = isFastClick(ui->start_btn, 1000);
  if (!res) {
    return;
  }
  setMutuallyLight(GREEN);

  // 开始指令
  rclcomm_->startCmd();
  emit signUpdateLabelState("运行中");

  this->btnControl({ui->stop_btn}, {ui->start_btn, ui->reset_btn, ui->add_cloth_btn, ui->change_board_btn});
}

// 停止按钮槽函数
void MainWindow::stopBtnClicked() {
  bool res = isFastClick(ui->stop_btn, 1000);
  if (!res) {
    return;
  }
  this->setMutuallyLight(YELLOW);

  // 停止指令
  emit signUpdateLabelState("停止运行中");
  rclcomm_->stopCmd();
  emit signUpdateLabelState("停止");

  this->btnControl({ui->reset_btn, ui->add_cloth_btn, ui->change_board_btn}, {ui->start_btn, ui->stop_btn});
}

// 补料按钮槽函数
void MainWindow::addClothBtnClicked() {
  bool res = isFastClick(ui->stop_btn, 1000);
  if (!res) {
    return;
  }

  waiting_spinner_widget_->start();

  this->setMutuallyLight(YELLOW);
  emit signUpdateLabelState("换料模式");

  QFuture<void> future_B = QtConcurrent::run([=] {
    rclcomm_->loadMachineAddCloth(0);
  });

  QThread::msleep(100);

  QFuture<void> future_A = QtConcurrent::run([=] {
    rclcomm_->loadMachineAddCloth(1);
  });

  // future_A.waitForFinished();
  // future_B.waitForFinished();

  this->btnControl({ui->reset_btn, ui->change_board_btn, ui->add_cloth_btn}, {ui->start_btn, ui->stop_btn});
}

// 换板按钮槽函数
void MainWindow::changePlateBtnClicked() {
  bool res = isFastClick(ui->stop_btn, 1000);
  if (!res) {
    return;
  }

  this->btnControl({ui->reset_btn, ui->change_board_btn, ui->add_cloth_btn}, {ui->start_btn, ui->stop_btn});
  this->setMutuallyLight(YELLOW);
  emit signUpdateLabelState("换压板模式");

  // 换压板模式
  // rclcomm_->stopCmd(); // TODO
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
      // future2.waitForFinished();
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
  auto pix = QPixmap::fromImage(image.scaled(ui->B_left_visual_label->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
  if (machine_id == 0) {
    if (cam_id == 0) {
      ui->B_left_visual_label->clear();
      ui->B_left_visual_label->setPixmap(pix);
      ui->B_left_visual_label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
      pix_B_left_ = pix;
    } else if (cam_id == 1) {
      ui->B_right_visual_label->clear();
      ui->B_right_visual_label->setPixmap(pix);
      ui->B_right_visual_label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
      pix_B_right_ = pix;
    } else {
      return;
    }

  } else if (machine_id == 1) {
    if (cam_id == 0) {
      ui->A_left_visual_label->clear();
      ui->A_left_visual_label->setPixmap(pix);
      ui->A_left_visual_label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
      pix_A_left_ = pix;
    } else if (cam_id == 1) {
      ui->A_right_visual_label->clear();
      ui->A_right_visual_label->setPixmap(pix);
      ui->A_right_visual_label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
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
  if (res == 0) {
    emit signHeadEyeWindowShow();
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

    QString text = QString("请将前片放置于上料台%1，后片放置于上料台%2").arg(cloth_style_front.cloth_location == syt_msgs::msg::ClothStyle::TABLE_A ? "A" : "B").arg(cloth_style_back.cloth_location == syt_msgs::msg::ClothStyle::TABLE_A ? "A" : "B");
    showMessageBox(this, SUCCESS, text, 1, {"确认"});
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
