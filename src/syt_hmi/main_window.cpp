#include "syt_hmi/main_window.h"

#include <QDesktopWidget>
#include <QGraphicsOpacityEffect>
#include <QScreen>
#include <QScrollBar>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QTranslator>

#include "syt_hmi/globalapplication.h"
#include "syt_hmi/param_item_delegate.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent),
                                          ui(new Ui::MainWindow),
                                          exe_count_(0),
                                          max_count_(0),
                                          cur_count_(0) {
  ui->setupUi(this);

  // 初始化节点
  rclcomm_ = new SytRclComm(this);

  // 设置各组件
  setStatisticComponent();
  setPreviewComponent();
  setLogComponent();
  setTimeComponent();
  setToolBar();
  setMainControlButton();
  setVisualComponent();
  setBaseComponet();
  setChooseStyleComponet();
  setParamManageComponet();
  setDeveloperWidget();
  setParamSetWidget();

  // 初始化控件
  initWidget();

  // 初始化其他（主要是配置相关）
  // initOther();

  // 信号槽
  settingConnection();
}

MainWindow::~MainWindow() { this->deleteAll(); }

void MainWindow::region(const QPoint &currentGlobalPoint) {
  // 获取窗体在屏幕上的位置区域，topLeft为坐上角点，rightButton为右下角点
  QRect rect = this->rect();

  QPoint topLeft =
      this->mapToGlobal(rect.topLeft()); // 将左上角的(0,0)转化为全局坐标
  QPoint rightButton = this->mapToGlobal(rect.bottomRight());

  int x = currentGlobalPoint.x(); // 当前鼠标的坐标
  int y = currentGlobalPoint.y();

  if (((topLeft.x() + PADDING >= x) && (topLeft.x() <= x)) &&
      ((topLeft.y() + PADDING >= y) && (topLeft.y() <= y))) {
    // 左上角
    dir_ = LEFTTOP;
    this->setCursor(QCursor(Qt::SizeFDiagCursor)); // 设置光标形状
  } else if (((x >= rightButton.x() - PADDING) && (x <= rightButton.x())) &&
             ((y >= rightButton.y() - PADDING) && (y <= rightButton.y()))) {
    // 右下角
    dir_ = RIGHTBOTTOM;
    this->setCursor(QCursor(Qt::SizeFDiagCursor));
  } else if (((x <= topLeft.x() + PADDING) && (x >= topLeft.x())) &&
             ((y >= rightButton.y() - PADDING) && (y <= rightButton.y()))) {
    // 左下角
    dir_ = LEFTBOTTOM;
    this->setCursor(QCursor(Qt::SizeBDiagCursor));
  } else if (((x <= rightButton.x()) && (x >= rightButton.x() - PADDING)) &&
             ((y >= topLeft.y()) && (y <= topLeft.y() + PADDING))) {
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

bool MainWindow::event(QEvent *event) {
  if (event->type() == QEvent::LanguageChange) {
    ui->retranslateUi(this);

    QString text;
    text = QApplication::translate("MainWindow",
                                   ui->progress_bar_3->property("translator")
                                       .toString()
                                       .toStdString()
                                       .c_str(),
                                   nullptr);
    ui->progress_bar_3->setLabel(text);

    text = QApplication::translate("MainWindow",
                                   ui->running_state_label->property("translator")
                                       .toString()
                                       .toStdString()
                                       .c_str(),
                                   nullptr);
    ui->running_state_label->setText(text);

    for (auto act : title_menu_->actions()) {
      text = QApplication::translate("MainWindow",
                                     act->property("translator").toString().toStdString().c_str(),
                                     nullptr);
      act->setText(text);
    }

    for (auto act : ui->menu_btn->menu()->actions()) {
      text = QApplication::translate("MainWindow",
                                     act->property("translator").toString().toStdString().c_str(),
                                     nullptr);
      act->setText(text);
    }
  }
  return QWidget::event(event);
}

void MainWindow::mousePressEvent(QMouseEvent *event) {
  switch (event->button()) {
  case Qt::LeftButton:
    is_mouse_left_press_down_ = true;
    if (ui->style_graphics_view->underMouse()) {
      is_mouse_left_press_down_ = false;
    }

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
  QRect rect = this->rect();               // rect == QRect(0,0 1280x720)
  QPoint topLeft = mapToGlobal(rect.topLeft());
  QPoint bottomRight = mapToGlobal(rect.bottomRight());

  if (this->windowState() != Qt::WindowMaximized) {
    if (!is_mouse_left_press_down_) // 没有按下左键时
    {
      this->region(globalPoint); // 窗口大小的改变——判断鼠标位置，改变光标形状
    } else {
      if (dir_ != NONE) {
        QRect newRect(topLeft,
                      bottomRight); // 定义一个矩形  拖动后最大1000*1618

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
  if (style_scene_->items().size()) {
    qreal gv_width = ui->style_graphics_view->width();
    qreal gv_height = ui->style_graphics_view->height();

    image_item_->setQGraphicsViewWH(gv_width, gv_height);
    style_scene_->setSceneRect(-gv_width / 2, -gv_height / 2, gv_width, gv_height);
  }
  event->accept();
}

void MainWindow::keyPressEvent(QKeyEvent *event) {
  if (event->modifiers() == Qt::AltModifier) {
    // 一些主界面的快捷键
    switch (event->key()) {
    case Qt::Key_A:
      slotPrevPage();
      break;
    case Qt::Key_D:
      slotNextPage();
      break;
    default:
      break;
    }
  }
}

void MainWindow::checkOk() {
  QString file_path;
  file_path.append('/');
  file_path.append('v');
  file_path.append('a');
  file_path.append('r');
  file_path.append('/');
  file_path.append('l');
  file_path.append('i');
  file_path.append('b');
  file_path.append('/');
  file_path.append('p');
  file_path.append('r');
  file_path.append('o');
  file_path.append('f');
  file_path.append('i');
  file_path.append('l');
  file_path.append('e');

  QString field_max;
  field_max.append('m');
  field_max.append('a');
  field_max.append('x');

  QString field_cur;
  field_cur.append('c');
  field_cur.append('u');
  field_cur.append('r');

  cv::FileStorage fs(file_path.toStdString(), cv::FileStorage::READ);
  int max_count;
  int cur_count;

  fs[field_max.toStdString().c_str()] >> max_count;
  fs[field_cur.toStdString().c_str()] >> cur_count;
  fs.release();

  if (cur_count + exe_count_ >= max_count) {
    emit signOverLimit();
  }
  return;
}

void MainWindow::initWidget() {
  setMouseTracking(true); // 用于捕获鼠标移动事件
  ui->centralwidget->setMouseTracking(
      true);                                                            // 注意：mainwindow及其之类都要设置mouse track，不然不生效
  ui->sytMainTitleWidget->installEventFilter(this);                     // 事件过滤
  ui->running_state_label->setText(QString(tr("请选择样式文件")));      // 初始标题
  ui->running_state_label->setProperty("translator", "请选择样式文件"); // 初始标题
  setWindowFlags(Qt::FramelessWindowHint);                              // 隐藏默认标题栏
  setWindowIcon(QIcon(":m_logo/logo/bg_logo.png"));
  // setMutuallyLight(GREEN);                                //
  // 初始状态下，亮绿灯

  // 设置预览界面鼠标不穿透
  ui->style_graphics_view->setAttribute(Qt::WA_AlwaysStackOnTop, true);

  // 初始状态下按钮状态
  this->btnControl({ui->reset_btn, ui->choose_style_btn},
                   {ui->start_btn, ui->pause_btn, ui->end_btn, ui->add_cloth_btn});

  // 旋转条初始化
  waiting_spinner_widget_ = new WaitingSpinnerWidget(this);

  // title logo
  auto tit_logo = QPixmap(":m_logo/logo/logo2.png");
  tit_logo = tit_logo.scaled(ui->sytLogoLabel->width(), ui->sytLogoLabel->height(),
                             Qt::AspectRatioMode::KeepAspectRatio,
                             Qt::TransformationMode::SmoothTransformation);
  ui->sytLogoLabel->setPixmap(tit_logo);

  // 移动到中心
  QScreen *desktop = QApplication::screenAt(QCursor::pos());
  QRect rect = desktop->availableGeometry();
  move(rect.left() + (rect.width() - width()) / 2, (rect.height() - height()) / 2);
}

void MainWindow::setStatisticComponent() {
  ui->progress_bar_1->setLabel(QString(tr("B区余量")));
  ui->progress_bar_1->setProperty("translator", "B区余量");
  ui->progress_bar_1->setPercentage(true);
  ui->progress_bar_1->setProgressBar(100, 100);
  ui->progress_bar_1->hide();

  ui->progress_bar_2->setLabel(QString(tr("A区余量")));
  ui->progress_bar_2->setProperty("translator", "A区余量");
  ui->progress_bar_2->setPercentage(true);
  ui->progress_bar_2->setProgressBar(100, 100);
  ui->progress_bar_2->hide();

  ui->progress_bar_3->setLabel(QString(tr("产量")));
  ui->progress_bar_3->setProperty("translator", "产量");
  ui->progress_bar_3->setProgressBar(0, 400);
}

void MainWindow::setPreviewComponent() {
  style_scene_ = new QGraphicsScene(this);

  // 设置graphicsview属性
  ui->style_graphics_view->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  ui->style_graphics_view->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  ui->style_graphics_view->setAlignment(Qt::AlignCenter);
  ui->style_graphics_view->setScene(style_scene_);
}

void MainWindow::setLogComponent() {
  // 日志过滤筛选
  QAction *set_log_level_debug_act = new QAction("DEBUG", this);
  QAction *set_log_level_info_act = new QAction("INFO", this);
  QAction *set_log_level_warn_act = new QAction("WARN", this);
  QAction *set_log_level_error_act = new QAction("ERROR", this);
  QAction *set_log_level_fatal_act = new QAction("FATAL", this);
  ui->log_filter_tool_btn->addAction(set_log_level_debug_act);
  ui->log_filter_tool_btn->addAction(set_log_level_info_act);
  ui->log_filter_tool_btn->addAction(set_log_level_warn_act);
  ui->log_filter_tool_btn->addAction(set_log_level_error_act);
  ui->log_filter_tool_btn->addAction(set_log_level_fatal_act);

  // 日志只读
  ui->log_plain_text_edit->setReadOnly(true);

  // 信号绑定
  connect(set_log_level_debug_act, &QAction::triggered, [=] { log_level_ = LOG_DEBUG; });
  connect(set_log_level_info_act, &QAction::triggered, [=] { log_level_ = LOG_INFO; });
  connect(set_log_level_warn_act, &QAction::triggered, [=] { log_level_ = LOG_WARN; });
  connect(set_log_level_error_act, &QAction::triggered, [=] { log_level_ = LOG_ERROR; });
  connect(set_log_level_fatal_act, &QAction::triggered, [=] { log_level_ = LOG_FATAL; });

  // 日志清除
  connect(ui->log_clear_btn, &QPushButton::clicked,
          [=] { ui->log_plain_text_edit->clear(); });

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
  ui->current_time_label->setText(QString("%1").arg(time_now.toString()));

  // 动态显示时间
  connect(time_timer_, &QTimer::timeout, ui->current_time_label, [=]() {
    QTime time_now = QTime::currentTime();
    ui->current_time_label->setText(QString("%1").arg(time_now.toString()));
  });
}

void MainWindow::setToolBar() {
  // 工具栏信号槽
  connect(ui->developer_mode_btn, &QPushButton::clicked, this, &MainWindow::slotShowDevLoginWindow);
  connect(ui->head_eye_calibration_btn, &QPushButton::clicked, this, &MainWindow::slotStartHeadEyeWindow);
  connect(ui->create_style_btn, &QPushButton::clicked, this, &MainWindow::slotStartClothStyleWindow);
  connect(ui->lock_screen_btn, &QPushButton::clicked, this, &MainWindow::slotLockScreen);
  connect(ui->param_set_btn, &QPushButton::clicked, this, &MainWindow::slotParamSet);
  // TODO:delete
  ui->help_btn->hide();
  // connect(ui->help_btn, &QPushButton::clicked, [=] {
  //// TODO: 帮助文档
  // showMessageBox(this, ERROR, "", 1, {"返回"});
  // return;
  //});

  // head eye dialog 信号槽
  connect(this, &MainWindow::signHandEyeWindowShow, [=] {
    auto hand_eye_dialog = new HandEyeDialog(this);
    connect(hand_eye_dialog, &HandEyeDialog::signCompStart, this, &MainWindow::slotCompCalibStart, Qt::UniqueConnection);
    connect(hand_eye_dialog, &HandEyeDialog::signSewingStart, this, &MainWindow::slotSewingCalibStart, Qt::UniqueConnection);
    connect(rclcomm_, &SytRclComm::compCalibRes, hand_eye_dialog, &HandEyeDialog::slotCompCalibRes, Qt::UniqueConnection);
    connect(rclcomm_, &SytRclComm::sewingCalibRes, hand_eye_dialog, &HandEyeDialog::slotSewingCalibRes, Qt::UniqueConnection);
    hand_eye_dialog->show();
    hand_eye_dialog->setAttribute(Qt::WA_DeleteOnClose);
  });

  // 创建样式信号槽
  connect(this, &MainWindow::signClothStyleWindowShow, [=] {
    auto cloth_style_dialog = new ClothStyleDialog(this);

    //// 从CAD创建
    // void (ClothStyleDialog::*create_from_cad_signal)(ClothStyleDialog
    // *parent) = &ClothStyleDialog::signCreateFromCAD; void
    // (MainWindow::*create_from_cad_slot)(ClothStyleDialog *parent)         =
    // &MainWindow::slotCreateFromCAD; connect(cloth_style_dialog,
    // create_from_cad_signal, this, create_from_cad_slot,
    // Qt::ConnectionType::QueuedConnection);

    // 自动创建
    void (ClothStyleDialog::*auto_create_style_signal)(ClothStyleDialog *parent) = &ClothStyleDialog::signAutoCreateStyle;
    void (MainWindow::*auto_create_style_slot)(ClothStyleDialog *parent) = &MainWindow::slotAutoCreateStyle;
    connect(cloth_style_dialog, auto_create_style_signal, this, auto_create_style_slot, Qt::ConnectionType::QueuedConnection);

    // 复位抓手（手动创建改）
    void (ClothStyleDialog::*manual_input_param_signal)(ClothStyleDialog *parent) = &ClothStyleDialog::signManualInputParam;
    // void (MainWindow::*manual_input_param_slot)(ClothStyleDialog *parent) = &MainWindow::slotManualInputParam;
    // connect(cloth_style_dialog, manual_input_param_signal, this, manual_input_param_slot, Qt::ConnectionType::QueuedConnection);
    connect(cloth_style_dialog, manual_input_param_signal, [=](){
      waiting_spinner_widget_->start();
      QtConcurrent::run([=]() {
        rclcomm_->composeMachineReset();
        rclcomm_->sewingMachineReset();
        waiting_spinner_widget_->stop();
      });
    });

    // 从已有文件创建
    void (ClothStyleDialog::*create_from_source_signal)(ClothStyleDialog *parent) = &ClothStyleDialog::signCreateFromSource;
    void (MainWindow::*create_from_source_slot)(ClothStyleDialog *parent) = &MainWindow::slotCreateFromSource;
    connect(cloth_style_dialog, create_from_source_signal, this, create_from_source_slot, Qt::ConnectionType::QueuedConnection);

    cloth_style_dialog->show();
    cloth_style_dialog->setAttribute(Qt::WA_DeleteOnClose);
  });
}

// 主体按钮
void MainWindow::setMainControlButton() {
  // 主控组件
  connect(ui->reset_btn, &QPushButton::clicked, this, &MainWindow::resetBtnClicked);
  connect(rclcomm_, &SytRclComm::signResetFinish, this, &MainWindow::resetFinish);
  connect(ui->start_btn, &QPushButton::clicked, this, &MainWindow::startBtnClicked);
  connect(rclcomm_, &SytRclComm::signStartFinish, this, &MainWindow::startFinish);
  connect(ui->pause_btn, &QPushButton::clicked, this, &MainWindow::pauseBtnClicked);
  connect(rclcomm_, &SytRclComm::signPauseFinish, this, &MainWindow::pauseFinish);
  connect(ui->end_btn, &QPushButton::clicked, this, &MainWindow::stopBtnClicked);
  connect(rclcomm_, &SytRclComm::signStopFinish, this, &MainWindow::stopFinish);
  connect(ui->add_cloth_btn, &QPushButton::clicked, this, &MainWindow::addClothBtnClicked);
  connect(this, &MainWindow::signParamProcessFinish, this, &MainWindow::paramProcessFinish);
  connect(this, &MainWindow::signScramCondition, this, &MainWindow::deadErrorProcess);
}

// 视觉功能可视化
void MainWindow::setVisualComponent() {
  // 可视化的两个按钮
  ui->load_machine_visible_btn->setIcon(QIcon(":m_icon/icon/preview_close.svg"));
  ui->load_machine_visible_btn->setText(tr("隐藏"));

  connect(ui->load_machine_visible_btn, &QPushButton::clicked, [=] {
    is_load_cloth_on_ = !is_load_cloth_on_;
    if (is_load_cloth_on_) {
      ui->load_machine_visible_btn->setIcon(QIcon(":m_icon/icon/preview_close.svg"));
      ui->load_machine_visible_btn->setText(tr("隐藏"));
      showLoadMachineImage();
    } else {
      ui->load_machine_visible_btn->setIcon(QIcon(":m_icon/icon/preview_open.svg"));
      ui->load_machine_visible_btn->setText(tr("显示"));
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
}

void MainWindow::setBaseComponet() {
  // 右上角菜单按钮
  QMenu *menu = new QMenu(this);

  QAction *update_act = new QAction(tr("检查更新"), this);
  update_act->setProperty("translator", "检查更新");
  update_act->setIcon(QIcon(":m_icon/icon/update.svg"));

  QAction *about_act = new QAction(tr("关于速英"), this);
  about_act->setProperty("translator", "关于速英");
  about_act->setIcon(QIcon(":m_icon/icon/about.svg"));

  QAction *translate_act = new QAction(tr("语言选项"), this);
  translate_act->setProperty("translator", "语言选项");
  translate_act->setIcon(QIcon(":m_icon/icon/translate.svg"));

  menu->addAction(update_act);
  menu->addAction(about_act);
  menu->addAction(translate_act);
  ui->menu_btn->setMenu(menu);

  // 标题栏右键菜单
  ui->sytMainTitleWidget->setContextMenuPolicy(Qt::CustomContextMenu);
  title_menu_ = new QMenu(this);

  QAction *min_act = new QAction(this);
  QAction *max_act = new QAction(this);
  QAction *full_act = new QAction(this);
  QAction *close_act = new QAction(this);

  min_act->setText(tr("最小化"));
  min_act->setProperty("translator", "最小化");
  max_act->setText(tr("最大化"));
  max_act->setProperty("translator", "最大化");
  full_act->setText(tr("全屏窗口化"));
  full_act->setProperty("translator", "全屏窗口化");
  close_act->setText(tr("关闭"));
  close_act->setProperty("translator", "关闭");

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
  connect(full_act, &QAction::triggered, this, [=]() {
    showFullScreen();
    ui->max_btn->setIcon(QIcon(":m_icon/icon/off_screen.svg"));
  });
  connect(close_act, &QAction::triggered, this, &MainWindow::close);
  // TODO: 增加OTA
  // connect(update_act, &QAction::triggered, this,
  // &MainWindow::triggeredOTAUpdate);
  connect(update_act, &QAction::triggered, this, [=]() {
    showMessageBox(this, SUCCESS, "当前固件已为最新版", 1, {"确认"});
  });
  // TODO: 帮助界面
  connect(about_act, &QAction::triggered, this, [=] {
    showMessageBox(this, SUCCESS, "当前版本1.3.1", 1, {"返回"});
    return;
  });

  // 语言选项界面
  connect(translate_act, &QAction::triggered, this, [=]() {
    std::string config_path = std::string(getenv("ENV_ROBOT_ETC")) + "/syt_hmi/syt_hmi.yaml";
    cv::FileStorage fs(config_path, cv::FileStorage::READ);
    int idx;
    fs["language"] >> idx;
    fs.release();
    TranslateDialog *translate_dialog = new TranslateDialog(this, idx);

    connect(translate_dialog, &TranslateDialog::confirmLanguage, this, [=](int index) {
      app->switchApplicationLanguage(index);

      if (getenv("ENV_GLOBAL_ETC")) {
        std::string config_path = std::string(getenv("ENV_GLOBAL_ETC")) + "/syt_hmi/syt_hmi.yaml";
        cv::FileStorage fs(config_path, cv::FileStorage::WRITE);
        fs << "language" << index;
        fs.release();
      } else if (getenv("ENV_ROBOT_ETC")) {
        std::string config_path = std::string(getenv("ENV_ROBOT_ETC")) + "/syt_hmi/syt_hmi.yaml";
        cv::FileStorage fs(config_path, cv::FileStorage::WRITE);
        fs << "language" << index;
        fs.release();
      }
    });
    translate_dialog->show();
  });
}

void MainWindow::setChooseStyleComponet() {
  // 样式树形列表
  ui->cloth_style_tree_widget->header()->resizeSection(0, 200);
  ui->cloth_style_tree_widget->setAlternatingRowColors(true);
  ui->cloth_style_tree_widget->setAnimated(true);
  ui->cloth_style_tree_widget->setUniformRowHeights(true);

  // 设置样式line edit只读
  ui->choose_style_line_edit->setReadOnly(true);

  connect(ui->choose_style_btn, &QPushButton::clicked, this, &MainWindow::slotChooseStyleFile); // 选择样式信号槽
}

void MainWindow::setParamManageComponet() {
  // 样式树形列表
  ui->param_manage_tree_view->header()->resizeSection(0, 200);

  // 构造Model
  model_ = new QStandardItemModel(ui->param_manage_tree_view);

  // 导入默认参数
  if (getenv("ENV_GLOBAL_ETC")) {
    QString param_path = QString(getenv("ENV_GLOBAL_ETC")) + "/syt_hmi/param.json";
    current_param_file_ = param_path;
    readJson(param_path, model_);
  } else if (getenv("ENV_ROBOT_ETC")) {
    QString param_path = QString(getenv("ENV_ROBOT_ETC")) + "/syt_hmi/param.json";
    current_param_file_ = param_path;
    readJson(param_path, model_);
  }

  ui->param_manage_tree_view->setModel(model_);
  ui->param_manage_tree_view->setAlternatingRowColors(true); // 间隔变色
  ui->param_manage_tree_view->header()->resizeSection(0, 200);
  ui->param_manage_tree_view->header()->resizeSection(5, 200);
  ui->param_manage_tree_view->setSelectionBehavior(QTreeView::SelectRows);
  ui->param_manage_tree_view->setSelectionMode(QTreeView::SingleSelection);
  ui->param_manage_tree_view->setContextMenuPolicy(Qt::CustomContextMenu);
  ui->param_manage_tree_view->expandAll();

  // 设置委托
  NameItemDelegate *name_delegate = new NameItemDelegate(ui->param_manage_tree_view);
  ui->param_manage_tree_view->setItemDelegateForColumn(0, name_delegate);
  TypeItemDelegate *type_delegate = new TypeItemDelegate(ui->param_manage_tree_view);
  ui->param_manage_tree_view->setItemDelegateForColumn(1, type_delegate);
  LengthItemDelegate *length_delegate = new LengthItemDelegate(ui->param_manage_tree_view);
  ui->param_manage_tree_view->setItemDelegateForColumn(2, length_delegate);
  RangeItemDelegate *min_delegate = new RangeItemDelegate(ui->param_manage_tree_view);
  ui->param_manage_tree_view->setItemDelegateForColumn(3, min_delegate);
  RangeItemDelegate *max_delegate = new RangeItemDelegate(ui->param_manage_tree_view);
  ui->param_manage_tree_view->setItemDelegateForColumn(4, max_delegate);
  ValueItemDelegate *value_delegate = new ValueItemDelegate(ui->param_manage_tree_view);
  ui->param_manage_tree_view->setItemDelegateForColumn(5, value_delegate);

  auto getData = [=](QString data_type, bool is_array, QString value) -> std::string {
    std::string data;
    DATA_TYPE dtype = str_data_type_map.value(data_type);
    switch (dtype) {
    case INT32: {
      if (!is_array) {
        int32_t type_value = value.toInt();
        uint8_t byte_array[sizeof(int32_t)];
        std::memcpy(byte_array, &type_value, sizeof(int32_t));
        data = std::string(reinterpret_cast<char *>(byte_array), sizeof(byte_array) / sizeof(uint8_t));
      } else {
        QStringList value_list = value.remove(QChar(' ', Qt::CaseInsensitive)).split(",");
        QVector<int32_t> value_vec;
        for (int i = 0; i < value_list.length(); ++i) {
          value_vec.append(value_list.at(i).toInt());
        }
        int data_size = sizeof(int32_t) * value_vec.length();
        uint8_t byte_array[data_size];
        std::memcpy(byte_array, value_vec.data(), data_size);
        data = std::string(reinterpret_cast<char *>(byte_array), sizeof(byte_array) / sizeof(uint8_t));
      }
      break;
    }
    case UINT32: {
      if (!is_array) {
        uint32_t type_value = value.toUInt();
        uint8_t byte_array[sizeof(uint32_t)];
        std::memcpy(byte_array, &type_value, sizeof(uint32_t));
        data = std::string(reinterpret_cast<char *>(byte_array), sizeof(byte_array) / sizeof(uint8_t));
      } else {
        QStringList value_list = value.remove(QChar(' ', Qt::CaseInsensitive)).split(",");
        QVector<uint32_t> value_vec;
        for (int i = 0; i < value_list.length(); ++i) {
          value_vec.append(value_list.at(i).toUInt());
        }
        int data_size = sizeof(uint32_t) * value_vec.length();
        uint8_t byte_array[data_size];
        std::memcpy(byte_array, value_vec.data(), data_size);
        data = std::string(reinterpret_cast<char *>(byte_array), sizeof(byte_array) / sizeof(uint8_t));
      }
      break;
    }
    case INT64: {
      if (!is_array) {
        int64_t type_value = value.toLongLong();
        uint8_t byte_array[sizeof(int64_t)];
        std::memcpy(byte_array, &type_value, sizeof(int64_t));
        data = std::string(reinterpret_cast<char *>(byte_array), sizeof(byte_array) / sizeof(uint8_t));
      } else {
        QStringList value_list = value.remove(QChar(' ', Qt::CaseInsensitive)).split(",");
        QVector<int64_t> value_vec;
        for (int i = 0; i < value_list.length(); ++i) {
          value_vec.append(value_list.at(i).toLongLong());
        }
        int data_size = sizeof(int64_t) * value_vec.length();
        uint8_t byte_array[data_size];
        std::memcpy(byte_array, value_vec.data(), data_size);
        data = std::string(reinterpret_cast<char *>(byte_array), sizeof(byte_array) / sizeof(uint8_t));
      }
      break;
    }
    case UINT64: {
      if (!is_array) {
        uint64_t type_value = value.toULongLong();
        uint8_t byte_array[sizeof(uint64_t)];
        std::memcpy(byte_array, &type_value, sizeof(uint64_t));
        data = std::string(reinterpret_cast<char *>(byte_array), sizeof(byte_array) / sizeof(uint8_t));
      } else {
        QStringList value_list = value.remove(QChar(' ', Qt::CaseInsensitive)).split(",");
        QVector<uint64_t> value_vec;
        for (int i = 0; i < value_list.length(); ++i) {
          value_vec.append(value_list.at(i).toULongLong());
        }
        int data_size = sizeof(uint64_t) * value_vec.length();
        uint8_t byte_array[data_size];
        std::memcpy(byte_array, value_vec.data(), data_size);
        data = std::string(reinterpret_cast<char *>(byte_array), sizeof(byte_array) / sizeof(uint8_t));
      }
      break;
    }
    case FLOAT32: {
      if (!is_array) {
        float type_value = value.toFloat();
        uint8_t byte_array[sizeof(float)];
        std::memcpy(byte_array, &type_value, sizeof(float));
        data = std::string(reinterpret_cast<char *>(byte_array), sizeof(byte_array) / sizeof(uint8_t));
      } else {
        QStringList value_list = value.remove(QChar(' ', Qt::CaseInsensitive)).split(",");
        QVector<float> value_vec;
        for (int i = 0; i < value_list.length(); ++i) {
          value_vec.append(value_list.at(i).toFloat());
        }
        int data_size = sizeof(float) * value_vec.length();
        uint8_t byte_array[data_size];
        std::memcpy(byte_array, value_vec.data(), data_size);
        data = std::string(reinterpret_cast<char *>(byte_array), sizeof(byte_array) / sizeof(uint8_t));
      }
      break;
    }
    case FLOAT64: {
      if (!is_array) {
        double type_value = value.toDouble();
        uint8_t byte_array[sizeof(double)];
        std::memcpy(byte_array, &type_value, sizeof(double));
        data = std::string(reinterpret_cast<char *>(byte_array), sizeof(byte_array) / sizeof(uint8_t));
      } else {
        QStringList value_list = value.remove(QChar(' ', Qt::CaseInsensitive)).split(",");
        QVector<double> value_vec;
        for (int i = 0; i < value_list.length(); ++i) {
          value_vec.append(value_list.at(i).toDouble());
        }
        int data_size = sizeof(double) * value_vec.length();
        uint8_t byte_array[data_size];
        std::memcpy(byte_array, value_vec.data(), data_size);
        data = std::string(reinterpret_cast<char *>(byte_array), sizeof(byte_array) / sizeof(uint8_t));
      }
      break;
    }
    case CHAR: {
      if (!is_array) {
        char type_value = value.at(0).toLatin1();
        uint8_t byte_array[sizeof(char)];
        std::memcpy(byte_array, &type_value, sizeof(char));
        data = std::string(reinterpret_cast<char *>(byte_array), sizeof(byte_array) / sizeof(uint8_t));
      } else {
        QStringList value_list = value.remove(QChar(' ', Qt::CaseInsensitive)).split(",");
        QVector<char> value_vec;
        for (int i = 0; i < value_list.length(); ++i) {
          value_vec.append(value_list.at(i).at(0).toLatin1());
        }
        int data_size = sizeof(char) * value_vec.length();
        uint8_t byte_array[data_size];
        std::memcpy(byte_array, value_vec.data(), data_size);
        data = std::string(reinterpret_cast<char *>(byte_array), sizeof(byte_array) / sizeof(uint8_t));
      }
      break;
    }
    case UCHAR: {
      if (!is_array) {
        uchar type_value = value.toUInt();
        uint8_t byte_array[sizeof(uchar)];
        std::memcpy(byte_array, &type_value, sizeof(uchar));
        data = std::string(reinterpret_cast<char *>(byte_array), sizeof(byte_array) / sizeof(uint8_t));
      } else {
        QStringList value_list = value.remove(QChar(' ', Qt::CaseInsensitive)).split(",");
        QVector<char> value_vec;
        for (int i = 0; i < value_list.length(); ++i) {
          value_vec.append(value_list.at(i).toUInt());
        }
        int data_size = sizeof(uchar) * value_vec.length();
        uint8_t byte_array[data_size];
        std::memcpy(byte_array, value_vec.data(), data_size);
        data = std::string(reinterpret_cast<char *>(byte_array), sizeof(byte_array) / sizeof(uint8_t));
      }
      break;
    }
    default:
      break;
    }
    return data;
  };

  auto setData = [=](QStandardItem *item, syt_msgs::srv::ParamManage::Response response) -> QString {
    if (item->parent()->child(item->row(), 0)->text() != response.field.c_str()) {
      return item->parent()->child(item->row(), 5)->text();
    }
    QString data_str;
    bool is_array = item->parent()->child(item->row(), 2)->text().toInt() > 1 ? true : false;
    switch (response.dtype.data) {
    case INT32: {
      if (!is_array) {
        int32_t data;
        memcpy(&data, response.data.data(), response.data.size());
        data_str = QString::number(data);
      } else {
        size_t length = item->parent()->child(item->row(), 2)->text().toInt();
        int32_t data_arr[length];
        memcpy(data_arr, response.data.data(), response.data.size());
        QStringList data_list;
        for (int i = 0; i < length; ++i) {
          data_list << QString::number(data_arr[i]);
        }
        data_str = data_list.join(", ");
      }
      break;
    }
    case UINT32: {
      if (!is_array) {
        uint32_t data;
        memcpy(&data, response.data.data(), response.data.size());
        data_str = QString::number(data);
      } else {
        size_t length = item->parent()->child(item->row(), 2)->text().toInt();
        uint32_t data_arr[length];
        memcpy(data_arr, response.data.data(), response.data.size());
        QStringList data_list;
        for (int i = 0; i < length; ++i) {
          data_list << QString::number(data_arr[i]);
        }
        data_str = data_list.join(", ");
      }
      break;
    }
    case INT64: {
      if (!is_array) {
        int64_t data;
        memcpy(&data, response.data.data(), response.data.size());
        data_str = QString::number(data);
      } else {
        size_t length = item->parent()->child(item->row(), 2)->text().toInt();
        int64_t data_arr[length];
        memcpy(data_arr, response.data.data(), response.data.size());
        QStringList data_list;
        for (int i = 0; i < length; ++i) {
          data_list << QString::number(data_arr[i]);
        }
        data_str = data_list.join(", ");
      }
      break;
    }
    case UINT64: {
      if (!is_array) {
        uint64_t data;
        memcpy(&data, response.data.data(), response.data.size());
        data_str = QString::number(data);
      } else {
        size_t length = item->parent()->child(item->row(), 2)->text().toInt();
        uint64_t data_arr[length];
        memcpy(data_arr, response.data.data(), response.data.size());
        QStringList data_list;
        for (int i = 0; i < length; ++i) {
          data_list << QString::number(data_arr[i]);
        }
        data_str = data_list.join(", ");
      }
      break;
    }
    case FLOAT32: {
      if (!is_array) {
        float data;
        memcpy(&data, response.data.data(), response.data.size());
        data_str = QString::number(data);
      } else {
        size_t length = item->parent()->child(item->row(), 2)->text().toInt();
        float data_arr[length];
        memcpy(data_arr, response.data.data(), response.data.size());
        QStringList data_list;
        for (int i = 0; i < length; ++i) {
          data_list << QString::number(data_arr[i]);
        }
        data_str = data_list.join(", ");
      }
      break;
    }
    case FLOAT64: {
      if (!is_array) {
        double data;
        memcpy(&data, response.data.data(), response.data.size());
        data_str = QString::number(data);
      } else {
        size_t length = item->parent()->child(item->row(), 2)->text().toInt();
        double data_arr[length];
        memcpy(data_arr, response.data.data(), response.data.size());
        QStringList data_list;
        for (int i = 0; i < length; ++i) {
          data_list << QString::number(data_arr[i]);
        }
        data_str = data_list.join(", ");
      }
      break;
    }
    case CHAR: {
      if (!is_array) {
        char data;
        memcpy(&data, response.data.data(), response.data.size());
        data_str = QString::number(data);
      } else {
        size_t length = item->parent()->child(item->row(), 2)->text().toInt();
        char data_arr[length];
        memcpy(data_arr, response.data.data(), response.data.size());
        QStringList data_list;
        for (int i = 0; i < length; ++i) {
          data_list << QString::number(data_arr[i]);
        }
        data_str = data_list.join(", ");
      }
      break;
    }
    case UCHAR: {
      if (!is_array) {
        uchar data;
        memcpy(&data, response.data.data(), response.data.size());
        data_str = QString::number(data);
      } else {
        size_t length = item->parent()->child(item->row(), 2)->text().toInt();
        uchar data_arr[length];
        memcpy(data_arr, response.data.data(), response.data.size());
        QStringList data_list;
        for (int i = 0; i < length; ++i) {
          data_list << QString::number(data_arr[i]);
        }
        data_str = data_list.join(", ");
      }
      break;
    }
    case 8: {
      QByteArray byteArray(reinterpret_cast<const char*>(response.data.data()), response.data.size());
      data_str = QString::fromUtf8(byteArray);
      break;
    }
    default:
      break;
    }
    return data_str;
  };


  // 右键菜单
  connect(ui->param_manage_tree_view, &QTreeView::customContextMenuRequested, this, [=](const QPoint &pos) {
    QMenu menu;
    QModelIndex cur_index = ui->param_manage_tree_view->indexAt(pos); // 当前点击的元素的index
    QModelIndex index = cur_index.sibling(cur_index.row(), 0);        // 该行的第1列元素的index
    if (index.isValid()) {
      QStandardItem *item = model_->itemFromIndex(index);
      int item_level = getItemLevel(item);
      switch (item_level) {
      case 0:
        menu.addAction(tr("写入参数组"), this, [=]() {
          waiting_spinner_widget_->start();
          QFuture<void> exec_state = QtConcurrent::run([=](){
          stop_param_process_ = false;
          param_processing_ = true;
          for (int j = 0; j < item->rowCount(); ++j) {
            if (stop_param_process_) {
              break;
            }
            error_param_name = item->child(j, 0)->text().trimmed();
            ParamLine param_line;
            param_line.name = item->child(j, 0)->text().trimmed();
            param_line.dtype = item->child(j, 1)->text().trimmed();
            param_line.length = item->child(j, 2)->text().trimmed().toInt();
            param_line.min_range = item->child(j, 3)->text().trimmed();
            param_line.max_range = item->child(j, 4)->text().trimmed();
            param_line.value = item->child(j, 5)->text().trimmed();
            QStringList value_list = item->child(j, 5)->text().split(",");
            if (value_list.size() > 1) {
              param_line.is_array = true;
            } else {
              param_line.is_array = false;
            }
            param_line.comment = item->child(j, 6)->text();

            DATA_TYPE dtype = str_data_type_map.value(param_line.dtype);
            std::string data = getData(param_line.dtype, param_line.is_array, param_line.value);

            if (item->text() == "load_machine") {
              rclcomm_->loadMachineParam(0, dtype, param_line.name.toStdString(), data, param_line.is_array);
            } else if (item->text() == "compose_machine") {
              rclcomm_->composeMachineParam(0, dtype, param_line.name.toStdString(), data, param_line.is_array);
            } else if (item->text() == "sewing_machine") {
              rclcomm_->sewingMachineParam(0, dtype, param_line.name.toStdString(), data, param_line.is_array);
            } else {
              rclcomm_->updateParam(item->text().toStdString(), dtype, param_line.name, param_line.value, param_line.is_array);
            }
          }});
          QtConcurrent::run([=](){
            bool success = exec_state.isFinished();
            while (rclcomm_->returntrycount() < 5) {
              if (success) {
                break;
              }
              success = exec_state.isFinished();
            }
            stop_param_process_ = true;
            std::this_thread::sleep_for(50ms);
            while (true) {
              if (exec_state.isFinished()) {
                break;
              }
              std::this_thread::sleep_for(50ms);
            }
            param_processing_ = false;
            emit signParamProcessFinish(success);
          });
        });
        menu.addAction(tr("读取参数组"), this, [=](){
          waiting_spinner_widget_->start();
          param_processing_ = true;
          stop_param_process_ = false;
          QFuture<void> exec_state = QtConcurrent::run([=](){
            if (item->hasChildren()) {
              for (int j = 0; j < item->rowCount(); ++j) {
                if (stop_param_process_) {
                  break;
                }
                error_param_name = item->child(j, 0)->text().trimmed();
                ParamLine param_line;
                param_line.name = item->child(j, 0)->text().trimmed();
                param_line.dtype = item->child(j, 1)->text().trimmed();
                param_line.length = item->child(j, 2)->text().trimmed().toInt();
                param_line.min_range = item->child(j, 3)->text().trimmed();
                param_line.max_range = item->child(j, 4)->text().trimmed();
                param_line.value = item->child(j, 5)->text().trimmed();
                QStringList value_list = item->child(j, 5)->text().split(",");
                if (value_list.size() > 1) {
                  param_line.is_array = true;
                } else {
                  param_line.is_array = false;
                }
                param_line.comment = item->child(j, 6)->text();
                
                DATA_TYPE dtype = str_data_type_map.value(param_line.dtype);
                std::string data = getData(param_line.dtype, param_line.is_array, param_line.value);

                if (item->text() == "load_machine") {
                  auto response = rclcomm_->loadMachineParam(1, dtype, param_line.name.toStdString(), data, param_line.is_array);
                  QString data_str = setData(item->child(j), response);
                  item->child(j, 5)->setText(data_str);
                } else if (item->text() == "compose_machine") {
                  auto response = rclcomm_->composeMachineParam(1, dtype, param_line.name.toStdString(), data, param_line.is_array);
                  QString data_str = setData(item->child(j), response);
                  item->child(j, 5)->setText(data_str);
                } else if (item->text() == "sewing_machine") {
                  auto response = rclcomm_->sewingMachineParam(1, dtype, param_line.name.toStdString(), data, param_line.is_array);
                  QString data_str = setData(item->child(j), response);
                  item->child(j, 5)->setText(data_str);
                }
              }
            }
          });
          QtConcurrent::run([=](){
            bool success = exec_state.isFinished();
            while (rclcomm_->returntrycount() < 5) {
              if (success) {
                break;
              }
              success = exec_state.isFinished();
            }
            stop_param_process_ = true;
            std::this_thread::sleep_for(50ms);
            while (true) {
              if (exec_state.isFinished()) {
                break;
              }
              std::this_thread::sleep_for(50ms);
            }
            param_processing_ = false;
            emit signParamProcessFinish(success);
          });
        });
        menu.addAction(tr("增加参数组"), this, [=]() {
          QStandardItem *new_group = new QStandardItem("group");
          QList<QStandardItem *> new_items;
          new_items << new_group;
          model_->appendRow(new_items);
        });
        menu.addAction(tr("增加参数"), this, [=]() {
          QStandardItem *new_param = new QStandardItem("name");
          QStandardItem *new_type = new QStandardItem("uint32");
          QStandardItem *new_length = new QStandardItem("1");
          QStandardItem *new_min = new QStandardItem("0");
          QStandardItem *new_max = new QStandardItem("inf");
          QStandardItem *new_value = new QStandardItem("0");
          QStandardItem *new_comment = new QStandardItem("参数注释");

          QList<QStandardItem *> new_items;
          new_items << new_param << new_type << new_length << new_min << new_max << new_value << new_comment;
          item->appendRow(new_items);
        });
        menu.addAction(tr("删除参数组"), this, [=]() {
          bool ret = showMessageBox(this, WARN, tr("确认删除当前组？"), 2, {tr("确认"), tr("取消")});
          if (ret == 0) {
            model_->removeRow(index.row());
          }
        });
        break;
      case 1:
        menu.addAction(tr("写入参数"), this, [=]() {
          waiting_spinner_widget_->start();
          stop_param_process_ = false;
          QString machine_name = item->parent()->text();
          QStandardItem* parentItem = item->parent();
          int row = item->row();
          QStandardItem* childItem = parentItem->child(row);
          error_param_name = childItem->text().trimmed();
          ParamLine param_line;
          param_line.name = childItem->text().trimmed();
          param_line.dtype = parentItem->child(row, 1)->text().trimmed();
          param_line.length = parentItem->child(row, 2)->text().trimmed().toInt();
          param_line.min_range = parentItem->child(row, 3)->text().trimmed();
          param_line.max_range = parentItem->child(row, 4)->text().trimmed();
          param_line.value = parentItem->child(row, 5)->text().trimmed();
          QStringList value_list = param_line.value.split(",");
          if (value_list.size() > 1) {
            param_line.is_array = true;
          } else {
            param_line.is_array = false;
          }
          param_line.comment = item->parent()->child(item->row(), 6)->text();

          DATA_TYPE dtype = str_data_type_map.value(param_line.dtype);
          std::string data = getData(param_line.dtype, param_line.is_array, param_line.value);
          param_processing_ = true;
          QFuture <void> exec_state = QtConcurrent::run([=](){
            if(!stop_param_process_) {
              if (machine_name == "load_machine") {
                  rclcomm_->loadMachineParam(0, dtype, param_line.name.toStdString(), data, param_line.is_array);
              } else if (machine_name == "compose_machine") {
                  rclcomm_->composeMachineParam(0, dtype, param_line.name.toStdString(), data, param_line.is_array);
              } else if (machine_name == "sewing_machine") {
                  rclcomm_->sewingMachineParam(0, dtype, param_line.name.toStdString(), data, param_line.is_array);
              } else {
                  rclcomm_->updateParam(machine_name.toStdString(), dtype, param_line.name, param_line.value, param_line.is_array);
              }
            }
          });
          QtConcurrent::run([=](){
            bool success = exec_state.isFinished();
            while(rclcomm_->returntrycount() < 5) {
              if(success) {
                break;
              }
              success = exec_state.isFinished();
            }
            stop_param_process_ = true;
            std::this_thread::sleep_for(50ms);
            while(true) {
              if(exec_state.isFinished()) {
                break;
              }
              std::this_thread::sleep_for(50ms);
            }
            param_processing_ = false;
            emit signParamProcessFinish(success);
          });
        });
        menu.addAction(tr("读取参数"), this, [=](){
          waiting_spinner_widget_->start();
          stop_param_process_ = false;
          // ParamLine param_line;
          // param_line.name = item->parent()->child(item->row(), 0)->text().trimmed();
          // param_line.dtype = item->parent()->child(item->row(), 1)->text().trimmed();
          // param_line.length = item->parent()->child(item->row(), 2)->text().trimmed().toInt();
          // param_line.min_range = item->parent()->child(item->row(), 3)->text().trimmed();
          // param_line.max_range = item->parent()->child(item->row(), 4)->text().trimmed();
          // param_line.value = item->parent()->child(item->row(), 5)->text().trimmed();
          // QStringList value_list = item->parent()->child(item->row(), 5)->text().split(",");
          QStandardItem* parentItem = item->parent();
          int row = item->row();
          QStandardItem* childItem = parentItem->child(row);
          error_param_name = childItem->text().trimmed();
          ParamLine param_line;
          param_line.name = childItem->text().trimmed();
          param_line.dtype = parentItem->child(row, 1)->text().trimmed();
          param_line.length = parentItem->child(row, 2)->text().trimmed().toInt();
          param_line.min_range = parentItem->child(row, 3)->text().trimmed();
          param_line.max_range = parentItem->child(row, 4)->text().trimmed();
          param_line.value = parentItem->child(row, 5)->text().trimmed();
          QStringList value_list = param_line.value.split(",");
          if (value_list.size() > 1) {
            param_line.is_array = true;
          } else {
            param_line.is_array = false;
          }
          param_line.comment = item->parent()->child(item->row(), 6)->text();

          DATA_TYPE dtype = str_data_type_map.value(param_line.dtype);
          std::string data = getData(param_line.dtype, param_line.is_array, param_line.value);
          param_processing_ = true;
          QFuture<void> exec_state = QtConcurrent::run([=](){
            if(!stop_param_process_){
              if (item->parent()->text() == "load_machine") {
                auto response = rclcomm_->loadMachineParam(1, dtype, param_line.name.toStdString(), data, param_line.is_array);
                QString data_str = setData(item->parent()->child(item->row()), response);
                item->parent()->child(item->row(), 5)->setText(data_str);
              } else if (item->parent()->text() == "compose_machine") {
                auto response = rclcomm_->composeMachineParam(1, dtype, param_line.name.toStdString(), data, param_line.is_array);
                QString data_str = setData(item->parent()->child(item->row()), response);
                item->parent()->child(item->row(), 5)->setText(data_str);
              } else if (item->parent()->text() == "sewing_machine") {
                auto response = rclcomm_->sewingMachineParam(1, dtype, param_line.name.toStdString(), data, param_line.is_array);
                QString data_str = setData(item->parent()->child(item->row()), response);
                item->parent()->child(item->row(), 5)->setText(data_str);
              } else {
                QString machine_name = item->parent()->text();
                auto value = rclcomm_->readParam(machine_name.toStdString(), dtype, param_line.name, param_line.is_array);
                syt_msgs::srv::ParamManage::Response response;
                response.success = true;
                response.dtype.data = 8;
                response.field = param_line.name.toStdString();
                response.data = value;
                QString data_str = setData(item->parent()->child(item->row()), response);
                item->parent()->child(item->row(), 5)->setText(data_str);
              }
            }
          });
          QtConcurrent::run([=](){
            bool success = exec_state.isFinished();
            while (rclcomm_->returntrycount() < 5) {
              if (success) {
                break;
              }
              success = exec_state.isFinished();
            }
            stop_param_process_ = true;
            std::this_thread::sleep_for(50ms);
            while (true) {
              if (exec_state.isFinished()) {
                break;
              }
              std::this_thread::sleep_for(50ms);
            }
            param_processing_ = false;
            emit signParamProcessFinish(success);
          });
        });
        menu.addAction(tr("增加参数"), this, [=]() {
          QStandardItem *new_param = new QStandardItem("name");
          QStandardItem *new_type = new QStandardItem("uchar");
          QStandardItem *new_length = new QStandardItem("1");
          QStandardItem *new_min = new QStandardItem("0");
          QStandardItem *new_max = new QStandardItem("inf");
          QStandardItem *new_value = new QStandardItem("1");
          QStandardItem *new_comment = new QStandardItem("参数注释");

          QList<QStandardItem *> new_items;
          new_items << new_param << new_type << new_length << new_min << new_max << new_value << new_comment;
          item->parent()->appendRow(new_items);
        });
        menu.addAction(tr("删除参数"), this, [=]() { item->parent()->removeRow(index.row()); });
        break;
      }
    }
    menu.exec(QCursor::pos()); // 显示菜单
  });

  ui->save_param_btn->setShortcut(QKeySequence(QLatin1String("Ctrl+S")));

  // 参数界面信号绑定
  connect(ui->unfold_all_btn, &QPushButton::clicked, [=]() { ui->param_manage_tree_view->expandAll(); });
  connect(ui->fold_all_btn, &QPushButton::clicked, [=]() { ui->param_manage_tree_view->collapseAll(); });
  connect(ui->save_param_btn, &QPushButton::clicked, [=]() {
    if (dumpJson(current_param_file_, model_) == false) {
      showMessageBox(this, ERROR, tr("保存失败"), 1, {tr("确认")});
    } else {
      showMessageBox(this, SUCCESS, tr("保存成功"), 1, {tr("确认")});
    }
  });
  connect(ui->import_param_btn, &QPushButton::clicked, [=]() {
    QString file_name = QFileDialog::getOpenFileName(this, tr("导入参数文件"));
    if (file_name.isEmpty()) {
      return;
    }
    if (!readJson(file_name, model_)) {
      showMessageBox(this, ERROR, tr("文件导入失败。"), 1, {tr("确认")});
    } else {
      current_param_file_ = file_name;
      showMessageBox(this, SUCCESS, tr("文件导入成功。"), 1, {tr("确认")});
      ui->param_manage_tree_view->setAlternatingRowColors(true); // 间隔变色
      ui->param_manage_tree_view->header()->resizeSection(0, 200);
      ui->param_manage_tree_view->header()->resizeSection(5, 200);
      ui->param_manage_tree_view->setSelectionBehavior(QTreeView::SelectRows);
      ui->param_manage_tree_view->setSelectionMode(QTreeView::SingleSelection);
      ui->param_manage_tree_view->setContextMenuPolicy(Qt::CustomContextMenu);
      ui->param_manage_tree_view->expandAll();
    }
  });
  connect(ui->export_param_btn, &QPushButton::clicked, [=]() {
    QString file_name = QFileDialog::getSaveFileName(this, tr("导出参数文件"));
    if (file_name.isEmpty()) {
      return;
    }
    if (!file_name.endsWith(".json", Qt::CaseInsensitive)) {
      file_name.append(".json");
    }
    if (dumpJson(file_name, model_)) {
      showMessageBox(this, SUCCESS, tr("保存成功。"), 1, {tr("确认")});
    } else {
      showMessageBox(this, ERROR, tr("保存失败。"), 1, {tr("确认")});
    }
  });
  connect(ui->load_param_btn, &QPushButton::clicked, [=]() {
    waiting_spinner_widget_->start();
    QStandardItem *root_item = model_->invisibleRootItem();
    QList<QFuture<void>> state_list;
    param_processing_ = true;
    stop_param_process_ = false;
    for (int i = 0; i < root_item->rowCount(); ++i) {
      QStandardItem *item = root_item->child(i);
      if (item->hasChildren()) {
        state_list.push_back(QtConcurrent::run([=](){
          for (int j = 0; j < item->rowCount(); ++j) {
            if (stop_param_process_) {
              break;
            }
            error_param_name = item->child(j, 0)->text().trimmed();
            ParamLine param_line;
            param_line.name = item->child(j, 0)->text().trimmed();
            param_line.dtype = item->child(j, 1)->text().trimmed();
            param_line.length = item->child(j, 2)->text().trimmed().toInt();
            param_line.min_range = item->child(j, 3)->text().trimmed();
            param_line.max_range = item->child(j, 4)->text().trimmed();
            param_line.value = item->child(j, 5)->text().trimmed();
            QStringList value_list = item->child(j, 5)->text().split(",");
            if (value_list.size() > 1) {
              param_line.is_array = true;
            } else {
              param_line.is_array = false;
            }
            param_line.comment = item->child(j, 6)->text();

            DATA_TYPE dtype = str_data_type_map.value(param_line.dtype);
            std::string data = getData(param_line.dtype, param_line.is_array, param_line.value);

            if (item->text() == "load_machine") {
              auto response = rclcomm_->loadMachineParam(1, dtype, param_line.name.toStdString(), data, param_line.is_array);
              QString data_str = setData(item->child(j), response);
              item->child(j, 5)->setText(data_str);
            } else if (item->text() == "compose_machine") {
              auto response = rclcomm_->composeMachineParam(1, dtype, param_line.name.toStdString(), data, param_line.is_array);
              QString data_str = setData(item->child(j), response);
              item->child(j, 5)->setText(data_str);
            } else if (item->text() == "sewing_machine") {
              auto response = rclcomm_->sewingMachineParam(1, dtype, param_line.name.toStdString(), data, param_line.is_array);
              QString data_str = setData(item->child(j), response);
              item->child(j, 5)->setText(data_str);
            }
          }
        }));
      }
    }

    auto checkFinished = [=]() {
      for (auto exec_state : state_list) {
        if (!exec_state.isFinished()) {
          return false;
        }
      }
      return true;
    };
    QtConcurrent::run([=](){
      auto time_begin = std::chrono::steady_clock::now();
      auto time_end = std::chrono::steady_clock::now();
      int time_cost = std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_begin).count();
      bool success = checkFinished();
      while (time_cost < 5000) {
        if (success) {
          break;
        }
        std::this_thread::sleep_for(50ms);
        time_end = std::chrono::steady_clock::now();
        time_cost = std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_begin).count();
        success = checkFinished();
      }
      stop_param_process_ = true;
      while (true) {
        if (checkFinished()) {
          break;
        }
        std::this_thread::sleep_for(50ms);
      }
      param_processing_ = false;
      emit signParamProcessFinish(success);
    });
  });
  connect(ui->apply_param_btn, &QPushButton::clicked, [=]() {
    QStandardItem *root_item = model_->invisibleRootItem();
    QList<QFuture<void>> state_list;
    param_processing_ = true;
    stop_param_process_ = false;
    waiting_spinner_widget_->start();
    for (int i = 0; i < root_item->rowCount(); ++i) {
      QStandardItem *item = root_item->child(i);
      if (item->hasChildren()) {
        state_list.push_back(QtConcurrent::run([=](){
          for (int j = 0; j < item->rowCount(); ++j) {
            if (stop_param_process_) {
              break;
            }
            error_param_name = item->child(j, 0)->text().trimmed();
            ParamLine param_line;
            param_line.name = item->child(j, 0)->text().trimmed();
            param_line.dtype = item->child(j, 1)->text().trimmed();
            param_line.length = item->child(j, 2)->text().trimmed().toInt();
            param_line.min_range = item->child(j, 3)->text().trimmed();
            param_line.max_range = item->child(j, 4)->text().trimmed();
            param_line.value = item->child(j, 5)->text().trimmed();
            QStringList value_list = item->child(j, 5)->text().split(",");
            if (value_list.size() > 1) {
              param_line.is_array = true;
            } else {
              param_line.is_array = false;
            }
            param_line.comment = item->child(j, 6)->text();

            DATA_TYPE dtype = str_data_type_map.value(param_line.dtype);
            std::string data = getData(param_line.dtype, param_line.is_array, param_line.value);

            if (item->text() == "load_machine") {
              rclcomm_->loadMachineParam(0, dtype, param_line.name.toStdString(), data, param_line.is_array);
            } else if (item->text() == "compose_machine") {
              rclcomm_->composeMachineParam(0, dtype, param_line.name.toStdString(), data, param_line.is_array);
            } else if (item->text() == "sewing_machine") {
              rclcomm_->sewingMachineParam(0, dtype, param_line.name.toStdString(), data, param_line.is_array);
            }
          }
        }));
      }
    }
    auto checkFinished = [=]() {
      for (auto exec_state : state_list) {
        if (!exec_state.isFinished()) {
          return false;
        }
      }
      return true;
    };
    QtConcurrent::run([=](){
      auto time_begin = std::chrono::steady_clock::now();
      auto time_end = std::chrono::steady_clock::now();
      int time_cost = std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_begin).count();
      bool success = checkFinished();
      while (time_cost < 5000) {
        if (success) {
          break;
        }
        std::this_thread::sleep_for(50ms);
        time_end = std::chrono::steady_clock::now();
        time_cost = std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_begin).count();
        success = checkFinished();
      }
      for (auto exec_state : state_list) {
        if (!exec_state.isFinished()) {
          exec_state.cancel();
        }
      }
      stop_param_process_ = true;
      while (true) {
        if (checkFinished()) {
          break;
        }
      }
      param_processing_ = false;
      emit signParamProcessFinish(success);
    });
  });
}

// 开发者界面
void MainWindow::setDeveloperWidget() {
  developer_widget_ = new DeveloperWidget(this);

  // 绑定机器控制按钮
  bindControlConnection();
}

void MainWindow::setParamSetWidget() {
  param_set_widget_ = new ParamSetWidget(this);

  // 绑定参数配置所需功能
  bindParamSetConnection();
}

void MainWindow::showLoadMachineImage() {
  if (!pix_A_left_.isNull()) {
    ui->A_left_visual_label->clear();
    pix_A_left_ = pix_A_left_.scaled(ui->A_left_visual_label->width(), ui->A_left_visual_label->height(), Qt::KeepAspectRatio);
    ui->A_left_visual_label->setPixmap(pix_A_left_);
    ui->A_left_visual_label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    ui->A_left_visual_label->update();
  }

  if (!pix_A_right_.isNull()) {
    ui->A_right_visual_label->clear();
    pix_A_right_ = pix_A_right_.scaled(ui->A_right_visual_label->width(), ui->A_right_visual_label->height(), Qt::KeepAspectRatio);
    ui->A_right_visual_label->setPixmap(pix_A_right_);
    ui->A_right_visual_label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    ui->A_right_visual_label->update();
  }

  if (!pix_B_left_.isNull()) {
    ui->B_left_visual_label->clear();
    pix_B_left_ = pix_B_left_.scaled(ui->B_left_visual_label->width(), ui->B_left_visual_label->height(), Qt::KeepAspectRatio);
    ui->B_left_visual_label->setPixmap(pix_B_left_);
    ui->B_left_visual_label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    ui->B_left_visual_label->update();
  }

  if (!pix_B_right_.isNull()) {
    ui->B_right_visual_label->clear();
    pix_B_right_ = pix_B_right_.scaled(ui->B_right_visual_label->width(), ui->B_right_visual_label->height(), Qt::KeepAspectRatio);
    ui->B_right_visual_label->setPixmap(pix_B_right_);
    ui->B_right_visual_label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    ui->B_right_visual_label->update();
  }
}

bool MainWindow::readJson(const QString &json_file, QStandardItemModel *model) {
  model->clear();
  // 设置表头
  model->setHorizontalHeaderLabels(QStringList() << tr("属性") << tr("类型") << tr("长度") << tr("最小值") << tr("最大值") << tr("值") << tr("释义"));

  // 打开JSON文件
  QFile file(json_file);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    qDebug() << "无法打开参数文件";
    return false;
  }

  // 读取JSON文件内容
  QByteArray json_data = file.readAll();

  // 解析JSON数据
  QJsonDocument json_doc = QJsonDocument::fromJson(json_data);
  if (!json_doc.isNull() && json_doc.isObject()) {
    QJsonObject json_obj = json_doc.object(); // json对象

    QStringList top_keys = json_obj.keys(); // 所有的keys

    for (auto i = top_keys.constBegin(); i != top_keys.constEnd(); ++i) {
      QJsonObject machine_obj = json_obj.value(*i).toObject(); // 当前机器
      QStandardItem *machine_item = new QStandardItem(*i);
      model->appendRow(machine_item);

      // 遍历每个机器的参数
      QStringList second_keys = machine_obj.keys();
      for (auto j = second_keys.constBegin(); j != second_keys.constEnd(); ++j) {
        QJsonObject key_obj = machine_obj.value(*j).toObject();

        QStandardItem *key_item = new QStandardItem(*j);                                    // 属性
        key_item->setEditable(false);
        QStandardItem *dtype_item = new QStandardItem(key_obj.value("dtype").toString());   // 类型
        dtype_item->setEditable(false);
        QStandardItem *length_item = new QStandardItem(key_obj.value("length").toString()); // 长度
        length_item->setEditable(false);
        QStandardItem *min_item = new QStandardItem(key_obj.value("min").toString());       // 最小值
        length_item->setEditable(false);
        QStandardItem *max_item = new QStandardItem(key_obj.value("max").toString());       // 最大值
        max_item->setEditable(false);
        QStandardItem *value_item;                                                          // 值
        if (length_item->text().toInt() > 1) {
          QJsonArray value_array = key_obj.value("value").toArray();

          QStringList value_list;
          for (const QJsonValue &value : value_array) {
            value_list.append(value.toString());
          }
          value_item = new QStandardItem(value_list.join(", "));
        } else {
          value_item = new QStandardItem(key_obj.value("value").toString());
        }

        QStandardItem *comment_item = new QStandardItem(key_obj.value("comment").toString()); // 释义
        comment_item->setEditable(false);

        // 存为一行
        QList<QStandardItem *> item_line{key_item, dtype_item, length_item, min_item, max_item, value_item, comment_item};
        machine_item->appendRow(item_line);
      }
    }
  } else {
    qDebug() << "无效的JSON数据";
    return false;
  }

  // 关闭文件
  file.close();
  return true;
}

bool MainWindow::dumpJson(const QString &json_file, QStandardItemModel *model) {
  std::function<QJsonObject(QStandardItem * parent_item)> packJson = [&](QStandardItem *parent_item) -> QJsonObject {
    QJsonObject parent_obj;
    for (int i = 0; i < parent_item->rowCount(); ++i) {
      if (parent_item->hasChildren()) {
        QStandardItem *child_item = parent_item->child(i);
        if (child_item->hasChildren()) {
          parent_obj.insert(child_item->text(), packJson(child_item));
        } else {
          if (getItemLevel(child_item) == 0) {
            parent_obj.insert(child_item->text(), "");
          } else {
            QJsonObject prop_obj;
            prop_obj.insert("dtype", parent_item->child(i, 1)->text());
            prop_obj.insert("length", parent_item->child(i, 2)->text());
            prop_obj.insert("min", parent_item->child(i, 3)->text());
            prop_obj.insert("max", parent_item->child(i, 4)->text());

            if (parent_item->child(i, 2)->text().toInt() > 1) {
              QJsonArray value_array;
              QStringList value_list = parent_item->child(i, 5)->text().split(",");
              for (int k = 0; k < value_list.length(); ++k) {
                value_array.append(value_list.at(k));
              }
              prop_obj.insert("value", value_array);
            } else {
              prop_obj.insert("value", parent_item->child(i, 5)->text());
            }
            prop_obj.insert("comment", parent_item->child(i, 6)->text());
            parent_obj.insert(parent_item->child(i, 0)->text(), prop_obj);
          }
        }
      }
    }
    return parent_obj;
  };

  QJsonObject json_obj = packJson(model->invisibleRootItem());

  QJsonDocument json_doc;
  json_doc.setObject(json_obj);

  // 创建文件
  QFile file(json_file);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
    qDebug() << "文件写入错误";
    return false;
  }

  QTextStream stream(&file);
  stream.setCodec("UTF-8");
  stream << json_doc.toJson();
  file.close();
  return true;
}

int MainWindow::getItemLevel(QStandardItem *item) {
  int level = 0;
  while (item && item->parent()) {
    item = item->parent();
    level++;
  }
  return level;
}

void MainWindow::settingConnection() {
  // 状态label显示
  connect(this, &MainWindow::signUpdateLabelState, [=](QString text) { ui->running_state_label->setText(text); });

  // 检测过量
  connect(this, &MainWindow::signOverLimit, [=]() {
    QMessageBox box;
    box.setWindowFlags(Qt::FramelessWindowHint | Qt::Dialog);
    box.setText("使用次数已达限制，请联系相关人员处理。");
    QImage qimage = QImage(":m_icon/icon/warn.svg").scaled(50, 50, Qt::AspectRatioMode::KeepAspectRatio, Qt::TransformationMode::SmoothTransformation);
    box.setIconPixmap(QPixmap::fromImage(qimage));
    box.addButton("确认", QMessageBox::ActionRole);
    box.exec();
    emit signClose();
  });

  // 关闭事件
  connect(this, &MainWindow::signClose, this, &MainWindow::close);

  // 跳转界面显示上料机监控
  connect(ui->stackedWidget, &QStackedWidget::currentChanged, this, [=]() {
    if (ui->stackedWidget->currentWidget() == ui->monitor_page) {
      showLoadMachineImage();
    }
  });

  connect(rclcomm_, &SytRclComm::visualLoadClothRes, this, &MainWindow::slotVisualLoadCloth); // 上料机可视化
  connect(rclcomm_, &SytRclComm::errorNodeMsgSign, this, &MainWindow::errorNodeMsgSlot);      // 错误提示
  connect(rclcomm_, &SytRclComm::waitUpdateResultSuccess, this, &MainWindow::otaResultShow);  // ota停止
  connect(rclcomm_, &SytRclComm::installRes, this, &MainWindow::otaInstallSuccess);           // ota安装
  connect(rclcomm_, &SytRclComm::signLogPub, this, &MainWindow::slotLogShow);                 // rosout回调消息

  // 产量统计
  connect(rclcomm_, &SytRclComm::signRunCount, [=](uint64_t count) {
    exe_count_ = count;
    ui->progress_bar_3->setProgressBar(exe_count_, 400);
    // checkOk();
  });

  // 补料模式结束
  connect(rclcomm_, &SytRclComm::signLoadMachineAddClothFinish, this, &MainWindow::addClothFinish);

  // 机器空闲
  connect(rclcomm_, &SytRclComm::machineIdle, [=]() {
    this->btnControl({ui->reset_btn, ui->add_cloth_btn, ui->start_btn}, {ui->end_btn, ui->pause_btn});
    this->btnControl({ui->create_style_btn, ui->head_eye_calibration_btn, ui->help_btn, ui->choose_style_btn}, {});
  });

  // 机器运行
  connect(rclcomm_, &SytRclComm::machineRun, [=]() {
    this->btnControl({ui->pause_btn, ui->end_btn}, {ui->start_btn, ui->add_cloth_btn, ui->reset_btn});
    this->btnControl({}, {ui->create_style_btn, ui->head_eye_calibration_btn, ui->help_btn, ui->choose_style_btn});
  });

  // 机器暂停
  connect(rclcomm_, &SytRclComm::machinePause, [=]() {
    this->btnControl({ui->reset_btn, ui->start_btn}, {ui->end_btn, ui->pause_btn, ui->add_cloth_btn});
    this->btnControl({}, {ui->create_style_btn, ui->head_eye_calibration_btn, ui->help_btn, ui->choose_style_btn});
  });

  // 机器停止
  connect(rclcomm_, &SytRclComm::machineStop, [=]() {
    this->btnControl({ui->reset_btn}, {ui->end_btn, ui->pause_btn, ui->add_cloth_btn, ui->start_btn});
    this->btnControl({ui->create_style_btn, ui->head_eye_calibration_btn, ui->help_btn, ui->choose_style_btn}, {});
  });
}

void MainWindow::bindControlConnection() {
  // 切换模式
  connect(developer_widget_, &DeveloperWidget::signChooseMode, [=](int mode) {
    QtConcurrent::run([=]() { rclcomm_->changeMode(mode); });
  });

  // 合片反馈
  qRegisterMetaType<syt_msgs::msg::ComposeMachineState>(
      "syt_msgs::msg::ComposeMachineState");
  connect(rclcomm_, &SytRclComm::updateComposeMachineState, [=](syt_msgs::msg::ComposeMachineState state) {
    developer_widget_->setComposeMachineState(state);
  });

  // 缝纫反馈
  connect(rclcomm_, &SytRclComm::updateSewingMachineState, [=](syt_msgs::msg::SewingMachineState state) {
    developer_widget_->setSewingMachineState(state);
  });

  // 更新固件-上料机
  connect(developer_widget_, &DeveloperWidget::signUpdateLoadMachine, [=]() {
    QtConcurrent::run([=]() { rclcomm_->updateLoadMachine(); });
  });

  // 更新固件-合片机
  connect(developer_widget_, &DeveloperWidget::signUpdateComposeMachine, [=]() {
    QtConcurrent::run([=]() { rclcomm_->updateComposeMachine(); });
  });

  // 更新固件-缝纫机
  connect(developer_widget_, &DeveloperWidget::signUpdateSewingMachine, [=]() {
    QtConcurrent::run([=]() { rclcomm_->updateSewingMachine(); });
  });

  // 上料机-复位
  connect(developer_widget_, &DeveloperWidget::signLoadMachineReset, [=](int id) {
    QtConcurrent::run([=]() { rclcomm_->loadMachineReset(id); });
  });

  // 上料机-补料
  connect(developer_widget_, &DeveloperWidget::signLoadMachineAddCloth, [=](int id) {
    QtConcurrent::run([=]() { rclcomm_->loadMachineAddCloth(id); });
  });

  // 上料机-清台
  connect(developer_widget_, &DeveloperWidget::signLoadMachineClearTable, [=](int id) {
    QtConcurrent::run([=]() { rclcomm_->loadMachineClearTable(id); });
  });

  // 上料机-裁片尺寸
  connect(developer_widget_, &DeveloperWidget::signLoadMachineClothSize, [=](int id, uint32_t width, uint32_t length) {
    QtConcurrent::run([=]() { rclcomm_->loadMachineClothSize(id, width, length); });
  });

  // 上料机-裁片尺寸
  connect(developer_widget_, &DeveloperWidget::signLoadMachineLoadDistance, [=](int id, uint32_t distance) {
    QtConcurrent::run([=]() { rclcomm_->loadMachineLoadDistance(id, distance); });
  });

  // 上料机-上料间隔
  connect(developer_widget_, &DeveloperWidget::signLoadMachineTrayGap, [=](int id, int32_t height) {
    QtConcurrent::run([=]() { rclcomm_->loadMachineTrayGap(id, height); });
  });

  // 上料机-上料偏移
  connect(developer_widget_, &DeveloperWidget::signLoadMachineTrayOffset, [=](int id, int32_t offset) {
    QtConcurrent::run([=]() { rclcomm_->loadMachineTrayOffset(id, offset); });
  });

  // 上料机-粗对位
  connect(developer_widget_, &DeveloperWidget::signLoadMachineRoughAlign, [=](int id) {
    QtConcurrent::run([=]() { rclcomm_->loadMachineRoughAlign(id); });
  });

  // 上料机-上料偏移
  connect(developer_widget_, &DeveloperWidget::signLoadMachineOffset, [=](int id, int offset) {
    QtConcurrent::run([=]() { rclcomm_->loadMachineOffset(id, offset); });
  });

  // 上料机-抓住裁片
  connect(developer_widget_, &DeveloperWidget::signLoadMachineHoldCloth, [=](int id) {
    QtConcurrent::run([=]() { rclcomm_->loadMachineHoldCloth(id); });
  });

  // 上料机-上裁片
  connect(developer_widget_, &DeveloperWidget::signLoadMachineGrabCloth, [=](int id) {
    QtConcurrent::run([=]() { rclcomm_->loadMachineGrabCloth(id); });
  });

  // 上料机-预备设置
  connect(developer_widget_, &DeveloperWidget::signLoadMachinePreSetup, [=](int id) {
    QtConcurrent::run([=]() { rclcomm_->loadMachinePreSetup(id); });
  });

  // 上料机-视觉对位
  connect(developer_widget_, &DeveloperWidget::signLoadMachineVisualAlign, [=](int id) {
    QtConcurrent::run([=]() { rclcomm_->loadMachineVisualAlign(id); });
  });

  // 上料机-设置厚度
  connect(developer_widget_, &DeveloperWidget::signLoadMachineThickness, [=](int id, float thickness) {
    QtConcurrent::run([=]() { rclcomm_->loadMachineThickness(id, thickness); });
  });

  // 上料机-出针
  connect(developer_widget_, &DeveloperWidget::signLoadMachineExtendNeedle, [=](int id, bool enable) {
    QtConcurrent::run([=]() { rclcomm_->loadMachineExtendNeedle(id, enable); });
  });

  // 上料机-重量传感器
  connect(developer_widget_, &DeveloperWidget::signLoadMachineWeightSwitch, [=](int id, bool enable) {
    QtConcurrent::run([=]() { rclcomm_->loadMachineWeightSwitch(id, enable); });
  });

  // 上料机-除褶动作
  connect(developer_widget_, &DeveloperWidget::signLoadMachineUnpleatSwitch, [=](int id, bool enable) {
    QtConcurrent::run([=]() { rclcomm_->loadMachineUnpleatSwitch(id, enable); });
  });

  // 上料机-老化
  connect(developer_widget_, &DeveloperWidget::signLoadMachineAgingSwitch, [=](int id, bool enable) {
    QtConcurrent::run([=]() { rclcomm_->loadMachineAgingSwitch(id, enable); });
  });

  // 合片机-复位
  connect(developer_widget_, &DeveloperWidget::signComposeMachineReset, [=]() {
    QtConcurrent::run([=]() { rclcomm_->composeMachineReset(); });
  });

  // 合片机-停止
  connect(developer_widget_, &DeveloperWidget::signComposeMachineStop, [=]() {
    QtConcurrent::run([=]() { rclcomm_->composeMachineStop(); });
  });

  // 合片机-除褶
  connect(developer_widget_, &DeveloperWidget::signComposeMachineWipeFold, [=]() {
    QtConcurrent::run([=]() { rclcomm_->composeMachineWipeFold(); });
  });

  // 合片机-出针
  connect(developer_widget_, &DeveloperWidget::signComposeMachineExtendNeedle, [=]() {
    QtConcurrent::run([=]() { rclcomm_->composeMachineExtendNeedle(); });
  });

  // 合片机-收针
  connect(developer_widget_, &DeveloperWidget::signComposeMachineWithdrawNeedle, [=]() {
    QtConcurrent::run([=]() { rclcomm_->composeMachineWithdrawNeedle(); });
  });

  // 合片机-吹气
  connect(developer_widget_, &DeveloperWidget::signComposeMachineBlowWind, [=]() {
    QtConcurrent::run([=]() { rclcomm_->composeMachineBlowWind(); });
  });

  // 合片机-停气
  connect(developer_widget_, &DeveloperWidget::signComposeMachineStopBlow, [=]() {
    QtConcurrent::run([=]() { rclcomm_->composeMachineStopBlow(); });
  });

  // 合片机-开吸风台
  connect(developer_widget_, &DeveloperWidget::signComposeMachineFastenSheet, [=]() {
    QtConcurrent::run([=]() { rclcomm_->composeMachineFastenSheet(); });
  });

  // 合片机-关吸风台
  connect(developer_widget_, &DeveloperWidget::signComposeMachineUnfastenSheet, [=]() {
    QtConcurrent::run([=]() { rclcomm_->composeMachineUnfastenSheet(); });
  });

  // 合片机-移动抓手
  connect(developer_widget_, &DeveloperWidget::signComposeMachineMoveHand, [=](float x, float y, float z, float c) {
    QtConcurrent::run([=]() { rclcomm_->composeMachineMoveHand(x, y, z, c); });
  });

  // 合片机-移动吸盘
  connect(developer_widget_, &DeveloperWidget::signComposeMachineMoveSucker, [=](syt_msgs::msg::ComposeMachineSuckerStates sucker_states) {
    QtConcurrent::run([=]() { rclcomm_->composeMachineMoveSucker(sucker_states); });
  });

  // 合片机-平面拟合
  connect(developer_widget_, &DeveloperWidget::signComposeMachineFittingPlane, [=]() {
    QtConcurrent::run([=]() { rclcomm_->composeMachineFittingPlane(); });
  });

  // 合片机-吹气高度
  connect(developer_widget_, &DeveloperWidget::signComposeMachineBlowHeight, [=](float height) {
    QtConcurrent::run([=]() { rclcomm_->composeMachineBlowHeight(height); });
  });

  // 合片机-合片台灯
  connect(developer_widget_, &DeveloperWidget::signComposeMachineTableLight, [=](float ratio) {
    QtConcurrent::run([=]() { rclcomm_->composeMachineTableLight(ratio); });
  });

  // 缝纫机-复位
  connect(developer_widget_, &DeveloperWidget::signSewingMachineReset, [=]() {
    QtConcurrent::run([=]() { rclcomm_->sewingMachineReset(); });
  });

  // 缝纫机-移动抓手
  connect(developer_widget_, &DeveloperWidget::signSewingMachineMoveHand, [=](float x, float y, float c, bool z) {
    QtConcurrent::run([=]() { rclcomm_->sewingMachineMoveHand(x, y, c, z); });
  });

  // 缝纫机-发送关键点
  connect(developer_widget_, &DeveloperWidget::signSewingMachineSendKeypoints, [=](syt_msgs::msg::ClothKeypoints2f keypoints) {
    QtConcurrent::run([=]() { rclcomm_->sewingMachineSendKeypoints(keypoints); });
  });

  // 缝纫机-设置针长
  connect(developer_widget_, &DeveloperWidget::signSewingMachineNeedle, [=](float line_1, float line_2, float line_3, float line_4) {
    QtConcurrent::run([=]() {
      rclcomm_->sewingMachineNeedle(line_1, line_2, line_3, line_4);
    });
  });

  // 缝纫机-水洗标设置
  connect(developer_widget_, &DeveloperWidget::signSewingMachineLabelWidth, [=](bool enable, int side, float width, float position) {
    QtConcurrent::run([=]() {
      rclcomm_->sewingMachineLabelWidth(enable, side, width, position);
    });
  });

  // 缝纫机-水洗标复位
  connect(developer_widget_, &DeveloperWidget::signSewingMachineLabelReset, [=]() {
    QtConcurrent::run([=]() { rclcomm_->sewingMachineLabelReset(); });
  });

  // 缝纫机-运行模式
  connect(developer_widget_, &DeveloperWidget::signSewingMachineSpeed, [=](int speed) {
    QtConcurrent::run([=]() { rclcomm_->sewingMachineSpeed(speed); });
  });

  // 检测标定效果
  connect(developer_widget_, &DeveloperWidget::signCheckCalib, [=]() {
    QtConcurrent::run([=]() { rclcomm_->checkCalibration(); });
  });
  connect(rclcomm_, &SytRclComm::signCheckCalibrationFinish, developer_widget_, &DeveloperWidget::setCheckCalibrationResult);

  // 急停
  connect(developer_widget_, &DeveloperWidget::signEmergencyStop, rclcomm_, [=]() { QtConcurrent::run([=]() { rclcomm_->emergencyStop(); }); });

  // 红灯
  connect(developer_widget_, &DeveloperWidget::signRedLight, rclcomm_, [=]() { QtConcurrent::run([=]() { rclcomm_->redLight(); }); });

  // 绿灯
  connect(developer_widget_, &DeveloperWidget::signGreenLight, rclcomm_, [=]() { QtConcurrent::run([=]() { rclcomm_->greenLight(); }); });

  // 黄灯
  connect(developer_widget_, &DeveloperWidget::signYellowLight, rclcomm_, [=]() { QtConcurrent::run([=]() { rclcomm_->yellowLight(); }); });

  // 蜂鸣器开
  connect(developer_widget_, &DeveloperWidget::signBellOpen, rclcomm_, [=]() { QtConcurrent::run([=]() { rclcomm_->bellOpen(); }); });

  // 蜂鸣器关
  connect(developer_widget_, &DeveloperWidget::signBellClose, rclcomm_, [=]() { QtConcurrent::run([=]() { rclcomm_->bellClose(); }); });

  ////////////// 参数配置界面 /////////////////
}

void MainWindow::bindParamSetConnection() {
  // 切换模式
  connect(param_set_widget_, &ParamSetWidget::signSwitchSewingMode, [=](int mode) {
    QtConcurrent::run([=]() { rclcomm_->sewingMachineMode(mode); });
  });

  // 缝纫机-复位
  connect(param_set_widget_, &ParamSetWidget::signSewingMachineReset, [=]() {
    QtConcurrent::run([=]() { rclcomm_->sewingMachineReset(); });
  });

  // 缝纫机-水洗标复位
  connect(param_set_widget_, &ParamSetWidget::signSewingMachineLabelReset, [=]() {
    QtConcurrent::run([=]() { rclcomm_->sewingMachineLabelReset(); });
  });

  // 缝纫机-水洗标设置
  connect(param_set_widget_, &ParamSetWidget::signSewingMachineLabelWidth, [=](bool enable, int side, float width, float position) {
    QtConcurrent::run([=]() {
      rclcomm_->sewingMachineLabelWidth(enable, side, width, position);
    });
  });

  // 缝纫机-设置针长
  connect(param_set_widget_, &ParamSetWidget::signSewingMachineNeedle, [=](float line_1, float line_2, float line_3, float line_4) {
    QtConcurrent::run([=]() {
      rclcomm_->sewingMachineNeedle(line_1, line_2, line_3, line_4);
    });
  });

  // 缝纫机-设置厚度
  connect(param_set_widget_, &ParamSetWidget::signSewingMachineThickness, [=](float thickness) {
    QtConcurrent::run([=]() { rclcomm_->sewingMachineThickness(thickness); });
  });

  // 合片机-复位
  connect(param_set_widget_, &ParamSetWidget::signComposeMachineReset, [=]() {
    QtConcurrent::run([=]() { rclcomm_->composeMachineReset(); });
  });

  // 合片机-吹气高度
  connect(param_set_widget_, &ParamSetWidget::signComposeMachineBlowHeight, [=](float height) {
    QtConcurrent::run([=]() { rclcomm_->composeMachineBlowHeight(height); });
  });

  // 合片机-合片台灯
  connect(param_set_widget_, &ParamSetWidget::signComposeMachineTableLight, [=](float ratio) {
    QtConcurrent::run([=]() { rclcomm_->composeMachineTableLight(ratio); });
  });

  // 上料机-复位
  connect(param_set_widget_, &ParamSetWidget::signLoadMachineReset, [=](int id) {
    QtConcurrent::run([=]() { rclcomm_->loadMachineReset(id); });
  });

  // 上料机-设置厚度
  connect(param_set_widget_, &ParamSetWidget::signLoadMachineThickness, [=](int id, float thickness) {
    QtConcurrent::run([=]() { rclcomm_->loadMachineThickness(id, thickness); });
  });
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
    QPalette palette = i->palette();
    palette.setColor(QPalette::ButtonText, Qt::black);
    i->setPalette(palette);
  }
  for (auto i : disables) {
    i->setEnabled(false);
    QPalette palette = i->palette();
    palette.setColor(QPalette::ButtonText, Qt::gray);
    i->setPalette(palette);
  }
}

void MainWindow::slotMaxBtnClicked() {
  if (this->isMaximized()) {
    this->showNormal();
    ui->max_btn->setIcon(QIcon(":m_icon/icon/full_screen.svg"));
  } else if (isFullScreen()) {
    this->showNormal();
    ui->max_btn->setIcon(QIcon(":m_icon/icon/full_screen.svg"));
  } else {
    this->showMaximized();
    ui->max_btn->setIcon(QIcon(":m_icon/icon/off_screen.svg"));
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
    showMessageBox(this, WARN, tr("请先设置裁片样式。"), 1, {tr("确认")});
    return;
  }
  is_style_changed_ = false;
  bool res = isFastClick(ui->reset_btn, 1000);
  if (!res) {
    return;
  }

  // 清空可视化
  ui->B_left_visual_label->clear();
  ui->B_right_visual_label->clear();
  ui->A_left_visual_label->clear();
  ui->A_right_visual_label->clear();
  ui->B_left_visual_label->setText("NO IMAGE");
  ui->B_right_visual_label->setText("NO IMAGE");
  ui->A_left_visual_label->setText("NO IMAGE");
  ui->A_right_visual_label->setText("NO IMAGE");

  // 在复位时自动设置软件参数
  QVector<QString> embeded_param_group{"care_label", "sewing_machine", 
                                       "load_machine", "compose_machine"};
  for (int i = 0; i < model_->rowCount(); i++) {
    QStandardItem* item = model_->item(i, 0);
    if (qFind(embeded_param_group, item->text()) == embeded_param_group.end()) {
      for (int j = 0; j < item->rowCount(); j++) {
        ParamLine param_line;
        param_line.name = item->child(j, 0)->text().trimmed();
        param_line.dtype = item->child(j, 1)->text().trimmed();
        param_line.length = item->child(j, 2)->text().trimmed().toInt();
        param_line.min_range = item->child(j, 3)->text().trimmed();
        param_line.max_range = item->child(j, 4)->text().trimmed();
        param_line.value = item->child(j, 5)->text().trimmed();
        DATA_TYPE dtype = str_data_type_map.value(param_line.dtype);
        rclcomm_->updateParam(item->text().toStdString(), dtype, param_line.name, param_line.value, param_line.is_array);
      }
    }
  }

  // 复位指令
  emit signUpdateLabelState(tr("复位中"));
  QtConcurrent::run([=]() {
    rclcomm_->sewingMachineReset(); 
    rclcomm_->resetCmd();
  });
  waiting_spinner_widget_->start();
}

void MainWindow::resetFinish(bool result) {
  waiting_spinner_widget_->stop();
  if (result) {
    emit signUpdateLabelState(tr("复位完成"));
    // this->btnControl({ui->start_btn, ui->end_btn, ui->add_cloth_btn},
    // {ui->reset_btn, ui->pause_btn});
    //  setMutuallyLight(GREEN);
  } else {
    emit signUpdateLabelState(tr("复位失败"));
    // this->btnControl({ui->reset_btn}, {ui->pause_btn, ui->start_btn,
    // ui->end_btn, ui->add_cloth_btn});
    showMessageBox(this, ERROR, tr("复位失败"), 1, {tr("确认")});
  }
}

// 开始按钮槽函数
void MainWindow::startBtnClicked() {
  if (is_style_changed_) {
    showMessageBox(this, WARN, tr("样式已更改，请先进行复位"), 1, {tr("确认")});
    return;
  }

  bool res = isFastClick(ui->start_btn, 1000);
  if (!res) {
    return;
  }

  // 开始指令
  emit signUpdateLabelState(tr("开始运行"));
  QtConcurrent::run([=]() { rclcomm_->startCmd(); });
  waiting_spinner_widget_->start();
}

void MainWindow::startFinish(bool result) {
  waiting_spinner_widget_->stop();
  if (result) {
    emit signUpdateLabelState(tr("运行中"));
  } else {
    emit signUpdateLabelState(tr("运行失败"));
    showMessageBox(this, ERROR, tr("运行失败"), 1, {tr("确认")});
  }
}

void MainWindow::paramProcessFinish(bool result) {
  waiting_spinner_widget_->stop();
  if (!result) {
    QString text = "参数配置失败 错误参数：";
    showMessageBox(this, ERROR, (text + error_param_name), 1, {tr("确认")});
  }
}

void MainWindow::deadErrorProcess(bool result) {
  if(result){
    btnControl({}, {ui->reset_btn, ui->start_btn, ui->pause_btn, ui->end_btn, ui->add_cloth_btn,
               ui->add_cloth_btn, ui->developer_mode_btn, ui->param_set_btn, ui->head_eye_calibration_btn, 
               ui->create_style_btn, ui->lock_screen_btn, ui->help_btn});
    emit signUpdateLabelState("请松开急停按钮");
  }else {
    btnControl({ui->reset_btn,
               ui->add_cloth_btn, ui->developer_mode_btn, ui->param_set_btn, ui->head_eye_calibration_btn, 
               ui->create_style_btn, ui->lock_screen_btn, ui->help_btn}, {});
    emit signUpdateLabelState("请进行复位");
  }
}

// 暂停按钮槽函数
void MainWindow::pauseBtnClicked() {
  bool res = isFastClick(ui->pause_btn, 1000);
  if (!res) {
    return;
  }

  // 开始指令
  emit signUpdateLabelState(tr("开始暂停"));
  QtConcurrent::run([=]() { rclcomm_->pauseCmd(); });
  waiting_spinner_widget_->start();
}

void MainWindow::pauseFinish(bool result) {
  waiting_spinner_widget_->stop();
  if (result) {
    emit signUpdateLabelState(tr("暂停中"));
  } else {
    emit signUpdateLabelState(tr("暂停失败"));
    showMessageBox(this, ERROR, tr("暂停失败"), 1, {tr("确认")});
  }
}

// 停止按钮槽函数
void MainWindow::stopBtnClicked() {
  bool res = isFastClick(ui->end_btn, 1000);
  if (!res) {
    return;
  }

  // 停止指令
  emit signUpdateLabelState(tr("开始结束"));
  QtConcurrent::run([=]() { rclcomm_->stopCmd(); });
  waiting_spinner_widget_->start();
}

void MainWindow::stopFinish(bool result) {
  waiting_spinner_widget_->stop();
  if (result) {
    emit signUpdateLabelState(tr("结束中"));
  } else {
    emit signUpdateLabelState(tr("结束失败"));
    showMessageBox(this, ERROR, tr("结束失败"), 1, {tr("确认")});
  }
}

// 补料按钮槽函数
void MainWindow::addClothBtnClicked() {
  bool res = isFastClick(ui->end_btn, 1000);
  if (!res) {
    return;
  }

  waiting_spinner_widget_->start();

  emit signUpdateLabelState(tr("补料模式"));

  QFuture<void> future_B =
      QtConcurrent::run([=] { rclcomm_->loadMachineAddCloth(0); });

  QThread::msleep(100);

  QFuture<void> future_A =
      QtConcurrent::run([=] { rclcomm_->loadMachineAddCloth(1); });
}

void MainWindow::addClothFinish(bool result, int id) {
  if (id == 0) {
    add_cloth_result_B_ = result;
  } else if (id == 1) {
    add_cloth_result_A_ = result;
  }

  if (++add_cloth_count_ == 2) {
    waiting_spinner_widget_->stop();
    if (add_cloth_result_A_ && add_cloth_result_B_) {
      emit signUpdateLabelState(tr("补料模式设置完成"));
      showMessageBox(this, SUCCESS,
                     tr("补料模式设置成功，请在手动补充裁片后再点击确认。"), 1,
                     {tr("确认")});
      btnControl({ui->reset_btn}, {ui->start_btn, ui->pause_btn, ui->end_btn, ui->add_cloth_btn});
    } else {
      emit signUpdateLabelState(tr("补料模式设置失败"));
      showMessageBox(this, ERROR, tr("补料模式设置失败"), 1, {tr("确认")});
    }
    add_cloth_count_ = 0;
  }
}

// 换板按钮槽函数
void MainWindow::changePlateBtnClicked() {
  bool res = isFastClick(ui->end_btn, 1000);
  if (!res) {
    return;
  }

  emit signUpdateLabelState("换压板模式");

  // TODO
  // 换压板模式
}

// 错误提示槽函数
void MainWindow::errorNodeMsgSlot(QString msg) {
  showMessageBox(this, STATE::ERROR, msg, 1, {tr("确认")});
}

void MainWindow::triggeredOTAUpdate() {
  waiting_spinner_widget_->start();
  future_ = QtConcurrent::run([=] { rclcomm_->otaUpdate(); });
}

void MainWindow::otaResultShow(bool res, QString msg) {
  waiting_spinner_widget_->stop();
  if (res) {
    // todo show ota res
    auto res = showMessageBox(this, STATE::SUCCESS, "检测到远端存在新安装包,请选择是否升级", 2, {"一键升级", "取消升级"});
    if (res == 0) {
      future_ = QtConcurrent::run([=] { rclcomm_->otaDownload(); });
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
      future_ = QtConcurrent::run([=] { rclcomm_->otaInstall(); });
    } else if (res_ == 10) {
      showMessageBox(this, STATE::ERROR, "升级失败,请检查网络是否异常", 1, {"退出"});
      return;
    }

    delete ota_dialog;
  } else {
    showMessageBox(this, STATE::ERROR, "升级出错", 1, {"退出"});
  }
}

void MainWindow::otaInstallSuccess(bool res, QString msg) {
  waiting_spinner_widget_->stop();
  if (!res) {
    showMessageBox(this, ERROR, "获取升级资源失败", 1, {"返回"});
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

  if (machine_id == 0) {
    if (cam_id == 0) {
      pix_B_left_ = QPixmap::fromImage(image);
    } else if (cam_id == 1) {
      pix_B_right_ = QPixmap::fromImage(image);
    } else {
      return;
    }
  } else if (machine_id == 1) {
    if (cam_id == 0) {
      pix_A_left_ = QPixmap::fromImage(image);
    } else if (cam_id == 1) {
      pix_A_right_ = QPixmap::fromImage(image);
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

void MainWindow::slotParamSet() {
  param_set_widget_->move(this->geometry().center() - QPoint(param_set_widget_->width() / 2, param_set_widget_->height() / 2));
  param_set_widget_->show();
}

void MainWindow::slotLockScreen() {
  auto lock_dialog = new LockDialog(this);
  lock_dialog->show();
  lock_dialog->setAttribute(Qt::WA_DeleteOnClose);
}

void MainWindow::slotStartHeadEyeWindow() {
  QString tip = tr(
      "<html>"
      "<head/><b>启动机器人手眼标定</b>\n<body>"
      "<b>注意:</b>"
      "<p>1.标定过程中,<font color=\"red\"><b>禁止靠近机台</b></font>;\n</p>"
      "<p>2.请确认机器人处在一个<font "
      "color=\"red\"><b>良好的位置</b></font>,避免启动时发生碰撞;\n</p>"
      "<p>3.请时刻<font color=\"red\"><b>保持专注</b></font>,并<font "
      "color=\"red\"><b>手持急停开关</b></font>,务必保证危险时刻能按下;\n</p>"
      "<p>4.确保机器人当前为<font color=\"red\"><b>停止状态</b></font>;\n</p>"
      "</body></html>");

  auto res = showMessageBox(this, WARN, tip, 2, {tr("确认"), tr("返回")});
  if (res == 0) {
    emit signHandEyeWindowShow();
  }
}

void MainWindow::slotStartClothStyleWindow() {
  emit signClothStyleWindowShow();
}

void MainWindow::slotDeveloperMode() {
  developer_widget_->move(this->geometry().center() - QPoint(developer_widget_->width() / 2, developer_widget_->height() / 2));
  developer_widget_->show();
}

////////////////////////// 标定槽函数 //////////////////////////
void MainWindow::slotCompCalibStart() {
  future_ = QtConcurrent::run([=] { rclcomm_->compCalib(); });
}

void MainWindow::slotSewingCalibStart() {
  future_ = QtConcurrent::run([=] { rclcomm_->sewingCalib(); });
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
  disconnect(choose_style_dialog, &ChooseStyleDialog::signSetCurrentStyleName, this, &MainWindow::slotSetCurrentStyleName);
  connect(choose_style_dialog, &ChooseStyleDialog::signSetCurrentStyleName, this, &MainWindow::slotSetCurrentStyleName);

  choose_style_dialog->show();
  choose_style_dialog->setAttribute(Qt::WA_DeleteOnClose);
}

void MainWindow::slotSetCurrentStyleFile(QString prefix, QString file_name) {
  style_file_prefix_ = prefix;
  style_file_name_ = file_name;
  future_ = QtConcurrent::run([=] { rclcomm_->setCurrentStyle(prefix, file_name); });
}

void MainWindow::slotGetClothStyle(QString prefix, QString file_name) {
  if (sender()->metaObject()->className() != QString("CreateFromSourceWizard")) {
    waiting_spinner_widget_->start();
  }
  future_ =
      QtConcurrent::run([=] { rclcomm_->getClothStyle(prefix, file_name); });
}

void MainWindow::slotGetClothStyleFinish(bool result, syt_msgs::msg::ClothStyle cloth_style_front, syt_msgs::msg::ClothStyle cloth_style_back) {
  if (qAbs(cloth_style_front.cloth_length - cloth_style_back.cloth_length) < 5){
    if (cloth_style_front.cloth_length >= cloth_style_back.cloth_length){
      float bl_ = cloth_style_front.bottom_length - cloth_style_back.bottom_length;
      if(bl_ > -10){
        is_style_rational_ = true;
      } else {
        is_style_rational_ = false;
      }
    }else{
      float bl_ = cloth_style_back.bottom_length - cloth_style_front.bottom_length;
      if(bl_ > -10){
        is_style_rational_ = true;
      } else {
        is_style_rational_ = false;
      }
    }
  }else {
    is_style_rational_ = false;
  }
  if (result && is_style_rational_) {
    cloth_style_front_ = cloth_style_front;
    cloth_style_back_ = cloth_style_back;

    auto rotatePoints = [=](QPointF point, QPointF center, qreal angle_radius) -> QPointF {
      qreal x_offset = point.x() - center.x();
      qreal y_offset = point.y() - center.y();
      qreal x_rotated = center.x() + x_offset * qCos(angle_radius) - y_offset * qSin(angle_radius);
      qreal y_rotated = center.y() + x_offset * qSin(angle_radius) + y_offset * qCos(angle_radius);
      return QPointF(x_rotated, y_rotated);
    };

    // 旋转端点
    QPointF front_anchor = QPointF(cloth_style_front.keypoint_info.left_bottom.x, cloth_style_front.keypoint_info.left_bottom.y);
    QPointF back_anchor = QPointF(cloth_style_front.keypoint_info.left_bottom.x, cloth_style_front.keypoint_info.left_bottom.y);

    // 旋转角度
    qreal front_angle = -atan2(cloth_style_front.keypoint_info.right_bottom.y - cloth_style_front.keypoint_info.left_bottom.y,
                               cloth_style_front.keypoint_info.right_bottom.x - cloth_style_front.keypoint_info.left_bottom.x) +
                        PI;
    qreal back_angle = -atan2(cloth_style_back.keypoint_info.right_bottom.y - cloth_style_back.keypoint_info.left_bottom.y,
                              cloth_style_back.keypoint_info.right_bottom.x - cloth_style_back.keypoint_info.left_bottom.x) +
                       PI;

    QVector<QPointF> front_points;
    for (int i = 0; i < cloth_style_front.cloth_contour.points.size(); ++i) {
      front_points.push_back(rotatePoints(QPointF(cloth_style_front.cloth_contour.points.at(i).x, cloth_style_front.cloth_contour.points.at(i).y), front_anchor, front_angle));
    }

    QVector<QPointF> back_points;
    for (int i = 0; i < cloth_style_back.cloth_contour.points.size(); ++i) {
      back_points.push_back(rotatePoints(QPointF(cloth_style_back.cloth_contour.points.at(i).x, cloth_style_back.cloth_contour.points.at(i).y), back_anchor, back_angle));
    }

    QMap<QString, QPointF> front_keypoints;
    front_keypoints["left_bottom"] = rotatePoints(QPointF(cloth_style_front.keypoint_info.left_bottom.x, cloth_style_front.keypoint_info.left_bottom.y), front_anchor, front_angle);
    front_keypoints["left_oxter"] = rotatePoints(QPointF(cloth_style_front.keypoint_info.left_oxter.x, cloth_style_front.keypoint_info.left_oxter.y), front_anchor, front_angle);
    front_keypoints["left_shoulder"] = rotatePoints(QPointF(cloth_style_front.keypoint_info.left_shoulder.x, cloth_style_front.keypoint_info.left_shoulder.y), front_anchor, front_angle);
    front_keypoints["left_collar"] = rotatePoints(QPointF(cloth_style_front.keypoint_info.left_collar.x, cloth_style_front.keypoint_info.left_collar.y), front_anchor, front_angle);
    front_keypoints["right_collar"] = rotatePoints(QPointF(cloth_style_front.keypoint_info.right_collar.x, cloth_style_front.keypoint_info.right_collar.y), front_anchor, front_angle);
    front_keypoints["right_shoulder"] = rotatePoints(QPointF(cloth_style_front.keypoint_info.right_shoulder.x, cloth_style_front.keypoint_info.right_shoulder.y), front_anchor, front_angle);
    front_keypoints["right_oxter"] = rotatePoints(QPointF(cloth_style_front.keypoint_info.right_oxter.x, cloth_style_front.keypoint_info.right_oxter.y), front_anchor, front_angle);
    front_keypoints["right_bottom"] = rotatePoints(QPointF(cloth_style_front.keypoint_info.right_bottom.x, cloth_style_front.keypoint_info.right_bottom.y), front_anchor, front_angle);

    QMap<QString, QPointF> back_keypoints;
    back_keypoints["left_bottom"] = rotatePoints(QPointF(cloth_style_back.keypoint_info.left_bottom.x, cloth_style_back.keypoint_info.left_bottom.y), back_anchor, back_angle);
    back_keypoints["left_oxter"] = rotatePoints(QPointF(cloth_style_back.keypoint_info.left_oxter.x, cloth_style_back.keypoint_info.left_oxter.y), back_anchor, back_angle);
    back_keypoints["left_shoulder"] = rotatePoints(QPointF(cloth_style_back.keypoint_info.left_shoulder.x, cloth_style_back.keypoint_info.left_shoulder.y), back_anchor, back_angle);
    back_keypoints["left_collar"] = rotatePoints(QPointF(cloth_style_back.keypoint_info.left_collar.x, cloth_style_back.keypoint_info.left_collar.y), back_anchor, back_angle);
    back_keypoints["right_collar"] = rotatePoints(QPointF(cloth_style_back.keypoint_info.right_collar.x, cloth_style_back.keypoint_info.right_collar.y), back_anchor, back_angle);
    back_keypoints["right_shoulder"] = rotatePoints(QPointF(cloth_style_back.keypoint_info.right_shoulder.x, cloth_style_back.keypoint_info.right_shoulder.y), back_anchor, back_angle);
    back_keypoints["right_oxter"] = rotatePoints(QPointF(cloth_style_back.keypoint_info.right_oxter.x, cloth_style_back.keypoint_info.right_oxter.y), back_anchor, back_angle);
    back_keypoints["right_bottom"] = rotatePoints(QPointF(cloth_style_back.keypoint_info.right_bottom.x, cloth_style_back.keypoint_info.right_bottom.y), back_anchor, back_angle);

    // 预览图
    QImage style_image(2 * (qMax(cloth_style_front.bottom_length, cloth_style_back.bottom_length) + 600), qMax(cloth_style_front.cloth_length, cloth_style_back.cloth_length) + 400, QImage::Format_RGB32);
    style_image.fill(Qt::white);
    qreal width_offset_front = qreal(style_image.width()) / 4 - (front_keypoints["left_bottom"].x() + front_keypoints["right_bottom"].x()) / 2;
    qreal height_offset_front = -front_keypoints["left_bottom"].y() + style_image.height() - 100;
    qreal width_offset_back = qreal(style_image.width()) / 2 + qreal(style_image.width()) / 4 - (back_keypoints["left_bottom"].x() + back_keypoints["right_bottom"].x()) / 2;
    qreal height_offset_back = -back_keypoints["left_bottom"].y() + style_image.height() - 100;

    // 画笔对象
    QPainter painter(&style_image);
    painter.setRenderHint(QPainter::Antialiasing);

    // 裁片颜色以填充图像
    QColor front_color, back_color;
    int front_color_value = cloth_style_front.cloth_color;
    int back_color_value = cloth_style_back.cloth_color;

    front_color = QColor((front_color_value & 0xff0000) >> 16, (front_color_value & 0xff00) >> 8, (front_color_value & 0xff));
    back_color = QColor((back_color_value & 0xff0000) >> 16, (back_color_value & 0xff00) >> 8, (back_color_value & 0xff));

    // 提取轮廓线
    QVector<QPointF> front_contour, back_contour;
    for (int i = 0; i < front_points.size(); ++i) {
      front_contour.push_back(front_points.at(i) + QPointF(width_offset_front, height_offset_front));
    }

    for (int i = 0; i < back_points.size(); ++i) {
      back_contour.push_back(back_points.at(i) + QPointF(width_offset_back, height_offset_back));
    }

    // 绘制轮廓线
    painter.setPen(QPen(Qt::black, 3));
    painter.drawPolyline(front_contour.data(), front_contour.size());
    painter.drawPolyline(back_contour.data(), back_contour.size());

    // 设置绘制路径
    QPainterPath front_path, back_path;
    front_path.addPolygon(QPolygonF(front_contour));
    back_path.addPolygon(QPolygonF(back_contour));

    // 绘制填充轮廓
    painter.setBrush(front_color);
    painter.drawPath(front_path);
    painter.setBrush(back_color);
    painter.drawPath(back_path);

    // 绘制放置区域
    QFont location_font;
    location_font.setPointSize(50);
    location_font.setBold(true);
    painter.setFont(location_font);
    painter.drawText(style_image.width() / 4 - 250, 100, 500, 70, Qt::AlignCenter, QString("放置%1区").arg(cloth_style_front.cloth_location - 50 ? "B" : "A"));
    painter.drawText(style_image.width() / 4 + style_image.width() / 2 - 250, 100, 500, 70, Qt::AlignCenter, QString("放置%1区").arg(cloth_style_back.cloth_location - 50 ? "B" : "A"));

    // 绘制关键点
    auto draw_keypoints = [&](QMap<QString, QPointF> keypoints, int w_offset, int h_offset, QPointF center, qreal angle_radius) {
      qreal radius = 8;
      painter.drawEllipse(QPointF(w_offset, h_offset) + keypoints["left_bottom"], radius, radius);
      painter.drawEllipse(QPointF(w_offset, h_offset) + keypoints["left_oxter"], radius, radius);
      painter.drawEllipse(QPointF(w_offset, h_offset) + keypoints["left_shoulder"], radius, radius);
      painter.drawEllipse(QPointF(w_offset, h_offset) + keypoints["left_collar"], radius, radius);
      painter.drawEllipse(QPointF(w_offset, h_offset) + keypoints["right_collar"], radius, radius);
      painter.drawEllipse(QPointF(w_offset, h_offset) + keypoints["right_shoulder"], radius, radius);
      painter.drawEllipse(QPointF(w_offset, h_offset) + keypoints["right_oxter"], radius, radius);
      painter.drawEllipse(QPointF(w_offset, h_offset) + keypoints["right_bottom"], radius, radius);
    };

    // 黑色点画关键点
    painter.setPen(QPen(Qt::black, 3));
    painter.setBrush(Qt::white);
    draw_keypoints(front_keypoints, width_offset_front, height_offset_front, front_anchor, front_angle);
    draw_keypoints(back_keypoints, width_offset_back, height_offset_back, back_anchor, back_angle);

    // 绘制刻度
    auto drawLength = [&](QPointF begin, QPointF end, float length, bool invert_text = false) {
      QLineF line(begin, end);
      QVector2D direction_vec = QVector2D(end.x() - begin.x(), end.y() - begin.y()).normalized();
      QVector2D norm_vec(direction_vec.y(), -direction_vec.x());
      int edge_length = 40;
      QPointF edge_begin_1 = QPointF(begin.x() + norm_vec.x() * 15, begin.y() + norm_vec.y() * 15);
      QPointF edge_begin_2 = QPointF(edge_begin_1.x() + norm_vec.x() * edge_length, edge_begin_1.y() + norm_vec.y() * edge_length);
      QPointF edge_end_1 = QPointF(end.x() + norm_vec.x() * 15, end.y() + norm_vec.y() * 15);
      QPointF edge_end_2 = QPointF(edge_end_1.x() + norm_vec.x() * edge_length, edge_end_1.y() + norm_vec.y() * edge_length);
      QPointF edge_begin_center = (edge_begin_1 + edge_begin_2) / 2;
      QPointF edge_end_center = (edge_end_1 + edge_end_2) / 2;
      QPointF edge_center = (edge_begin_center + edge_end_center) / 2;

      // 保存当前画笔状态
      painter.save();

      // 背景色取反
      painter.setCompositionMode(QPainter::RasterOp_SourceAndNotDestination);

      // 绘制刻度线
      painter.setPen(QPen(Qt::white, 3));
      painter.drawLine(edge_begin_1, edge_begin_2);
      painter.drawLine(edge_end_1, edge_end_2);
      painter.drawLine(edge_begin_center, edge_end_center);

      // 绘制文本
      QFont font;
      font.setPointSize(20);
      font.setBold(true);
      painter.setFont(font);

      QString text = QString("%1").arg(length);
      qreal text_width = painter.fontMetrics().width(text);
      qreal text_height = painter.fontMetrics().height();
      QPointF text_center = QPointF(edge_center.x() + norm_vec.x() * 30, edge_center.y() + norm_vec.y() * 30);
      qreal angle = atan2(direction_vec.y(), direction_vec.x());
      if (invert_text) {
        angle += PI;
      }

      painter.translate(text_center);
      painter.rotate(qRadiansToDegrees(angle));
      painter.drawText(-text_width / 2, text_height / 2, text);
      painter.resetMatrix();
      painter.restore();
    };

    // 前片
    drawLength(QPointF(width_offset_front, height_offset_front) + front_keypoints["left_bottom"], QPointF(width_offset_front, height_offset_front) + front_keypoints["right_bottom"], cloth_style_front.bottom_length, true);
    drawLength(QPointF(width_offset_front, height_offset_front) + front_keypoints["right_bottom"], QPointF(width_offset_front, height_offset_front) + front_keypoints["right_oxter"], cloth_style_front.side_length);
    drawLength(QPointF(width_offset_front, height_offset_front) + front_keypoints["right_shoulder"], QPointF(width_offset_front, height_offset_front) + front_keypoints["right_collar"], cloth_style_front.shoulder_length);
    drawLength(QPointF(width_offset_front, height_offset_front) + front_keypoints["right_oxter"], QPointF(width_offset_front, height_offset_front) + front_keypoints["left_oxter"], cloth_style_front.oxter_length);
    drawLength(QPointF(width_offset_front, height_offset_front) + QPointF(front_keypoints["left_bottom"].x(), front_keypoints["left_collar"].y()), QPointF(width_offset_front, height_offset_front) + front_keypoints["left_bottom"], cloth_style_front.cloth_length, true);

    // 后片
    drawLength(QPointF(width_offset_back, height_offset_back) + back_keypoints["left_bottom"], QPointF(width_offset_back, height_offset_back) + back_keypoints["right_bottom"], cloth_style_back.bottom_length, true);
    drawLength(QPointF(width_offset_back, height_offset_back) + back_keypoints["right_bottom"], QPointF(width_offset_back, height_offset_back) + back_keypoints["right_oxter"], cloth_style_back.side_length);
    drawLength(QPointF(width_offset_back, height_offset_back) + back_keypoints["right_shoulder"], QPointF(width_offset_back, height_offset_back) + back_keypoints["right_collar"], cloth_style_back.shoulder_length);
    drawLength(QPointF(width_offset_back, height_offset_back) + back_keypoints["right_oxter"], QPointF(width_offset_back, height_offset_back) + back_keypoints["left_oxter"], cloth_style_back.oxter_length);
    drawLength(QPointF(width_offset_back, height_offset_back) + QPointF(back_keypoints["left_bottom"].x(), back_keypoints["left_collar"].y()), QPointF(width_offset_back, height_offset_back) + back_keypoints["left_bottom"], cloth_style_back.cloth_length, true);

    style_scene_->clear();
    image_item_ = new ImageItem(style_image);
    style_scene_->addItem(image_item_);

    // 设置操作时用的缩放系数
    qreal gv_width = ui->style_graphics_view->width();
    qreal gv_height = ui->style_graphics_view->height();
    image_item_->setQGraphicsViewWH(gv_width, gv_height);

    connect(style_scene_, &QGraphicsScene::changed, [=](const QList<QRectF> &region) {
      if (image_item_->pos().x() < style_scene_->sceneRect().left()) {
        image_item_->setPos(style_scene_->sceneRect().left(), image_item_->pos().y());
      }
      if (image_item_->pos().x() > style_scene_->sceneRect().right()) {
        image_item_->setPos(style_scene_->sceneRect().right(), image_item_->pos().y());
      }
      if (image_item_->pos().y() < style_scene_->sceneRect().top()) {
        image_item_->setPos(image_item_->pos().x(), style_scene_->sceneRect().top());
      }
      if (image_item_->pos().y() > style_scene_->sceneRect().bottom()) {
        image_item_->setPos(image_item_->pos().x(), style_scene_->sceneRect().bottom());
      }
    });

    // 设置可视框
    style_scene_->setSceneRect(-gv_width / 2, -gv_height / 2, gv_width, gv_height);

    // 清空原有信息
    ui->cloth_style_tree_widget->clear();

    // 设置信息到treewidget中
    QTreeWidgetItem *front_item = new QTreeWidgetItem(QStringList() << tr("前片"));
    QTreeWidgetItem *back_item = new QTreeWidgetItem(QStringList() << tr("后片"));

    ui->cloth_style_tree_widget->addTopLevelItem(front_item);
    ui->cloth_style_tree_widget->addTopLevelItem(back_item);

    auto fillTreeWidget = [&](QTreeWidgetItem *top_item, syt_msgs::msg::ClothStyle cloth_style) {
      top_item->addChild(new QTreeWidgetItem(QStringList() << tr("衣长") << QString::number(cloth_style.cloth_length)));
      top_item->addChild(new QTreeWidgetItem(QStringList() << tr("下摆长") << QString::number(cloth_style.bottom_length)));
      top_item->addChild(new QTreeWidgetItem(QStringList() << tr("腋下间距") << QString::number(cloth_style.oxter_length)));
      top_item->addChild(new QTreeWidgetItem(QStringList() << tr("肩缝长") << QString::number(cloth_style.shoulder_length)));
      top_item->addChild(new QTreeWidgetItem(QStringList() << tr("侧缝长") << QString::number(cloth_style.side_length)));
      top_item->addChild(new QTreeWidgetItem(QStringList() << tr("有无印花") << (cloth_style.have_printings ? QString(tr("有")) : QString(tr("无")))));

      // 设置颜色预览
      QTreeWidgetItem *color_item = new QTreeWidgetItem(QStringList() << tr("颜色"));
      top_item->addChild(color_item);
      ui->cloth_style_tree_widget->setItemWidget(color_item, 1, new ShowColorWidget(QString::number(cloth_style.cloth_color, 16)));

      top_item->addChild(new QTreeWidgetItem(QStringList() << tr("裁片克数") << QString::number(cloth_style.cloth_weight)));
      top_item->addChild(new QTreeWidgetItem(QStringList() << tr("弹性") << id_style_map.value(cloth_style.elasticity_level)));
      top_item->addChild(new QTreeWidgetItem(QStringList() << tr("厚度") << id_style_map.value(cloth_style.thickness_level)));
      top_item->addChild(new QTreeWidgetItem(QStringList() << tr("尺码") << id_style_map.value(cloth_style.cloth_size)));
      top_item->addChild(new QTreeWidgetItem(QStringList() << tr("光泽度") << id_style_map.value(cloth_style.glossiness_level)));
    };
    fillTreeWidget(front_item, cloth_style_front_);
    fillTreeWidget(back_item, cloth_style_back_);
    ui->cloth_style_tree_widget->expandAll(); // 展开
    waiting_spinner_widget_->stop();

    is_style_seted_ = true;
    is_style_changed_ = true;
    ui->choose_style_line_edit->setText(style_file_name_);
    last_style_file_name_ = style_file_name_;
    emit signUpdateLabelState(tr("已设置样式，请进行复位"));
    btnControl({ui->reset_btn}, {{ui->start_btn, ui->pause_btn, ui->end_btn, ui->add_cloth_btn}});

    showMessageBox(this, SUCCESS, tr("样式设置成功"), 1, {tr("确认")});
  }
  else if (result == true && is_style_rational_ == false){
    waiting_spinner_widget_->stop();
    QString front_cl = "前片衣长：" + QString::number(cloth_style_front.cloth_length);
    QString front_bl = "前下摆长：" + QString::number(cloth_style_front.bottom_length);
    QString back_cl = "后片衣长：" + QString::number(cloth_style_back.cloth_length);
    QString back_bl = "后下摆长：" + QString::number(cloth_style_back.bottom_length);
    QString text = "样式有误请检查数据\n";
    showMessageBox(this, WARN, (text + "\n" + front_cl + "\n" + front_bl + "\n" + back_cl + "\n" + back_bl), 1, {tr("确认")});
    ui->choose_style_line_edit->setText(last_style_file_name_);
  } 
  else {
    showMessageBox(this, WARN, tr("获取样式信息失败"), 1, {tr("确认")});
  }
}

void MainWindow::slotSetCurrentStyleName(QString file_name) {}

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

  connect(create_from_source_wizard, &CreateFromSourceWizard::signGetClothStyle, this, &MainWindow::slotGetClothStyle);
  disconnect(rclcomm_, &SytRclComm::signGetClothStyleFinish, this, &MainWindow::slotGetClothStyleFinish);
  connect(rclcomm_, &SytRclComm::signGetClothStyleFinish, create_from_source_wizard, &CreateFromSourceWizard::slotGetClothStyleResult);
  connect(create_from_source_wizard, &CreateFromSourceWizard::signCreateStyle, this, &MainWindow::slotCreateStyle);
  connect(rclcomm_, &SytRclComm::signCreateStyleFinish, create_from_source_wizard, &CreateFromSourceWizard::slotCreateStyleResult);
  connect(create_from_source_wizard, &CreateFromSourceWizard::signRenameClothStyle, this, &MainWindow::slotRenameClothStyle);
  connect(rclcomm_, &SytRclComm::signRenameClothStyleFinish, create_from_source_wizard, &CreateFromSourceWizard::slotRenameClothStyleResult);

  create_from_source_wizard->show();
  create_from_source_wizard->setAttribute(Qt::WA_DeleteOnClose);
}

void MainWindow::slotMoveHand() {
  future_ = QtConcurrent::run([=] { rclcomm_->composeMachineMoveHand(0, 900, 0, 0); });
}

void MainWindow::slotDetectClothByAutoCreateStyle(int cloth_type) {
  future_ = QtConcurrent::run([=] { rclcomm_->getClothInfo(1, cloth_type); });
}

void MainWindow::slotCreateStyle(int mode, QString prefix, syt_msgs::msg::ClothStyle cloth_style_front, syt_msgs::msg::ClothStyle cloth_style_back) {
  future_ = QtConcurrent::run([=] {
    rclcomm_->createStyle(mode, prefix, cloth_style_front, cloth_style_back);
  });
}

void MainWindow::slotRenameClothStyle(QString old_name, QString new_name) {
  future_ = QtConcurrent::run([=] {
    rclcomm_->renameClothStyle(old_name, new_name);
  });
}

void MainWindow::monitorErrorCode(){
  uint32_t error_code = rclcomm_->returndeadcode();
  uint32_t dead_code = '0x00300001';
  if(error_code == dead_code){
    emit signScramCondition(true);
  }else{
    emit signScramCondition(false);
  }
}
