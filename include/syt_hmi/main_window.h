#pragma once
#include <QDesktopWidget>
#include <QGraphicsOpacityEffect>
#include <QMainWindow>
#include <QMenu>
#include <QScreen>
#include <QScrollBar>
#include <QtConcurrent/QtConcurrent>
#include <iostream>

#include "syt_btn/winclosebutton.h"
#include "syt_btn/winmaxbutton.h"
#include "syt_btn/winmenubutton.h"
#include "syt_btn/winminbutton.h"
#include "syt_hmi/auto_create_wizard.h"
#include "syt_hmi/choose_style_dialog.h"
#include "syt_hmi/cloth_style_dialog.h"
#include "syt_hmi/create_from_cad_wizard.h"
#include "syt_hmi/create_from_source_wizard.h"
#include "syt_hmi/dev_login_window.h"
#include "syt_hmi/dev_select_dialog.h"
#include "syt_hmi/dev_window.h"
#include "syt_hmi/head_eye_dialog.h"
#include "syt_hmi/lock_dialog.h"
#include "syt_hmi/manual_input_param_wizard.h"
#include "syt_hmi/ota_update_dialog.h"
#include "syt_hmi/show_color_widget.h"
#include "ui_main_window.h"
#include "utils/utils.h"
#include "utils/waitingspinnerwidget.h"

#include "syt_rclcomm/rcl_comm.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

#define PADDING 2
#define MAX_RECORDS 30

// 定义方向枚举，用于判断鼠标在mainWindow的哪个位置
enum Direction {
  UP   = 0,
  DOWN = 1,
  LEFT,
  RIGHT,
  LEFTTOP,
  LEFTBOTTOM,
  RIGHTBOTTOM,
  RIGHTTOP,
  NONE
};

enum LIGHT_COLOR {
  RED    = 0,
  YELLOW = 1,
  GREEN,
  GRAY
};

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = nullptr);
  ~MainWindow() override;
  void region(const QPoint &currentGlobalPoint); // 用于定位鼠标移动的位置,改变光标

protected:
  void mousePressEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  bool eventFilter(QObject *obj, QEvent *ev) override;
  void resizeEvent(QResizeEvent *event) override;
  virtual void keyPressEvent(QKeyEvent *event) override;

private:
  void initNode();
  void initWidget();
  void settingConnection();
  void setMutuallyLight(LIGHT_COLOR);
  void deleteAll();
  void initOther();
  void btnControl(std::vector<QPushButton *> enables, std::vector<QPushButton *> unables);

private slots:
  void slotMaxBtnClicked();
  void slotPrevPage();
  void slotNextPage();
  void resetBtnClicked();
  void startBtnClicked();
  void stopBtnClicked();
  void errorNodeMsgSlot(QString msg);
  void triggeredOTAUpdate();
  void otaResultShow(bool res, QString msg);
  void otaInstallSuccess(bool res, QString msg);
  void slotVisualLoadCloth(int machine_id, int cam_id, QImage image);

  ////////////////////////// 工具栏函数 //////////////////////////
  void slotShowDevLoginWindow();
  void slotLockScreen();
  void slotStartHeadEyeWindow();
  void slotStartClothStyleWindow();

  void slotDevWindow();

  ////////////////////////// 标定槽函数 //////////////////////////
  void slotCompCalibRes(bool f);
  void slotSewingCalibRes(bool f);
  void slotCompCalibStart();
  void slotSewingCalibStart();

  ////////////////////////// 显示log槽函数 //////////////////////////
  void slotLogShow(QString, QString, QString, QString, QString);

  ////////////////////////// 选择设置样式槽函数 //////////////////////////
  void slotChoseStyleFile(QString prefix, QString file_name);

  ////////////////////////// 创建衣服样式槽函数 //////////////////////////
  void slotCreateFromCAD(ClothStyleDialog *parent);
  void slotAutoCreateStyle(ClothStyleDialog *parent);
  void slotManualInputParam(ClothStyleDialog *parent);
  void slotCreateFromSource(ClothStyleDialog *parent);

  void slotMoveHandByAutoCreateStyle();
  void slotDetectClothByAutoCreateStyle(int cloth_type);
  void slotCreateStyleByAutoCreateStyle(syt_msgs::msg::ClothStyle cloth_style_front, syt_msgs::msg::ClothStyle cloth_style_back);
  void slotRenameClothStyleByAutoCreateStyle(std::string old_name, std::string new_name);

signals:
  void signHeadEyeWindowShow();
  void signClothStyleWindowShow();
  void processSuccessful();
  void signUpdateLabelState(QString);
  void signGetClothStyle(QString prefix, QString file_name);

private:
  Ui::MainWindow *ui;

  SytRclComm *rclcomm_ = nullptr;

  QFuture<void> future_;

  // 定义的一些bool类型标志位
  bool is_mouse_left_press_down_ = false;
  bool is_load_cloth_on_         = true;
  bool is_comp_cloth_on_         = true;

  // 样式文件变量
  QString style_file_prefix_;
  QString style_file_name_;
  syt_msgs::msg::ClothStyle cloth_style_front_;
  syt_msgs::msg::ClothStyle cloth_style_back_;

  // todo 进度条测试
  int value = 0;
  QTimer *test_timer;

  // 一些自定义的按钮控件
  WinMenuButton *m_menuBtn_;
  WinMinButton *m_hideBtn_;
  WinMaxButton *m_maxBtn_;
  WinCloseButton *m_closeBtn_;

  DevLoginWindow *dev_login_window_;
  InteractiveButtonBase *prev_btn;
  InteractiveButtonBase *next_btn_;

  // about action
  QMenu *m_titleMenu_;
  QMenu *m_menu_;
  QAction *minAct_;
  QAction *maxAct_;
  QAction *closeAct_;
  QAction *fullAct_;

  QAction *updateAct_;
  QAction *helpAct_;
  QAction *aboutAct_;

  WaitingSpinnerWidget *waiting_spinner_widget_;

  QPoint m_mousePos_;
  Direction dir_; // 窗口大小改变时，记录改变方向
};
