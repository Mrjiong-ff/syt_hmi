#pragma once
#include <QMainWindow>
#include <QMenu>
#include <QStandardItemModel>
#include <QtConcurrent/QtConcurrent>
#include <memory>
#include <queue>
#include <deque>

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
#include "syt_hmi/developer_widget.h"
#include "syt_hmi/hand_eye_dialog.h"
#include "syt_hmi/image_item.h"
#include "syt_hmi/lock_dialog.h"
#include "syt_hmi/manual_input_param_wizard.h"
#include "syt_hmi/ota_update_dialog.h"
#include "syt_hmi/param_set_widget.h"
#include "syt_hmi/show_color_widget.h"
#include "syt_hmi/translate_dialog.h"
#include "ui_main_window.h"
#include "utils/utils.h"

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
  UP = 0,
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
  RED = 0,
  YELLOW = 1,
  GREEN,
  GRAY
};

enum LOG_LEVEL {
  LOG_DEBUG = 10,
  LOG_INFO = 20,
  LOG_WARN = 30,
  LOG_ERROR = 40,
  LOG_FATAL = 50,
};

class MainWindow : public QMainWindow {
  Q_OBJECT

  Q_ENUM(Direction)
  Q_ENUM(LIGHT_COLOR)
  Q_ENUM(LOG_LEVEL)

public:
  explicit MainWindow(QWidget *parent = nullptr);
  ~MainWindow() override;
  void region(const QPoint &currentGlobalPoint); // 用于定位鼠标移动的位置,改变光标

protected:
  bool event(QEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  bool eventFilter(QObject *obj, QEvent *ev) override;
  void resizeEvent(QResizeEvent *event) override;
  virtual void keyPressEvent(QKeyEvent *event) override;

private:
  void checkOk();
  void initWidget();
  void settingConnection();
  void bindControlConnection();
  void bindParamSetConnection();
  void deleteAll();
  void initOther();
  // void setMutuallyLight(LIGHT_COLOR color);
  void btnControl(std::vector<QPushButton *> enables, std::vector<QPushButton *> unables);

  // 设置各个组件的属性和信号槽
  void setStatisticComponent();
  void setPreviewComponent();
  void setLogComponent();
  void setTimeComponent();
  void setToolBar();
  void setMainControlButton();
  void setVisualComponent();
  void setBaseComponet();
  void setChooseStyleComponet();
  void setParamManageComponet();
  void setDeveloperWidget();
  void setParamSetWidget();

  // 辅助函数
  void showLoadMachineImage();
  bool readJson(const QString &json_file, QStandardItemModel *model);
  bool dumpJson(const QString &json_file, QStandardItemModel *model);
  int getItemLevel(QStandardItem *item);
  void monitorErrorCode();

  // 参数配置相关函数
  bool param_processing_;
  bool stop_param_process_;

signals:
  void signClose();
  void signOverLimit();
  void signHandEyeWindowShow();
  void signClothStyleWindowShow();
  void signUpdateLabelState(QString);
  void signGetClothStyle(QString prefix, QString file_name);
  void signParamProcessFinish(bool);
  void signScramCondition(bool);

private slots:
  void slotMaxBtnClicked();
  void slotPrevPage();
  void slotNextPage();
  void resetBtnClicked();
  void resetFinish(bool);
  void startBtnClicked();
  void startFinish(bool);
  void pauseBtnClicked();
  void pauseFinish(bool);
  void stopBtnClicked();
  void stopFinish(bool);
  void paramProcessFinish(bool);
  void deadErrorProcess(bool);
  void addClothBtnClicked();
  void addClothFinish(bool result, int id);
  void changePlateBtnClicked();
  // void changePlateFinish();
  void errorNodeMsgSlot(QString msg);
  void triggeredOTAUpdate();
  void otaResultShow(bool res, QString msg);
  void otaInstallSuccess(bool res, QString msg);
  void slotVisualLoadCloth(int machine_id, int cam_id, QImage image);

  ////////////////////////// 工具栏函数 //////////////////////////
  void slotShowDevLoginWindow();
  void slotParamSet();
  void slotStartHeadEyeWindow();
  void slotStartClothStyleWindow();
  void slotLockScreen();

  void slotDeveloperMode();

  ////////////////////////// 标定槽函数 //////////////////////////
  void slotCompCalibStart();
  void slotSewingCalibStart();

  ////////////////////////// 显示log槽函数 //////////////////////////
  void slotLogShow(QString, int, QString, QString, QString);

  ////////////////////////// 选择设置样式槽函数 //////////////////////////
  void slotChooseStyleFile();
  void slotSetCurrentStyleFile(QString prefix, QString file_name);
  void slotGetClothStyle(QString prefix, QString file_name);
  void slotGetClothStyleFinish(bool result, syt_msgs::msg::ClothStyle cloth_style_front, syt_msgs::msg::ClothStyle cloth_style_back);
  void slotSetCurrentStyleName(QString file_name);

  ////////////////////////// 创建衣服样式槽函数 //////////////////////////
  void slotCreateFromCAD(ClothStyleDialog *parent);
  void slotAutoCreateStyle(ClothStyleDialog *parent);
  void slotManualInputParam(ClothStyleDialog *parent);
  void slotCreateFromSource(ClothStyleDialog *parent);

  void slotMoveHand();
  void slotDetectClothByAutoCreateStyle(int cloth_type);
  void slotCreateStyle(int mode, QString prefix, syt_msgs::msg::ClothStyle cloth_style_front, syt_msgs::msg::ClothStyle cloth_style_back);
  void slotRenameClothStyle(QString old_name, QString new_name);

private:
  Ui::MainWindow *ui;

  SytRclComm *rclcomm_ = nullptr;

  ImageItem *image_item_;
  QGraphicsScene *style_scene_; // 预览场景

  QFuture<void> future_;

  // 定义的一些bool类型标志位
  bool is_style_seted_ = false;
  bool is_style_changed_ = false;
  bool is_style_rational_ = false;

  // 运行次数
  int exe_count_;
  int max_count_;
  int cur_count_;
  
  // 急停错误动作
  bool error_flags = false;
  int time_count_ = 0;
  std::vector<uint32_t> codeQueue;
  bool is_dead_error_ = true;
  const int maxSize = 10;

  // 日志等级过滤动作
  LOG_LEVEL log_level_ = LOG_WARN;

  bool is_mouse_left_press_down_ = false;
  bool is_load_cloth_on_ = true;
  bool is_comp_cloth_on_ = true;

  // 上料用标志位
  int add_cloth_count_ = 0;
  bool add_cloth_result_A_ = 0;
  bool add_cloth_result_B_ = 0;

  QPixmap pix_B_left_;
  QPixmap pix_B_right_;
  QPixmap pix_A_left_;
  QPixmap pix_A_right_;

  // 样式文件变量
  QString style_file_prefix_;
  QString style_file_name_;
  QString last_style_file_name_;
  syt_msgs::msg::ClothStyle cloth_style_front_;
  syt_msgs::msg::ClothStyle cloth_style_back_;

  // 显示时间:
  QTimer *time_timer_;
  QTimer *error_timer_;

  QMenu *title_menu_;

  DeveloperWidget *developer_widget_; // 开发者界面
  ParamSetWidget *param_set_widget_;  // 配置界面

  WaitingSpinnerWidget *waiting_spinner_widget_;

  QPoint m_mousePos_;
  Direction dir_; // 窗口大小改变时，记录改变方向

  // 参数预览配置
  QStandardItemModel *model_;
  QString current_param_file_;

  // 参数错误名称
  QString error_param_name;
};
