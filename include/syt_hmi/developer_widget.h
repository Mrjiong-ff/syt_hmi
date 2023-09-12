#pragma once

#include "syt_msgs/msg/cloth_keypoints2f.hpp"
#include "syt_msgs/msg/compose_machine_state.hpp"
#include "syt_msgs/msg/compose_machine_sucker_states.hpp"
#include "syt_msgs/msg/fsm_run_mode.hpp"
#include "syt_msgs/msg/sewing_machine_state.hpp"
#include "utils/utils.h"
#include "utils/waitingspinnerwidget.h"

#include <QFileDialog>
#include <QWidget>
#include <QtConcurrent/QtConcurrent>

namespace Ui {
class DeveloperWidget;
}

class DeveloperWidget : public QWidget {
  Q_OBJECT

public:
  explicit DeveloperWidget(QWidget *parent = nullptr);
  ~DeveloperWidget();

protected:
  void mousePressEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;

private:
  Ui::DeveloperWidget *ui;

  WaitingSpinnerWidget *waiting_spinner_widget_;

  // 移动窗口
  bool is_mouse_left_press_down_;
  QPoint mouse_pos_;

  // 更新固件
  QString update_bin_path_;

  float left_bottom_init_x_ = 0;
  float left_bottom_init_y_ = 0;
  float left_oxter_init_x_ = 0;
  float left_oxter_init_y_ = 0;
  float left_shoulder_init_x_ = 0;
  float left_shoulder_init_y_ = 0;
  float left_shoulder_init_c_ = 0;
  float right_shoulder_init_x_ = 0;
  float right_shoulder_init_y_ = 0;
  float right_shoulder_init_c_ = 0;
  float right_oxter_init_x_ = 0;
  float right_oxter_init_y_ = 0;
  float right_bottom_init_x_ = 0;
  float right_bottom_init_y_ = 0;

private:
  void setParam();
  void setButtonFrame();
  void switchPage();
  void bindLoadMachine();
  void bindComposeMachine();
  void bindSewingMachine();
  void bindOther();
  void setChooseMode();
  void setUpdateBin();
  void setUseSewing();
  void setPressureTest();
  void setCurveSewing();
  void setCheckCalibration();

signals:
  // 上料机
  void signLoadMachineReset(int id);
  void signLoadMachineAddCloth(int id);
  void signLoadMachineClearTable(int id);
  void signLoadMachineClothSize(int id, uint32_t width, uint32_t height);
  void signLoadMachineLoadDistance(int id, uint32_t distance);
  void signLoadMachineTrayGap(int id, uint32_t height);
  void signLoadMachineRoughAlign(int id);
  void signLoadMachineOffset(int id, int offset);
  void signLoadMachineHoldCloth(int id);
  void signLoadMachineGrabCloth(int id);
  void signLoadMachinePreSetup(int id);
  void signLoadMachineVisualAlign(int id);

  // 合片机
  void signComposeMachineReset();
  void signComposeMachineStop();
  void signComposeMachineWipeFold();
  void signComposeMachineExtendNeedle();
  void signComposeMachineWithdrawNeedle();
  void signComposeMachineBlowWind();
  void signComposeMachineStopBlow();
  void signComposeMachineFastenSheet();
  void signComposeMachineUnfastenSheet();
  void signComposeMachineMoveHand(float x, float y, float z, float c);
  void signComposeMachineMoveSucker(syt_msgs::msg::ComposeMachineSuckerStates sucker_states);
  void signComposeMachineFittingPlane();

  // 缝纫机
  void signSewingMachineReset();
  void signSewingMachineStop();
  void signSewingMachineMoveHand(float x, float y, float c, bool z);
  void signSewingMachineSendKeypoints(syt_msgs::msg::ClothKeypoints2f keypoints);
  void signSewingMachineNeedle(float shoulder_length, float side_length);

  // 其他
  void signChooseMode(int mode); // 模式切换
  void signUpdateLoadMachine();  // 更新固件
  void signUpdateComposeMachine();
  void signUpdateSewingMachine();
  void signCheckCalib();   // 检测标定结果
  void signEmegencyStop(); // 急停

  // 急停
public slots:
  void setComposeMachineState(syt_msgs::msg::ComposeMachineState state);
  void setSewingMachineState(syt_msgs::msg::SewingMachineState state);
  void setCheckCalibrationResult(bool result, float bottom_length, float side_length);
};
