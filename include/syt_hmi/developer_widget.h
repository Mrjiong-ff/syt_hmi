#pragma once

#include "syt_msgs/msg/cloth_keypoints2f.hpp"
#include "syt_msgs/msg/compose_machine_state.hpp"
#include "syt_msgs/msg/compose_machine_sucker_states.hpp"
#include "syt_msgs/msg/sewing_machine_state.hpp"
#include "syt_msgs/msg/fsm_run_mode.hpp"
#include <QWidget>

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

  // 移动窗口
  bool is_mouse_left_press_down_;
  QPoint mouse_pos_;

private:
  void setChooseMode();
  void setButtonFrame();
  void switchPage();
  void bindLoadMachine();
  void bindComposeMachine();
  void bindSewingMachine();

signals:
  // 模式切换
  void signChooseMode(int mode);

  // 上料机
  void signLoadMachineReset(int id);
  void signLoadMachineAddCloth(int id);
  void signLoadMachineClearTable(int id);
  void signLoadMachineClothSize(int id, uint32_t width, uint32_t height);
  void signLoadMachineLoadDistance(int id, uint32_t distance);
  void signLoadMachineTrayGap(int id, uint32_t height);
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
  void signComposeMachineMoveHand(float x, float y, float z, float c);
  void signComposeMachineMoveSucker(syt_msgs::msg::ComposeMachineSuckerStates sucker_states);

  // 缝纫机
  void signSewingMachineReset();
  void signSewingMachineStop();
  void signSewingMachineMoveHand(float x, float y, float c, bool z);
  void signSewingMachineSendKeypoints(syt_msgs::msg::ClothKeypoints2f keypoints);

public slots:
  void setComposeMachineState(syt_msgs::msg::ComposeMachineState state);
  void setSewingMachineState(syt_msgs::msg::SewingMachineState state);
};
