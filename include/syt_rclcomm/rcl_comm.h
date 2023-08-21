#pragma once
#include "rcl_interfaces/msg/log.hpp"
#include "syt_msgs/msg/calib_state.hpp"
#include "syt_msgs/msg/cloth_keypoints2f.hpp"
#include "syt_msgs/msg/cloth_style.hpp"
#include "syt_msgs/msg/compose_machine_state.hpp"
#include "syt_msgs/msg/fsm_flow_control_command.hpp"
#include "syt_msgs/msg/fsm_run_mode.hpp"
#include "syt_msgs/msg/load_cloth_visual.hpp"
#include "syt_msgs/msg/motion_planner_state.hpp"
#include "syt_msgs/msg/sewing_machine_state.hpp"
#include "syt_msgs/srv/compose_machine_function.hpp"
#include "syt_msgs/srv/compose_machine_move_hand.hpp"
#include "syt_msgs/srv/compose_machine_move_sucker.hpp"
#include "syt_msgs/srv/compose_machine_reset.hpp"
#include "syt_msgs/srv/create_style.hpp"
#include "syt_msgs/srv/get_break_point_y.hpp"
#include "syt_msgs/srv/get_cloth_info.hpp"
#include "syt_msgs/srv/get_cloth_style.hpp"
#include "syt_msgs/srv/load_machine_add_cloth.hpp"
#include "syt_msgs/srv/load_machine_clear_table.hpp"
#include "syt_msgs/srv/load_machine_cloth_size.hpp"
#include "syt_msgs/srv/load_machine_grab_cloth.hpp"
#include "syt_msgs/srv/load_machine_load_cloth.hpp"
#include "syt_msgs/srv/load_machine_load_distance.hpp"
#include "syt_msgs/srv/load_machine_offset.hpp"
#include "syt_msgs/srv/load_machine_pre_setup.hpp"
#include "syt_msgs/srv/load_machine_reset.hpp"
#include "syt_msgs/srv/load_machine_tray_gap.hpp"
#include "syt_msgs/srv/rename_cloth_style.hpp"
#include "syt_msgs/srv/run_calibration.hpp"
#include "syt_msgs/srv/set_current_cloth_style.hpp"
#include "syt_msgs/srv/sewing_machine_keypoints.hpp"
#include "syt_msgs/srv/sewing_machine_move_hand.hpp"
#include "syt_msgs/srv/whole_machine_cmd.hpp"
#include "utils/utils.h"
#include <QDebug>
#include <QMessageBox>
#include <QProcess>
#include <QThread>
#include <QWidget>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <unistd.h>

using namespace std::chrono_literals;
using namespace std::placeholders;

class SytRclComm : public QThread {
  Q_OBJECT

public:
  SytRclComm(QObject *parent = nullptr);
  ~SytRclComm() override;

  enum CALL_RESULT { CALL_SUCCESS    = 0,
                     CALL_TIMEOUT    = 1,
                     CALL_INTERRUPT  = 2,
                     CALL_DISCONNECT = 3 };
  Q_ENUM(CALL_RESULT);

  void startCmd();           // 开始全流程指令
  void resetCmd();           // 复位指令
  void stopCmd();            // 急停指令
  void changeMode(int mode); // 转换运行模式

  template <class T>
  CALL_RESULT callService(std::string srv_name, std::string info, uint32_t timeout_ms, std::shared_ptr<typename T::Request> request, typename T::Response &response);

  void resetWholeMachine(); // 复位整机
  void stopWholeMachine();  // 停止整机
  void changeBoard();       // 换压板模式

  void otaUpdate();   // OTA更新
  void otaDownload(); // 下载更新包
  void otaInstall();  // 安装到指定位置
  void compCalib();   // 合片标定
  void sewingCalib(); // 缝纫标定

  // 上料机
  void loadMachineReset(int id);                                      // 上料机复位
  void loadMachineAddCloth(int id);                                   // 补料模式
  void loadMachineClearTable(int id);                                 // 清理台面
  void loadMachineClothSize(int id, uint32_t width, uint32_t length); // 裁片尺寸
  void loadMachineLoadDistance(int id, uint32_t distance);            // 上料行程
  void loadMachineOffset(int id, int offset);                         // 夹爪偏移
  void loadMachineTrayGap(int id, uint32_t height);                   // 上料间隔
  void loadMachineHoldCloth(int id);                                  // 抓住裁片
  void loadMachineGrabCloth(int id);                                  // 上裁片
  void loadMachinePreSetup(int id);                                   // 预备设置
  void loadMachineVisualAlign(int id);                                // 视觉对位

  // 合片机
  void composeMachineReset();                                                             // 合片机复位
  void composeMachineStop();                                                              // 停止
  void composeMachineWipeFold();                                                          // 除褶
  void composeMachineExtendNeedle();                                                      // 出针
  void composeMachineWithdrawNeedle();                                                    // 收针
  void composeMachineBlowWind();                                                          // 吹气
  void composeMachineStopBlow();                                                          // 停气
  void composeMachineMoveHand(float x, float y, float z, float c);                        // 移动抓手
  void composeMachineMoveSucker(syt_msgs::msg::ComposeMachineSuckerStates sucker_states); // 移动吸盘

  // 缝纫机
  void sewingMachineMoveHand(float x, float y, float c, bool z);              // 移动抓手
  void sewingMachineSendKeypoints(syt_msgs::msg::ClothKeypoints2f keypoints); // 发送关键点

  // 视觉检测
  void getClothInfo(uint8_t frame_id, int cloth_type); // 获取衣服信息

  // 样式相关
  void createStyle(int mode, syt_msgs::msg::ClothStyle cloth_style_front, syt_msgs::msg::ClothStyle cloth_style_back); // 创建样式
  void renameClothStyle(QString old_name, QString new_name);                                                           // 重命名样式
  void setCurrentStyle(QString prefix, QString file_name);                                                             // 设置当前样式
  void getClothStyle(QString prefix, QString file_name);                                                               // 获取当前样式

protected:
  void run() override;

private:
  bool start_flag_ = false;
  int last_state_  = 0;
  syt_msgs::msg::FSMRunMode run_mode_;

  // total
  int total_size     = 0;
  QProcess *process_ = nullptr;

  //////////////// ros相关 ///////////////////
  syt_msgs::msg::FSMFlowControlCommand fsm_flow_control_command_;

  rclcpp::WallRate rate_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;

  // subscription
  rclcpp::Subscription<syt_msgs::msg::ComposeMachineState>::SharedPtr compose_machine_state_subscription_;
  rclcpp::Subscription<syt_msgs::msg::SewingMachineState>::SharedPtr sewing_machine_state_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr download_subscription_;
  rclcpp::Subscription<syt_msgs::msg::LoadClothVisual>::SharedPtr load_cloth_visual_subscription_;
  rclcpp::Subscription<syt_msgs::msg::LoadClothVisual>::SharedPtr composer_visual_subscription_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr log_subscription_;
  rclcpp::Subscription<syt_msgs::msg::MotionPlannerState>::SharedPtr run_state_subscription_;

  // publisher
  rclcpp::Publisher<syt_msgs::msg::FSMFlowControlCommand>::SharedPtr fsm_flow_control_cmd_publisher_;
  rclcpp::Publisher<syt_msgs::msg::FSMRunMode>::SharedPtr fsm_run_mode_publisher_;

private:
  void killProcesses(std::string);
  bool initAllNodes();

  void composeMachineStateCallback(const syt_msgs::msg::ComposeMachineState::SharedPtr msg);
  void sewingMachineStateCallback(const syt_msgs::msg::SewingMachineState::SharedPtr msg);
  void downloadCallback(const std_msgs::msg::Int32::SharedPtr msg);
  void loadClothVisualCallback(const syt_msgs::msg::LoadClothVisual::SharedPtr msg);
  void logCallback(const rcl_interfaces::msg::Log::SharedPtr msg);
  void runStateCallback(const syt_msgs::msg::MotionPlannerState::SharedPtr msg);

signals:
  void updateComposeMachineState(syt_msgs::msg::ComposeMachineState state);
  void updateSewingMachineState(syt_msgs::msg::SewingMachineState state);
  void errorNodeMsgSign(QString msg);
  void waitUpdateResultSuccess(bool res, QString msg);
  void updateProcess(int, int);
  void processZero();
  void downloadRes(bool, QString);
  void installRes(bool, QString);
  void visualLoadClothRes(int, int, QImage);
  void signLogPub(QString current_time, int level, QString, QString, QString);
  void machineIdle();
  void finishOneRound();

  // 标定相关信号
  void compCalibRes(bool);
  void sewingCalibRes(bool);

  // 控制上料机相关信号
  void signLoadMachineResetFinish(bool result, int id);        // 复位
  void signLoadMachineAddClothFinish(bool result, int id);     // 补料模式
  void signLoadMachineClearTableFinish(bool result, int id);   // 清理台面
  void signLoadMachineClothSizeFinish(bool result, int id);    // 裁片尺寸
  void signLoadMachineLoadDistanceFinish(bool result, int id); // 上料行程
  void signLoadMachineOffsetFinish(bool result, int id);       // 夹爪偏移
  void signLoadMachineTrayGapFinish(bool result, int id);      // 上料间隔
  void signLoadMachineHoldClothFinish(bool result, int id);    // 抓住裁片
  void signLoadMachineGrabClothFinish(bool result, int id);    // 上裁片
  void signLoadMachinePreSetupFinish(bool result, int id);     // 预备设置
  void signLoadMachineVisualAlignFinish(bool result, int id);  // 视觉对位

  // 控制合片机相关信号
  void signComposeMachineResetFinish(bool result);          // 合片机复位
  void signComposeMachineStopFinish(bool result);           // 停止
  void signComposeMachineWipeFoldFinish(bool result);       // 除褶
  void signComposeMachineExtendNeedleFinish(bool result);   // 出针
  void signComposeMachineWithdrawNeedleFinish(bool result); // 收针
  void signComposeMachineBlowWindFinish(bool result);       // 吹气
  void signComposeMachineStopBlowFinish(bool result);       // 停气
  void signComposeMachineMoveHandFinish(bool result);       // 移动抓手
  void signComposeMachineMoveSuckerFinish(bool result);     // 移动吸盘

  // 缝纫机相关信号
  void signSewingMachineMoveHandFinish(bool result);      // 移动抓手
  void signSewingMachineSendKeypointsFinish(bool result); // 发送关键点

  // 视觉检测相关信号
  void signGetClothInfoFinish(bool result, int cloth_type, syt_msgs::msg::ClothInfo cloth_info);

  // 样式相关信号
  void signCreateStyleFinish(bool result, QString file_name);
  void signRenameClothStyleFinish(bool result);
  void signSetCurrentClothStyleFinish(bool result);
  void signGetClothStyleFinish(bool result, syt_msgs::msg::ClothStyle cloth_style_front, syt_msgs::msg::ClothStyle cloth_style_back);
};
