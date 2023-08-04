#pragma once
#include "rcl_interfaces/msg/log.hpp"
#include "syt_msgs/msg/calib_state.hpp"
#include "syt_msgs/msg/cloth_style.hpp"
#include "syt_msgs/msg/fsm_flow_control_command.hpp"
#include "syt_msgs/msg/fsm_run_mode.hpp"
#include "syt_msgs/msg/load_cloth_visual.hpp"
#include "syt_msgs/msg/motion_planner_state.hpp"
#include "syt_msgs/srv/compose_machine_move_hand.hpp"
#include "syt_msgs/srv/create_style.hpp"
#include "syt_msgs/srv/get_break_point_y.hpp"
#include "syt_msgs/srv/get_cloth_info.hpp"
#include "syt_msgs/srv/get_cloth_style.hpp"
#include "syt_msgs/srv/load_machine_add_cloth.hpp"
#include "syt_msgs/srv/rename_cloth_style.hpp"
#include "syt_msgs/srv/run_calibration.hpp"
#include "syt_msgs/srv/set_current_cloth_style.hpp"
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

class SytRclComm : public QThread {
  Q_OBJECT
private:
  // total
  bool start_flag_ = false;

  int total_size = 0;

  QProcess *process_ = nullptr;

  //////////////// ros相关 ///////////////////
  syt_msgs::msg::FSMFlowControlCommand fsm_flow_control_command_;

  rclcpp::WallRate rate_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr download_subscription_;
  rclcpp::Subscription<syt_msgs::msg::LoadClothVisual>::SharedPtr load_cloth_visual_subscription_;

  // subscription
  rclcpp::Subscription<syt_msgs::msg::LoadClothVisual>::SharedPtr composer_visual_subscription_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr log_subscription_;
  rclcpp::Subscription<syt_msgs::msg::MotionPlannerState>::SharedPtr run_state_subscription_;

  // publisher
  rclcpp::Publisher<syt_msgs::msg::FSMFlowControlCommand>::SharedPtr fsm_flow_control_cmd_publisher_;
  rclcpp::Publisher<syt_msgs::msg::FSMRunMode>::SharedPtr fsm_run_mode_publisher_;

  bool initAllNodes();
  void downloadCallback(const std_msgs::msg::Int32::SharedPtr msg);
  void loadClothVisualCallback(const syt_msgs::msg::LoadClothVisual::SharedPtr msg);
  void killProcesses(std::string);
  void logCallback(const rcl_interfaces::msg::Log::SharedPtr msg);
  void runStateCallback(const syt_msgs::msg::MotionPlannerState::SharedPtr msg);

protected:
  void run() override;

public:
  SytRclComm();
  ~SytRclComm() override;

  void startCmd(); // 开始全流程指令
  void resetCmd(); // 复位指令
  void stopCmd();  // 急停指令

  void resetWholeMachine(); // 复位整机
  void stopWholeMachine();  // 停止整机
  void addCloth(int id);    // 补料模式
  void ChangeBoard();       // 换压板模式

  void otaUpdate();   // OTA更新
  void otaDownload(); // 下载更新包
  void otaInstall();  // 安装到指定位置
  void compCalib();   // 合片标定
  void sewingCalib(); // 缝纫标定

  void composeMachineMoveHand(float x, float y, float z, float c);                                                     // 移动抓手
  void composeMachineDetectCloth(uint8_t frame_id, int cloth_type);                                                    // 检测关键点
  void createStyle(int mode, syt_msgs::msg::ClothStyle cloth_style_front, syt_msgs::msg::ClothStyle cloth_style_back); // 创建样式
  void renameClothStyle(QString old_name, QString new_name);                                                           // 重命名样式
  void setCurrentStyle(QString prefix, QString file_name);                                                             // 设置当前样式
  void getClothStyle(QString prefix, QString file_name);                                                               // 获取当前样式

signals:
  void errorNodeMsgSign(QString msg);
  void waitUpdateResultSuccess(bool res, QString msg);
  void updateProcess(int, int);
  void processZero();
  void downloadRes(bool, QString);
  void installRes(bool, QString);
  void visualLoadClothRes(int, int, QImage);
  void signLogPub(QString, QString, QString, QString, QString);
  void machineIdle(bool idle); // 处于空闲状态

  // 标定相关信号
  void compCalibRes(bool);
  void sewingCalibRes(bool);

  // 控制相关信号
  void signComposeMachineMoveHandFinish(bool result);
  void signComposeMachineDetectClothFinish(bool result, int cloth_type, syt_msgs::msg::ClothInfo cloth_info);
  void signCreateStyleFinish(bool result, QString file_name);
  void signRenameClothStyleFinish(bool result);
  void signSetCurrentClothStyleFinish(bool result);
  void signGetClothStyleFinish(bool result, syt_msgs::msg::ClothStyle cloth_style_front, syt_msgs::msg::ClothStyle cloth_style_back);
  void signAddClothFinish(bool result, int id);
};
