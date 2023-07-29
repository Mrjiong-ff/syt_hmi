#pragma once
#include "rcl_interfaces/msg/log.hpp"
#include "syt_msgs/msg/calib_state.hpp"
#include "syt_msgs/msg/cloth_style.hpp"
#include "syt_msgs/msg/fsm_flow_control_command.hpp"
#include "syt_msgs/msg/fsm_run_mode.hpp"
#include "syt_msgs/msg/load_cloth_visual.hpp"
#include "syt_msgs/srv/compose_machine_move_hand.hpp"
#include "syt_msgs/srv/create_style.hpp"
#include "syt_msgs/srv/get_break_point_y.hpp"
#include "syt_msgs/srv/get_cloth_info.hpp"
#include "syt_msgs/srv/get_cloth_style.hpp"
#include "syt_msgs/srv/rename_cloth_style.hpp"
#include "syt_msgs/srv/run_calibration.hpp"
#include "syt_msgs/srv/set_current_cloth_style.hpp"
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
  int total_size = 0;

  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::shared_ptr<rclcpp::Node> node_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr download_subscription_;
  rclcpp::Subscription<syt_msgs::msg::LoadClothVisual>::SharedPtr load_cloth_visual_subscription_;

  // todo
  rclcpp::Subscription<syt_msgs::msg::LoadClothVisual>::SharedPtr composer_visual_subscription_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr log_subscription_;

  rclcpp::Publisher<syt_msgs::msg::FSMFlowControlCommand>::SharedPtr fsm_flow_control_cmd_publisher_;
  rclcpp::Publisher<syt_msgs::msg::FSMRunMode>::SharedPtr fsm_run_mode_publisher_;

  QProcess *process_ = nullptr;

  void download_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void loadClothVisualCallback(const syt_msgs::msg::LoadClothVisual::SharedPtr msg);
  // void loadClothVisualCallback(const syt_msgs::msg::LoadClothVisual::SharedPtr msg);
  void killProcesses(std::string);
  void logCallback(const rcl_interfaces::msg::Log::SharedPtr msg);

protected:
  void run() override;

public:
  SytRclComm();
  ~SytRclComm() override;

  bool initAllNodes();
  void otaUpdate();
  void otaDownload();
  void otaInstall();
  void compCalib();
  void sewingCalib();
  void startCmd();
  void resetCmd();
  void stopCmd();
  void composeMachineMoveHand(float x, float y, float z, float c);
  void composeMachineDetectCloth(uint8_t frame_id, int cloth_type);
  void createStyle(int mode, syt_msgs::msg::ClothStyle cloth_style_front, syt_msgs::msg::ClothStyle cloth_style_back);
  void renameClothStyle(QString old_name, QString new_name);
  void setCurrentStyle(QString prefix, QString file_name);
  void getClothStyle(QString prefix, QString file_name);

signals:
  void errorNodeMsgSign(QString msg);
  void waitUpdateResultSuccess(bool res, QString msg);
  void updateProcess(int, int);
  void processZero();
  void downloadRes(bool, QString);
  void installRes(bool, QString);
  void visualLoadClothRes(int, int, QImage);
  void signLogPub(QString, QString, QString, QString, QString);

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
};
