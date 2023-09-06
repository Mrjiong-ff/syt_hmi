#include "syt_rclcomm/rcl_comm.h"

SytRclComm::SytRclComm(QObject *parent) : QThread(parent), rate_(100) {
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  node_     = rclcpp::Node::make_shared("syt_hmi_node");
  executor_->add_node(node_);

  run_mode_.mode = syt_msgs::msg::FSMRunMode::LOOP_ONCE;

  // 设备状态订阅
  // load_machine_state_subscription_    = node_->create_subscription<syt_msgs::msg::LoadMachineState>("/syt/robot_control/load_machine/state", 10, std::bind(&SytRclComm::loadMachineStateCallback, this, _1));
  compose_machine_state_subscription_ = node_->create_subscription<syt_msgs::msg::ComposeMachineState>("/syt/robot_control/compose_machine/state", 10, std::bind(&SytRclComm::composeMachineStateCallback, this, _1));
  sewing_machine_state_subscription_  = node_->create_subscription<syt_msgs::msg::SewingMachineState>("/syt/robot_control/sewing_machine/state", 10, std::bind(&SytRclComm::sewingMachineStateCallback, this, _1));

  // 固件下载会回调
  download_subscription_ = node_->create_subscription<std_msgs::msg::Int32>("/syt/ota/ftp_topic", 10, std::bind(&SytRclComm::downloadCallback, this, _1));

  // 上料机视觉显示回调
  load_cloth_visual_subscription_ = node_->create_subscription<syt_msgs::msg::LoadClothVisual>("/syt/cloth_edge_pydetect/cloth_edge_visual_topic", 10, std::bind(&SytRclComm::loadClothVisualCallback, this, _1));

  // rosout消息回调函数
  log_subscription_ = node_->create_subscription<rcl_interfaces::msg::Log>("/rosout", 10, std::bind(&SytRclComm::logCallback, this, _1));

  // 运行状态回调函数
  run_state_subscription_ = node_->create_subscription<syt_msgs::msg::FSMState>("/syt/motion_planner/run_state", 10, std::bind(&SytRclComm::runStateCallback, this, _1));

  // 计数回调
  run_count_subscription_ = node_->create_subscription<std_msgs::msg::UInt64>("/syt/motion_planner/run_count", 10, std::bind(&SytRclComm::runCountCallback, this, _1));

  // 错误码回调
  error_code_subscription_ = node_->create_subscription<syt_msgs::msg::ErrorCode>("/syt/diagnostic_system/exception_info", 10, std::bind(&SytRclComm::errorCodeCallback, this, _1));

  // 开始停止复位
  fsm_flow_control_cmd_publisher_ = node_->create_publisher<syt_msgs::msg::FSMFlowControlCommand>("/syt/robot_control/flow_control_cmd", 10);

  // 模式选择
  fsm_run_mode_publisher_ = node_->create_publisher<syt_msgs::msg::FSMRunMode>("/syt/robot_control/running_mode", 10);

  this->start();
}

SytRclComm::~SytRclComm() {
  if (process_ != nullptr) {
    process_->terminate();
    process_->waitForFinished();
    delete process_;
  }

  rclcpp::shutdown();

  for (int i = 0; i < 3; ++i) {
    killProcesses("thanos.launch.py");
    killProcesses("ros-args");
  }
}

void SytRclComm::run() {
  rclcpp::WallRate rate(50);

  while (rclcpp::ok()) {
    executor_->spin_some();
    rate.sleep();
  }
  rclcpp::shutdown();
}

bool SytRclComm::initAllNodes() {
  // 设置要启动的ROS 2程序和参数，例如ros2的话题列表程序
  QString program = "/bin/zsh"; // 或者 "/bin/bash"，根据实际情况选择合适的shell
  QStringList arguments;
  arguments << "-c"
            << "source /opt/ros/foxy/setup.zsh && echo $SYT_HOME && cd $SYT_HOME && export ENV_ROBOT_ETC=$(pwd)/install/etc/ && source install/setup.zsh &&  ros2 launch syt_common thanos.launch.py";
  process_ = new QProcess(this);

  process_->start(program, arguments);

  if (process_->waitForStarted()) {
    // 获取输出结果
    QByteArray output = process_->readAllStandardOutput();
    return true;
  }

  auto error_msg = process_->readAllStandardError().toStdString();
  if (error_msg.empty()) {
    return true;
  }
  QString msg = QString("Fatal: 节点初始化失败.\n错误消息: %1").arg(error_msg.data());
  emit errorNodeMsgSign(msg);
  return false;
}

void SytRclComm::otaUpdate() {
  auto request  = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;

  std_srvs::srv::SetBool::Response response;
  CALL_RESULT result = callService<std_srvs::srv::SetBool>("/syt/ota/update", "OTA更新", 10000, request, response);
  qDebug() << "OTA更新：" << result;
  switch (result) {
  case CALL_SUCCESS:
    if (response.success) {
      qDebug() << "更新包总大小: " << QString(response.message.data()).toInt();
    }
    emit waitUpdateResultSuccess(response.success, QString(response.message.data()));
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    emit waitUpdateResultSuccess(false, QString(""));
    break;
  }
}

void SytRclComm::otaDownload() {
  emit processZero();

  auto request  = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;

  std_srvs::srv::SetBool::Response response;
  CALL_RESULT result = callService<std_srvs::srv::SetBool>("/syt/ota/download", "下载新版本", 10 * 60000, request, response);
  qDebug() << "下载新版本：" << result;
  switch (result) {
  case CALL_SUCCESS:
    emit downloadRes(response.success, QString(response.message.data()));
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    emit downloadRes(false, QString(""));
    break;
  }
}

// void SytRclComm::loadMachineStateCallback(const syt_msgs::msg::LoadMachineState::SharedPtr msg) {
//;
//}

void SytRclComm::composeMachineStateCallback(const syt_msgs::msg::ComposeMachineState::SharedPtr msg) {
  emit updateComposeMachineState(*msg);
}

void SytRclComm::sewingMachineStateCallback(const syt_msgs::msg::SewingMachineState::SharedPtr msg) {
  emit updateSewingMachineState(*msg);
}

void SytRclComm::downloadCallback(const std_msgs::msg::Int32::SharedPtr msg) {
  int val = msg.get()->data;
  emit updateProcess(val, total_size);
}

void SytRclComm::loadClothVisualCallback(const syt_msgs::msg::LoadClothVisual::SharedPtr msg) {
  auto machine_id = msg.get()->machine_id;
  auto cam_id     = msg.get()->cam_id;

  int machine_id_ = static_cast<int>(machine_id);
  int cam_id_     = static_cast<int>(cam_id);

  auto img_h    = msg.get()->image.height;
  auto img_w    = msg.get()->image.width;
  auto img_data = msg.get()->image.data;

  cv::Mat image(img_h, img_w, CV_8UC3, const_cast<uint8_t *>(img_data.data()), msg.get()->image.step);
  cv::cvtColor(image, image, cv::COLOR_BGR2BGRA);
  auto qimage = cvMat2QImage(image);
  emit visualLoadClothRes(machine_id_, cam_id_, qimage);
}

// OTA安装
void SytRclComm::otaInstall() {
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client = node_->create_client<std_srvs::srv::SetBool>("/syt/ota/install");
  // killProcesses() // TODO

  auto request  = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;
  auto result   = client->async_send_request(request);
  auto success  = result.get()->success;
  auto msg      = result.get()->message;
  emit installRes(success, QString(msg.data()));
}

// 合片机标定
void SytRclComm::compCalib() {
  auto request        = std::make_shared<syt_msgs::srv::RunCalibration::Request>();
  request->mode.state = request->mode.EXTER_COMPOSE;

  syt_msgs::srv::RunCalibration::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::RunCalibration>("/syt/calibration_system/calibration_service", "相机标定", 6 * 60000, request, response);
  qDebug() << "缝纫机相机标定：" << result;
  switch (result) {
  case CALL_SUCCESS:
    emit compCalibRes(response.success);
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    emit compCalibRes(false);
    break;
  }
}

// 缝纫机标定
void SytRclComm::sewingCalib() {
  auto request        = std::make_shared<syt_msgs::srv::RunCalibration::Request>();
  request->mode.state = request->mode.EXTER_SEWING;

  syt_msgs::srv::RunCalibration::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::RunCalibration>("/syt/calibration_system/calibration_service", "相机标定", 6 * 60000, request, response);
  qDebug() << "缝纫机相机标定：" << result;
  switch (result) {
  case CALL_SUCCESS:
    emit sewingCalibRes(response.success);
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    emit sewingCalibRes(false);
    break;
  }
}

void SytRclComm::logCallback(const rcl_interfaces::msg::Log::SharedPtr msg) {
  auto cur_time  = getCurrentTime();
  auto msg_data  = msg->msg.data();
  auto func      = msg->function.data();
  auto node_name = msg->name.data();
  emit signLogPub(QString(cur_time.c_str()), msg->level, QString(node_name), QString(func), QString(msg_data));
}

// 监听全流程运行状态
void SytRclComm::runStateCallback(const syt_msgs::msg::FSMState::SharedPtr msg) {
  switch (msg->state_code) {
  case syt_msgs::msg::FSMState::IDLE:
    if (start_flag_) {
      if (last_state_ != syt_msgs::msg::FSMState::IDLE) {
        emit finishOneRound();
        if (run_mode_.mode == syt_msgs::msg::FSMRunMode::LOOP_ONCE || run_mode_.mode == syt_msgs::msg::FSMRunMode::COMPOSE_CLOTH) {
          emit machineIdle();
        }
      }
    }
    break;
  case syt_msgs::msg::FSMState::PAUSE:
  case syt_msgs::msg::FSMState::STOP:
  case syt_msgs::msg::FSMState::RUN:
    break;
  default:
    break;
  }
  last_state_ = msg->state_code;
}

// 监听运行次数
void SytRclComm::runCountCallback(const std_msgs::msg::UInt64::SharedPtr msg) {
  uint64_t count = msg->data;
  emit signRunCount(count);
}

// 监听错误码
void SytRclComm::errorCodeCallback(const syt_msgs::msg::ErrorCode::SharedPtr msg) {
  uint32_t error_code = msg->data;
  int exception_level = (error_code & 0x00f00000) >> 20;
  emit signErrorLevel(exception_level);
  current_exception_level_ = exception_level;
}

template <class T>
SytRclComm::CALL_RESULT SytRclComm::callService(std::string srv_name, std::string info, uint32_t timeout_ms, std::shared_ptr<typename T::Request> request, typename T::Response &response) {
  typename rclcpp::Client<T>::SharedPtr client = node_->create_client<T>(srv_name);

  int try_count = 0;
  while (!client->wait_for_service(1s)) {
    if (try_count++ >= 5) {
      RCLCPP_INFO(node_->get_logger(), "无法连接至" + info + "服务超过限制次数，停止连接...");
      return CALL_DISCONNECT;
    }
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(), "连接至" + info + "服务被打断");
      return CALL_INTERRUPT;
    }
    RCLCPP_WARN(node_->get_logger(), "无法连接至" + info + "服务，重试...");
  }

  auto result = client->async_send_request(request);

  auto begin_time = std::chrono::high_resolution_clock::now();
  while (result.wait_for(50ms) != std::future_status::ready) {
    auto current_time  = std::chrono::high_resolution_clock::now();
    uint32_t cost_time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - begin_time).count();
    if (cost_time > timeout_ms) {
      RCLCPP_INFO(node_->get_logger(), info + "服务调用超时");
      return CALL_TIMEOUT;
    }
  }

  response = *result.get();
  return CALL_SUCCESS;
}

void SytRclComm::resetCmd() {
  start_flag_                  = false;
  auto request                 = std::make_shared<syt_msgs::srv::FSMControlFlow::Request>();
  request->control_cmd.command = syt_msgs::msg::FSMFlowControlCommand::RESET;

  syt_msgs::srv::FSMControlFlow::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::FSMControlFlow>("/syt/motion_planner/control_flow", "整机复位", 60000, request, response);
  qDebug() << "整机复位：" << result;
  switch (result) {
  case CALL_SUCCESS:
    emit signResetFinish(response.success);
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    emit signResetFinish(false);
    break;
  }
}

void SytRclComm::startCmd() {
  start_flag_                  = true;
  auto request                 = std::make_shared<syt_msgs::srv::FSMControlFlow::Request>();
  request->control_cmd.command = syt_msgs::msg::FSMFlowControlCommand::RUN;

  syt_msgs::srv::FSMControlFlow::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::FSMControlFlow>("/syt/motion_planner/control_flow", "整机启动", 5000, request, response);
  qDebug() << "整机启动：" << result;
  switch (result) {
  case CALL_SUCCESS:
    emit signStartFinish(response.success);
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    emit signStartFinish(false);
    break;
  }
}

void SytRclComm::pauseCmd() {
  start_flag_                  = false;
  auto request                 = std::make_shared<syt_msgs::srv::FSMControlFlow::Request>();
  request->control_cmd.command = syt_msgs::msg::FSMFlowControlCommand::PAUSE;

  syt_msgs::srv::FSMControlFlow::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::FSMControlFlow>("/syt/motion_planner/control_flow", "整机暂停", 5000, request, response);
  qDebug() << "整机暂停：" << result;
  switch (result) {
  case CALL_SUCCESS:
    emit signPauseFinish(response.success);
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    emit signPauseFinish(false);
    break;
  }
}

void SytRclComm::stopCmd() {
  start_flag_                  = false;
  auto request                 = std::make_shared<syt_msgs::srv::FSMControlFlow::Request>();
  request->control_cmd.command = syt_msgs::msg::FSMFlowControlCommand::STOP;

  syt_msgs::srv::FSMControlFlow::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::FSMControlFlow>("/syt/motion_planner/control_flow", "整机停止", 60000, request, response);
  qDebug() << "整机停止：" << result;
  switch (result) {
  case CALL_SUCCESS:
    emit signStopFinish(response.success);
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    emit signStopFinish(false);
    break;
  }
}

// 转换运行模式
void SytRclComm::changeMode(int mode) {
  auto request           = std::make_shared<syt_msgs::srv::FSMChangeMode::Request>();
  request->mode_cmd.mode = mode;

  syt_msgs::srv::FSMChangeMode::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::FSMChangeMode>("/syt/motion_planner/change_mode", "切换模式", 5000, request, response);
  qDebug() << "切换模式：" << result;
}

void SytRclComm::resetWholeMachine() {
  auto request          = std::make_shared<syt_msgs::srv::WholeMachineCMD::Request>();
  request->command.data = request->command.RESET;

  syt_msgs::srv::WholeMachineCMD::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::WholeMachineCMD>("/syt/robot_control/whole_machine/command", "整机流程", 10000, request, response);
  qDebug() << "整机停止：" << result;
  switch (result) {
  case CALL_SUCCESS:
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    break;
  }
}

void SytRclComm::stopWholeMachine() {
  auto request          = std::make_shared<syt_msgs::srv::WholeMachineCMD::Request>();
  request->command.data = request->command.STOP;

  syt_msgs::srv::WholeMachineCMD::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::WholeMachineCMD>("/syt/robot_control/whole_machine/command", "整机流程", 10000, request, response);
  qDebug() << "整机停止：" << result;
  switch (result) {
  case CALL_SUCCESS:
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    break;
  }
}

/* -----------------------------固件更新---------------------------- */
// 上料机
void SytRclComm::updateLoadMachine() {
  auto request    = std::make_shared<syt_msgs::srv::MCURestart::Request>();
  request->enable = true;

  syt_msgs::srv::MCURestart::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::MCURestart>("/syt/robot_control/load_machine/primal/mcu_restart", "上料机重启单片机", 20000, request, response);
  qDebug() << "上料机重启单片机：" << result;
  switch (result) {
  case CALL_SUCCESS:
    // emit signLoadMachineResetFinish(response.success, id);
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    // emit signLoadMachineResetFinish(false, id);
    break;
  }
}

// 合片机
void SytRclComm::updateComposeMachine() {
  auto request    = std::make_shared<syt_msgs::srv::MCURestart::Request>();
  request->enable = true;

  syt_msgs::srv::MCURestart::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::MCURestart>("/syt/robot_control/compose_machine/primal/mcu_restart", "合片机重启单片机", 20000, request, response);
  qDebug() << "合片机重启单片机：" << result;
  switch (result) {
  case CALL_SUCCESS:
    // emit signLoadMachineResetFinish(response.success, id);
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    // emit signLoadMachineResetFinish(false, id);
    break;
  }
}

// 缝纫机
void SytRclComm::updateSewingMachine() {
  auto request    = std::make_shared<syt_msgs::srv::MCURestart::Request>();
  request->enable = true;

  syt_msgs::srv::MCURestart::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::MCURestart>("/syt/robot_control/sewing_machine/primal/mcu_restart", "缝纫机重启单片机", 20000, request, response);
  qDebug() << "缝纫机重启单片机：" << result;
  switch (result) {
  case CALL_SUCCESS:
    // emit signLoadMachineResetFinish(response.success, id);
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    // emit signLoadMachineResetFinish(false, id);
    break;
  }
}

/* -----------------------------上料机---------------------------- */
// 上料机复位
void SytRclComm::loadMachineReset(int id) {
  auto request     = std::make_shared<syt_msgs::srv::LoadMachineReset::Request>();
  request->id.data = id;
  request->enable  = true;

  syt_msgs::srv::LoadMachineReset::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::LoadMachineReset>("/syt/robot_control/load_machine/primal/reset", "上料复位", 20000, request, response);
  qDebug() << "上料复位：" << result;
  switch (result) {
  case CALL_SUCCESS:
    emit signLoadMachineResetFinish(response.success, id);
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    emit signLoadMachineResetFinish(false, id);
    break;
  }
}

// 补料模式
void SytRclComm::loadMachineAddCloth(int id) {
  auto request     = std::make_shared<syt_msgs::srv::LoadMachineAddCloth::Request>();
  request->id.data = id;
  request->enable  = true;

  syt_msgs::srv::LoadMachineAddCloth::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::LoadMachineAddCloth>("/syt/robot_control/load_machine/primal/add_cloth", "补料模式", 15000, request, response);
  qDebug() << "补料模式：" << result;
  switch (result) {
  case CALL_SUCCESS:
    emit signLoadMachineAddClothFinish(response.success, id);
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    emit signLoadMachineAddClothFinish(false, id);
    break;
  }
}
// 清理台面
void SytRclComm::loadMachineClearTable(int id) {
  auto request     = std::make_shared<syt_msgs::srv::LoadMachineClearTable::Request>();
  request->id.data = id;
  request->enable  = true;

  syt_msgs::srv::LoadMachineClearTable::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::LoadMachineClearTable>("/syt/robot_control/load_machine/primal/clear_table", "清理台面", 15000, request, response);
  qDebug() << "清理台面：" << result;
  switch (result) {
  case CALL_SUCCESS:
    emit signLoadMachineClearTableFinish(response.success, id);
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    emit signLoadMachineAddClothFinish(false, id);
    break;
  }
}

// 裁片尺寸
void SytRclComm::loadMachineClothSize(int id, uint32_t width, uint32_t length) {
  auto request     = std::make_shared<syt_msgs::srv::LoadMachineClothSize::Request>();
  request->id.data = id;
  request->width   = width;
  request->length  = length;

  syt_msgs::srv::LoadMachineClothSize::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::LoadMachineClothSize>("/syt/robot_control/load_machine/primal/cloth_size", "裁片尺寸", 5000, request, response);
  qDebug() << "裁片尺寸：" << result;
  switch (result) {
  case CALL_SUCCESS:
    emit signLoadMachineClothSizeFinish(response.success, id);
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    emit signLoadMachineClothSizeFinish(false, id);
    break;
  }
}

// 上料行程
void SytRclComm::loadMachineLoadDistance(int id, uint32_t distance) {
  auto request      = std::make_shared<syt_msgs::srv::LoadMachineLoadDistance::Request>();
  request->id.data  = id;
  request->distance = distance;

  syt_msgs::srv::LoadMachineLoadDistance::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::LoadMachineLoadDistance>("/syt/robot_control/load_machine/primal/load_distance", "上料行程", 5000, request, response);
  qDebug() << "上料行程：" << result;
  switch (result) {
  case CALL_SUCCESS:
    emit signLoadMachineLoadDistanceFinish(response.success, id);
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    emit signLoadMachineLoadDistanceFinish(false, id);
    break;
  }
}

// 夹爪偏移
void SytRclComm::loadMachineOffset(int id, int offset) {
  auto request     = std::make_shared<syt_msgs::srv::LoadMachineOffset::Request>();
  request->id.data = id;
  request->offset  = offset;

  syt_msgs::srv::LoadMachineOffset::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::LoadMachineOffset>("/syt/robot_control/load_machine/primal/offset", "上料夹爪偏移", 5000, request, response);
  qDebug() << "上料夹爪偏移：" << result;
  switch (result) {
  case CALL_SUCCESS:
    emit signLoadMachineOffsetFinish(response.success, id);
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    emit signLoadMachineOffsetFinish(false, id);
    break;
  }
}

// 上料间隔
void SytRclComm::loadMachineTrayGap(int id, uint32_t height) {
  auto request     = std::make_shared<syt_msgs::srv::LoadMachineTrayGap::Request>();
  request->id.data = id;
  request->height  = height;

  syt_msgs::srv::LoadMachineTrayGap::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::LoadMachineTrayGap>("/syt/robot_control/load_machine/primal/tray_gap", "上料间隔", 10000, request, response);
  qDebug() << "上料间隔：" << result;
  switch (result) {
  case CALL_SUCCESS:
    emit signLoadMachineTrayGapFinish(response.success, id);
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    emit signLoadMachineTrayGapFinish(false, id);
    break;
  }
}

// 抓住裁片
void SytRclComm::loadMachineHoldCloth(int id) {
  auto request        = std::make_shared<syt_msgs::srv::LoadMachineGrabCloth::Request>();
  request->id.data    = id;
  request->grab_cloth = 0;

  syt_msgs::srv::LoadMachineGrabCloth::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::LoadMachineGrabCloth>("/syt/robot_control/load_machine/primal/grab_cloth", "抓住裁片", 10000, request, response);
  qDebug() << "抓住裁片：" << result;
  switch (result) {
  case CALL_SUCCESS:
    emit signLoadMachineHoldClothFinish(response.success, id);
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    emit signLoadMachineHoldClothFinish(false, id);
    break;
  }
}

// 粗对位
void SytRclComm::loadMachineRoughAlign(int id) {
  auto request     = std::make_shared<syt_msgs::srv::LoadMachineRoughAlign::Request>();
  request->id.data = id;
  request->enable  = true;

  syt_msgs::srv::LoadMachineRoughAlign::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::LoadMachineRoughAlign>("/syt/robot_control/load_machine/primal/rough_align", "粗对位", 5000, request, response);
  qDebug() << "粗对位：" << result;
  switch (result) {
  case CALL_SUCCESS:
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    break;
  }
}

// 上裁片
void SytRclComm::loadMachineGrabCloth(int id) {
  auto request        = std::make_shared<syt_msgs::srv::LoadMachineGrabCloth::Request>();
  request->id.data    = id;
  request->grab_cloth = 1;

  syt_msgs::srv::LoadMachineGrabCloth::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::LoadMachineGrabCloth>("/syt/robot_control/load_machine/primal/grab_cloth", "上裁片", 20000, request, response);
  qDebug() << "上裁片：" << result;
  switch (result) {
  case CALL_SUCCESS:
    emit signLoadMachineGrabClothFinish(response.success, id);
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    emit signLoadMachineGrabClothFinish(false, id);
    break;
  }
}

// 预备设置
void SytRclComm::loadMachinePreSetup(int id) {
  auto request     = std::make_shared<syt_msgs::srv::LoadMachinePreSetup::Request>();
  request->id.data = id;
  request->enable  = true;

  syt_msgs::srv::LoadMachinePreSetup::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::LoadMachinePreSetup>("/syt/robot_control/load_machine/pre_setup", "预备设置", 50000, request, response);
  qDebug() << "预备设置：" << result;
  switch (result) {
  case CALL_SUCCESS:
    emit signLoadMachinePreSetupFinish(response.success, id);
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    emit signLoadMachinePreSetupFinish(false, id);
    break;
  }
}

// 视觉对位
void SytRclComm::loadMachineVisualAlign(int id) {
  auto request     = std::make_shared<syt_msgs::srv::LoadMachineLoadCloth::Request>();
  request->id.data = id;

  syt_msgs::srv::LoadMachineLoadCloth::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::LoadMachineLoadCloth>("/syt/robot_control/load_machine/load_cloth", "视觉对位", 10000, request, response);
  qDebug() << "视觉对位：" << result;
  switch (result) {
  case CALL_SUCCESS:
    emit signLoadMachineVisualAlignFinish(response.success, id);
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    emit signLoadMachineVisualAlignFinish(false, id);
    break;
  }
}

/* ---------------------------合片机------------------------------ */
// 合片机复位
void SytRclComm::composeMachineReset() {
  auto request    = std::make_shared<syt_msgs::srv::ComposeMachineReset::Request>();
  request->enable = true;

  syt_msgs::srv::ComposeMachineReset::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::ComposeMachineReset>("/syt/robot_control/compose_machine/primal/reset", "合片复位", 20000, request, response);
  qDebug() << "合片复位：" << result;
  switch (result) {
  case CALL_SUCCESS:
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    break;
  }
}

// 停止
void SytRclComm::composeMachineStop() {
}

// 除褶
void SytRclComm::composeMachineWipeFold() {
  auto request                = std::make_shared<syt_msgs::srv::ComposeMachineFunction::Request>();
  request->commands.wipe_fold = true;

  syt_msgs::srv::ComposeMachineFunction::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::ComposeMachineFunction>("/syt/robot_control/compose_machine/primal/function", "除褶", 10000, request, response);
  qDebug() << "除褶：" << result;
  switch (result) {
  case CALL_SUCCESS:
    emit signComposeMachineWipeFoldFinish(response.success);
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    emit signComposeMachineWipeFoldFinish(false);
    break;
  }
}

// 出针
void SytRclComm::composeMachineExtendNeedle() {
  auto request                           = std::make_shared<syt_msgs::srv::ComposeMachineFunction::Request>();
  request->commands.extend_sucker_needle = true;

  syt_msgs::srv::ComposeMachineFunction::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::ComposeMachineFunction>("/syt/robot_control/compose_machine/primal/function", "出针", 5000, request, response);
  qDebug() << "出针：" << result;
  switch (result) {
  case CALL_SUCCESS:
    emit signComposeMachineExtendNeedleFinish(response.success);
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    emit signComposeMachineExtendNeedleFinish(false);
    break;
  }
}

// 收针
void SytRclComm::composeMachineWithdrawNeedle() {
  auto request                           = std::make_shared<syt_msgs::srv::ComposeMachineFunction::Request>();
  request->commands.extend_sucker_needle = false;

  syt_msgs::srv::ComposeMachineFunction::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::ComposeMachineFunction>("/syt/robot_control/compose_machine/primal/function", "收针", 5000, request, response);
  qDebug() << "收针：" << result;
  switch (result) {
  case CALL_SUCCESS:
    emit signComposeMachineWithdrawNeedleFinish(response.success);
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    emit signComposeMachineWithdrawNeedleFinish(false);
    break;
  }
}

// 吹气
void SytRclComm::composeMachineBlowWind() {
  auto request                            = std::make_shared<syt_msgs::srv::ComposeMachineFunction::Request>();
  request->commands.pop_cylinder_bottom   = true;
  request->commands.pop_cylinder_chest    = true;
  request->commands.pop_cylinder_shoulder = true;

  syt_msgs::srv::ComposeMachineFunction::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::ComposeMachineFunction>("/syt/robot_control/compose_machine/primal/function", "吹气", 5000, request, response);
  qDebug() << "吹气：" << result;
  switch (result) {
  case CALL_SUCCESS:
    emit signComposeMachineBlowWindFinish(response.success);
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    emit signComposeMachineBlowWindFinish(false);
    break;
  }
}

// 停气
void SytRclComm::composeMachineStopBlow() {
  auto request                            = std::make_shared<syt_msgs::srv::ComposeMachineFunction::Request>();
  request->commands.pop_cylinder_bottom   = false;
  request->commands.pop_cylinder_chest    = false;
  request->commands.pop_cylinder_shoulder = false;

  syt_msgs::srv::ComposeMachineFunction::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::ComposeMachineFunction>("/syt/robot_control/compose_machine/primal/function", "停气", 5000, request, response);
  qDebug() << "停气：" << result;
  switch (result) {
  case CALL_SUCCESS:
    emit signComposeMachineStopBlowFinish(response.success);
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    emit signComposeMachineStopBlowFinish(false);
    break;
  }
}

// 开吸风台
void SytRclComm::composeMachineFastenSheet() {
  auto request                   = std::make_shared<syt_msgs::srv::ComposeMachineFunction::Request>();
  request->commands.fasten_sheet = true;

  syt_msgs::srv::ComposeMachineFunction::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::ComposeMachineFunction>("/syt/robot_control/compose_machine/primal/function", "开吸风台", 5000, request, response);
  qDebug() << "开吸风台：" << result;
  switch (result) {
  case CALL_SUCCESS:
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    break;
  }
}

// 关吸风台
void SytRclComm::composeMachineUnfastenSheet() {
  auto request                   = std::make_shared<syt_msgs::srv::ComposeMachineFunction::Request>();
  request->commands.fasten_sheet = false;

  syt_msgs::srv::ComposeMachineFunction::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::ComposeMachineFunction>("/syt/robot_control/compose_machine/primal/function", "关吸风台", 5000, request, response);
  qDebug() << "关吸风台：" << result;
  switch (result) {
  case CALL_SUCCESS:
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    break;
  }
}

// 合片抓手移动
void SytRclComm::composeMachineMoveHand(float x, float y, float z, float c) {
  //// TODO: delete
  // emit signComposeMachineMoveHandFinish(true);
  // return;

  auto request      = std::make_shared<syt_msgs::srv::ComposeMachineMoveHand::Request>();
  request->target.x = x;
  request->target.y = y;
  request->target.z = z;
  request->target.c = c;

  syt_msgs::srv::ComposeMachineMoveHand::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::ComposeMachineMoveHand>("/syt/robot_control/compose_machine/move_hand", "移动合片抓手", 10000, request, response);
  qDebug() << "移动合片抓手：" << result;
  switch (result) {
  case CALL_SUCCESS:
    emit signComposeMachineMoveHandFinish(response.success);
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    emit signComposeMachineMoveHandFinish(false);
    break;
  }
}

// 移动吸盘
void SytRclComm::composeMachineMoveSucker(syt_msgs::msg::ComposeMachineSuckerStates sucker_states) {
  auto request    = std::make_shared<syt_msgs::srv::ComposeMachineMoveSucker::Request>();
  request->target = sucker_states;

  syt_msgs::srv::ComposeMachineMoveSucker::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::ComposeMachineMoveSucker>("/syt/robot_control/compose_machine/move_sucker", "移动合片吸盘", 10000, request, response);
  qDebug() << "移动合片吸盘：" << result;
  switch (result) {
  case CALL_SUCCESS:
    emit signComposeMachineMoveSuckerFinish(response.success);
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    emit signComposeMachineMoveSuckerFinish(false);
    break;
  }
}

/* -----------------------------缝纫机---------------------------- */
// 缝纫机复位
void SytRclComm::sewingMachineReset() {
  auto request    = std::make_shared<syt_msgs::srv::SewingMachineReset::Request>();
  request->enable = true;

  syt_msgs::srv::SewingMachineReset::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::SewingMachineReset>("/syt/robot_control/sewing_machine/primal/reset", "缝纫复位", 20000, request, response);
  qDebug() << "缝纫复位：" << result;
  switch (result) {
  case CALL_SUCCESS:
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    break;
  }
}

// 移动抓手
void SytRclComm::sewingMachineMoveHand(float x, float y, float c, bool z) {
  auto request      = std::make_shared<syt_msgs::srv::SewingMachineMoveHand::Request>();
  request->target.x = x;
  request->target.y = y;
  request->target.c = c;
  request->target.z = z;

  syt_msgs::srv::SewingMachineMoveHand::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::SewingMachineMoveHand>("/syt/robot_control/sewing_machine/move_hand", "移动缝纫抓手", 20000, request, response);
  qDebug() << "移动缝纫抓手：" << result;
  switch (result) {
  case CALL_SUCCESS:
    emit signSewingMachineMoveHandFinish(response.success);
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    emit signSewingMachineMoveHandFinish(false);
    break;
  }
}

// 发送关键点
void SytRclComm::sewingMachineSendKeypoints(syt_msgs::msg::ClothKeypoints2f keypoints) {
  auto request       = std::make_shared<syt_msgs::srv::SewingMachineKeypoints::Request>();
  request->keypoints = keypoints;

  syt_msgs::srv::SewingMachineKeypoints::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::SewingMachineKeypoints>("/syt/robot_control/sewing_machine/primal/keypoints", "发送关键点", 5000, request, response);
  qDebug() << "发送关键点：" << result;
  switch (result) {
  case CALL_SUCCESS:
    emit signSewingMachineSendKeypointsFinish(response.success);
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    emit signSewingMachineSendKeypointsFinish(false);
    break;
  }
}

/* ---------------------------视觉检测------------------------------ */
// 获取衣服信息
void SytRclComm::getClothInfo(uint8_t frame_id, int cloth_type) {
  //// TODO: delete
  // emit signGetClothInfoFinish(true, cloth_type, syt_msgs::msg::ClothInfo());
  // return;

  auto request           = std::make_shared<syt_msgs::srv::GetClothInfo::Request>();
  request->frame_id.data = frame_id; // 0 为相机系 1 为合片机 2 为缝纫机
  request->compare_flag  = 0;

  syt_msgs::srv::GetClothInfo::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::GetClothInfo>("/syt/clothes_detector/get_cloth_info", "检测裁片", 8000, request, response);
  qDebug() << "检测裁片：" << result;
  switch (result) {
  case CALL_SUCCESS:
    emit signGetClothInfoFinish(response.success, cloth_type, response.info);
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    emit signGetClothInfoFinish(false, cloth_type, syt_msgs::msg::ClothInfo());
    break;
  }
}

/* ---------------------------样式相关------------------------------ */
// 创建样式文件
void SytRclComm::createStyle(int mode, QString prefix, syt_msgs::msg::ClothStyle cloth_style_front, syt_msgs::msg::ClothStyle cloth_style_back) {
  auto request               = std::make_shared<syt_msgs::srv::CreateStyle::Request>();
  request->mode.data         = mode;
  request->prefix            = prefix.toStdString();
  request->file_name         = "";
  request->cloth_style_front = cloth_style_front;
  request->cloth_style_back  = cloth_style_back;

  syt_msgs::srv::CreateStyle::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::CreateStyle>("/syt/style_base/create_style", "创建样式", 5000, request, response);
  qDebug() << "创建样式结果：" << result;
  switch (result) {
  case CALL_SUCCESS:
    emit signCreateStyleFinish(response.success, QString(response.file_name.c_str()));
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    emit signCreateStyleFinish(false, "");
    break;
  }
}

// 重命名样式文件
void SytRclComm::renameClothStyle(QString old_name, QString new_name) {
  auto request           = std::make_shared<syt_msgs::srv::RenameClothStyle::Request>();
  request->file_name_old = old_name.toStdString();
  request->file_name_new = new_name.toStdString();

  syt_msgs::srv::RenameClothStyle::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::RenameClothStyle>("/syt/style_base/rename_cloth_style", "重命名", 5000, request, response);
  qDebug() << "重命名结果：" << result;
  switch (result) {
  case CALL_SUCCESS:
    emit signRenameClothStyleFinish(response.success);
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    emit signRenameClothStyleFinish(false);
    break;
  }
}

void SytRclComm::setCurrentStyle(QString prefix, QString file_name) {
  auto request       = std::make_shared<syt_msgs::srv::SetCurrentClothStyle::Request>();
  request->prefix    = prefix.toStdString();
  request->file_name = file_name.toStdString();

  syt_msgs::srv::SetCurrentClothStyle::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::SetCurrentClothStyle>("/syt/style_base/set_current_cloth_style", "设置当前样式", 5000, request, response);
  qDebug() << "设置当前样式：" << result;
  switch (result) {
  case CALL_SUCCESS:
    emit signSetCurrentClothStyleFinish(response.success);
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    emit signSetCurrentClothStyleFinish(false);
    break;
  }
}

void SytRclComm::getClothStyle(QString prefix, QString file_name) {
  auto request       = std::make_shared<syt_msgs::srv::GetClothStyle::Request>();
  request->prefix    = prefix.toStdString();
  request->file_name = file_name.toStdString();

  syt_msgs::srv::GetClothStyle::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::GetClothStyle>("/syt/style_base/get_cloth_style", "获取样式", 5000, request, response);
  qDebug() << "获取样式：" << result;
  switch (result) {
  case CALL_SUCCESS:
    emit signGetClothStyleFinish(response.success, response.cloth_style_front, response.cloth_style_back);
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    emit signGetClothStyleFinish(false, syt_msgs::msg::ClothStyle(), syt_msgs::msg::ClothStyle());
    break;
  }
}
