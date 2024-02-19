#include "syt_rclcomm/rcl_comm.h"

SytRclComm::SytRclComm(QObject *parent) : QThread(parent), rate_(100), last_state_(-1) {
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  node_ = rclcpp::Node::make_shared("syt_hmi_node");
  executor_->add_node(node_);

  run_mode_.mode = syt_msgs::msg::FSMRunMode::LOOP_ONCE;

  // 设备状态订阅
  // load_machine_state_subscription_    =
  // node_->create_subscription<syt_msgs::msg::LoadMachineState>("/syt/robot_control/load_machine/state",
  // 10, std::bind(&SytRclComm::loadMachineStateCallback, this, _1));
  compose_machine_state_subscription_ = node_->create_subscription<syt_msgs::msg::ComposeMachineState>("/syt/robot_control/compose_machine/state", 10, std::bind(&SytRclComm::composeMachineStateCallback, this, _1));
  sewing_machine_state_subscription_ = node_->create_subscription<syt_msgs::msg::SewingMachineState>("/syt/robot_control/sewing_machine/state", 10, std::bind(&SytRclComm::sewingMachineStateCallback, this, _1));

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
  fsm_flow_control_cmd_publisher_ =
      node_->create_publisher<syt_msgs::msg::FSMFlowControlCommand>("/syt/robot_control/flow_control_cmd", 10);

  // 模式选择
  fsm_run_mode_publisher_ = node_->create_publisher<syt_msgs::msg::FSMRunMode>("/syt/robot_control/running_mode", 10);

  // 构造参数类型映射表
  rosparam_type_to_sytparam_type_[rcl_interfaces::msg::ParameterType::PARAMETER_BOOL] = "uchar";
  rosparam_type_to_sytparam_type_[rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER] = "int32";
  rosparam_type_to_sytparam_type_[rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE] = "float64";
  sytparam_type_to_rosparam_type_["uchar"] = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  sytparam_type_to_rosparam_type_["char"] = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  sytparam_type_to_rosparam_type_["int32"] = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  sytparam_type_to_rosparam_type_["uint32"] = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  sytparam_type_to_rosparam_type_["float32"] = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  sytparam_type_to_rosparam_type_["float64"] = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;

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
            << "source /opt/ros/foxy/setup.zsh && echo $SYT_HOME && cd $SYT_HOME && "
               "export ENV_ROBOT_ETC=$(pwd)/install/etc/ && source install/setup.zsh "
               "&&  ros2 launch syt_common thanos.launch.py";
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
  QString msg =
      QString("Fatal: 节点初始化失败.\n错误消息: %1").arg(error_msg.data());
  emit errorNodeMsgSign(msg);
  return false;
}

void SytRclComm::otaUpdate() {
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;

  std_srvs::srv::SetBool::Response response;
  CALL_RESULT result = callService<std_srvs::srv::SetBool>("/syt/ota/update", "OTA更新", 10000, request, response);
  qDebug() << "OTA更新：" << result;
  switch (result) {
  case CALL_SUCCESS:
    if (response.success) {
      qDebug() << "更新包总大小: " << QString(response.message.data()).toInt();
    }
    emit waitUpdateResultSuccess(response.success,
                                 QString(response.message.data()));
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

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;

  std_srvs::srv::SetBool::Response response;
  CALL_RESULT result = callService<std_srvs::srv::SetBool>(
      "/syt/ota/download", "下载新版本", 10 * 60000, request, response);
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

// void SytRclComm::loadMachineStateCallback(const
// syt_msgs::msg::LoadMachineState::SharedPtr msg) {
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
  auto cam_id = msg.get()->cam_id;

  int machine_id_ = static_cast<int>(machine_id);
  int cam_id_ = static_cast<int>(cam_id);

  auto img_h = msg.get()->image.height;
  auto img_w = msg.get()->image.width;
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

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;
  auto result = client->async_send_request(request);
  auto success = result.get()->success;
  auto msg = result.get()->message;
  emit installRes(success, QString(msg.data()));
}

// 合片机标定
void SytRclComm::compCalib() {
  auto request = std::make_shared<syt_msgs::srv::RunCalibration::Request>();
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
  auto request = std::make_shared<syt_msgs::srv::RunCalibration::Request>();
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
  auto cur_time = getCurrentTime();
  auto msg_data = msg->msg.data();
  auto func = msg->function.data();
  auto node_name = msg->name.data();
  emit signLogPub(QString(cur_time.c_str()), msg->level, QString(node_name), QString(func), QString(msg_data));
}

// 监听全流程运行状态
void SytRclComm::runStateCallback(
    const syt_msgs::msg::FSMState::SharedPtr msg) {
  if (last_state_ != msg->state_code) {
    switch (msg->state_code) {
    case syt_msgs::msg::FSMState::IDLE:
      emit machineIdle();
      break;
    case syt_msgs::msg::FSMState::PAUSE:
      emit machinePause();
      break;
    case syt_msgs::msg::FSMState::STOP:
      start_flag_ = false;
      emit machineStop();
      break;
    case syt_msgs::msg::FSMState::RUN:
      emit machineRun();
      break;
    default:
      break;
    }
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
  dead_error_code = msg->data;
  int exception_level = (error_code & 0x00f00000) >> 20;
  emit signErrorLevel(exception_level);
  current_exception_level_ = exception_level;
  error_count++;
}

template <class T>
SytRclComm::CALL_RESULT SytRclComm::callService(std::string srv_name, std::string info, uint32_t timeout_ms, std::shared_ptr<typename T::Request> request, typename T::Response &response) {
  typename rclcpp::Client<T>::SharedPtr client = node_->create_client<T>(srv_name);

  int try_count_ = 0;
  while (!client->wait_for_service(800ms)) {
    try_count = try_count_;
    if (try_count_++ >= 5) {
      RCLCPP_INFO(node_->get_logger(), "无法连接至" + info + "服务超过限制次数，停止连接...");
      return CALL_DISCONNECT;
    }
    if (!rclcpp::ok()) {
      // try_count = 5;
      RCLCPP_ERROR(node_->get_logger(), "连接至" + info + "服务被打断");
      return CALL_INTERRUPT;
    }
    RCLCPP_WARN(node_->get_logger(), "无法连接至" + info + "服务，重试...");
  }

  auto result = client->async_send_request(request);

  if (timeout_ms == 0) {
    timeout_ms = UINT32_MAX;
  }

  auto begin_time = std::chrono::steady_clock::now();
  while (result.wait_for(50ms) != std::future_status::ready) {
    auto current_time = std::chrono::steady_clock::now();
    uint32_t cost_time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - begin_time).count();
    if (cost_time > timeout_ms) {
      // try_count = 5;
      RCLCPP_INFO(node_->get_logger(), info + "服务调用超时");
      return CALL_TIMEOUT;
    }
  }

  response = *result.get();
  return CALL_SUCCESS;
}

void SytRclComm::resetCmd() {
  start_flag_ = false;
  auto request = std::make_shared<syt_msgs::srv::FSMControlFlow::Request>();
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
  start_flag_ = true;
  auto request = std::make_shared<syt_msgs::srv::FSMControlFlow::Request>();
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
  start_flag_ = false;
  auto request = std::make_shared<syt_msgs::srv::FSMControlFlow::Request>();
  request->control_cmd.command = syt_msgs::msg::FSMFlowControlCommand::PAUSE;

  syt_msgs::srv::FSMControlFlow::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::FSMControlFlow>("/syt/motion_planner/control_flow", "整机暂停", 30000, request, response);
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
  start_flag_ = false;
  auto request = std::make_shared<syt_msgs::srv::FSMControlFlow::Request>();
  request->control_cmd.command = syt_msgs::msg::FSMFlowControlCommand::END;

  syt_msgs::srv::FSMControlFlow::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::FSMControlFlow>("/syt/motion_planner/control_flow", "整机结束", 60000, request, response);
  qDebug() << "整机结束：" << result;
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
  auto request = std::make_shared<syt_msgs::srv::FSMChangeMode::Request>();
  request->mode_cmd.mode = mode;

  syt_msgs::srv::FSMChangeMode::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::FSMChangeMode>("/syt/motion_planner/change_mode", "切换模式", 5000, request, response);
  qDebug() << "切换模式：" << result;
}

void SytRclComm::resetWholeMachine() {
  auto request = std::make_shared<syt_msgs::srv::WholeMachineCMD::Request>();
  request->command.data = request->command.RESET;

  syt_msgs::srv::WholeMachineCMD::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::WholeMachineCMD>("/syt/robot_control/whole_machine/command", "整机流程", 10000, request, response);
  qDebug() << "整机停止：" << result;
}

void SytRclComm::stopWholeMachine() {
  auto request = std::make_shared<syt_msgs::srv::WholeMachineCMD::Request>();
  request->command.data = request->command.STOP;

  syt_msgs::srv::WholeMachineCMD::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::WholeMachineCMD>("/syt/robot_control/whole_machine/command", "整机流程", 10000, request, response);
  qDebug() << "整机停止：" << result;
}

/* -----------------------------固件更新---------------------------- */
// 上料机
void SytRclComm::updateLoadMachine() {
  auto request = std::make_shared<syt_msgs::srv::MCURestart::Request>();
  request->enable = true;

  syt_msgs::srv::MCURestart::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::MCURestart>("/syt/robot_control/load_machine/primal/mcu_restart", "上料机重启单片机", 20000, request, response);
  qDebug() << "上料机重启单片机：" << result;
}

// 合片机
void SytRclComm::updateComposeMachine() {
  auto request = std::make_shared<syt_msgs::srv::MCURestart::Request>();
  request->enable = true;

  syt_msgs::srv::MCURestart::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::MCURestart>("/syt/robot_control/compose_machine/primal/mcu_restart", "合片机重启单片机", 20000, request, response);
  qDebug() << "合片机重启单片机：" << result;
}

// 缝纫机
void SytRclComm::updateSewingMachine() {
  auto request = std::make_shared<syt_msgs::srv::MCURestart::Request>();
  request->enable = true;

  syt_msgs::srv::MCURestart::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::MCURestart>("/syt/robot_control/sewing_machine/primal/mcu_restart", "缝纫机重启单片机", 20000, request, response);
  qDebug() << "缝纫机重启单片机：" << result;
}

/* -----------------------------上料机---------------------------- */
// 上料机复位
void SytRclComm::loadMachineReset(int id) {
  auto request = std::make_shared<syt_msgs::srv::LoadMachineReset::Request>();
  request->id.data = id;
  request->enable = true;

  syt_msgs::srv::LoadMachineReset::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::LoadMachineReset>("/syt/robot_control/load_machine/primal/reset", "上料复位", 20000, request, response);
  qDebug() << "上料复位：" << result;
}

// 补料模式
void SytRclComm::loadMachineAddCloth(int id) {
  auto request = std::make_shared<syt_msgs::srv::LoadMachineAddCloth::Request>();
  request->id.data = id;
  request->enable = true;

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
  auto request = std::make_shared<syt_msgs::srv::LoadMachineClearTable::Request>();
  request->id.data = id;
  request->enable = true;

  syt_msgs::srv::LoadMachineClearTable::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::LoadMachineClearTable>("/syt/robot_control/load_machine/primal/clear_table", "清理台面", 15000, request, response);
  qDebug() << "清理台面：" << result;
}

// 裁片尺寸
void SytRclComm::loadMachineClothSize(int id, uint32_t width, uint32_t length) {
  auto request = std::make_shared<syt_msgs::srv::LoadMachineClothSize::Request>();
  request->id.data = id;
  request->width = width;
  request->length = length;

  syt_msgs::srv::LoadMachineClothSize::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::LoadMachineClothSize>("/syt/robot_control/load_machine/primal/cloth_size", "裁片尺寸", 5000, request, response);
  qDebug() << "裁片尺寸：" << result;
}

// 上料行程
void SytRclComm::loadMachineLoadDistance(int id, uint32_t distance) {
  auto request = std::make_shared<syt_msgs::srv::LoadMachineLoadDistance::Request>();
  request->id.data = id;
  request->distance = distance;

  syt_msgs::srv::LoadMachineLoadDistance::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::LoadMachineLoadDistance>("/syt/robot_control/load_machine/primal/load_distance", "上料行程", 5000, request, response);
  qDebug() << "上料行程：" << result;
}

// 夹爪偏移
void SytRclComm::loadMachineOffset(int id, int offset) {
  auto request = std::make_shared<syt_msgs::srv::LoadMachineOffset::Request>();
  request->id.data = id;
  request->offset = offset;

  syt_msgs::srv::LoadMachineOffset::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::LoadMachineOffset>("/syt/robot_control/load_machine/primal/offset", "上料夹爪偏移", 5000, request, response);
  qDebug() << "上料夹爪偏移：" << result;
}

// 上料间隔
void SytRclComm::loadMachineTrayGap(int id, int32_t height) {
  auto request = std::make_shared<syt_msgs::srv::LoadMachineTrayGap::Request>();
  request->id.data = id;
  request->height = height;

  syt_msgs::srv::LoadMachineTrayGap::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::LoadMachineTrayGap>("/syt/robot_control/load_machine/primal/tray_gap", "上料间隔", 10000, request, response);
  qDebug() << "上料间隔：" << result;
}

// 上料偏移
void SytRclComm::loadMachineTrayOffset(int id, int32_t offset) {
  auto request = std::make_shared<syt_msgs::srv::LoadMachineTrayOffset::Request>();
  request->id.data = id;
  request->offset = offset;

  syt_msgs::srv::LoadMachineTrayOffset::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::LoadMachineTrayOffset>("/syt/robot_control/load_machine/primal/tray_offset", "上料偏移", 5000, request, response);
  qDebug() << "上料偏移：" << result;
}

// 抓住裁片
void SytRclComm::loadMachineHoldCloth(int id) {
  auto request = std::make_shared<syt_msgs::srv::LoadMachineGrabCloth::Request>();
  request->id.data = id;
  request->grab_cloth = 0;

  syt_msgs::srv::LoadMachineGrabCloth::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::LoadMachineGrabCloth>("/syt/robot_control/load_machine/primal/grab_cloth", "抓住裁片", 10000, request, response);
  qDebug() << "抓住裁片：" << result;
}

// 粗对位
void SytRclComm::loadMachineRoughAlign(int id) {
  auto request = std::make_shared<syt_msgs::srv::LoadMachineRoughAlign::Request>();
  request->id.data = id;
  request->enable = true;

  syt_msgs::srv::LoadMachineRoughAlign::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::LoadMachineRoughAlign>("/syt/robot_control/load_machine/primal/rough_align", "粗对位", 10000, request, response);
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
  auto request = std::make_shared<syt_msgs::srv::LoadMachineGrabCloth::Request>();
  request->id.data = id;
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
  auto request = std::make_shared<syt_msgs::srv::LoadMachinePreSetup::Request>();
  request->id.data = id;
  request->enable = true;

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
  auto request = std::make_shared<syt_msgs::srv::LoadMachineLoadCloth::Request>();
  request->id.data = id;

  syt_msgs::srv::LoadMachineLoadCloth::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::LoadMachineLoadCloth>("/syt/robot_control/load_machine/load_cloth", "视觉对位", 10000, request, response);
  qDebug() << "视觉对位：" << result;
}

// 视觉对位
void SytRclComm::loadMachineThickness(int id, float thickness) {
  auto request = std::make_shared<syt_msgs::srv::LoadMachineClothThickness::Request>();
  request->id.data = id;
  request->thickness = thickness;

  syt_msgs::srv::LoadMachineClothThickness::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::LoadMachineClothThickness>("/syt/robot_control/load_machine/primal/thickness", "设置厚度", 5000, request, response);
  qDebug() << "设置厚度：" << result;
}

// 上料针
void SytRclComm::loadMachineExtendNeedle(int id, bool enable) {
  auto request = std::make_shared<syt_msgs::srv::LoadMachineFunction::Request>();
  request->id.data = id;
  request->commands.data = syt_msgs::msg::LoadMachineFunctionCommands::EXTEND_NEEDLE;
  request->enable = enable;

  syt_msgs::srv::LoadMachineFunction::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::LoadMachineFunction>("/syt/robot_control/load_machine/primal/function", "上料针", 5000, request, response);
  qDebug() << "上料针：" << result;
}

// 重量传感器
void SytRclComm::loadMachineWeightSwitch(int id, bool enable) {
  auto request = std::make_shared<syt_msgs::srv::LoadMachineFunction::Request>();
  request->id.data = id;
  request->commands.data = syt_msgs::msg::LoadMachineFunctionCommands::WEIGHT_SWITCH;
  request->enable = enable;

  syt_msgs::srv::LoadMachineFunction::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::LoadMachineFunction>("/syt/robot_control/load_machine/primal/function", "重量传感器", 5000, request, response);
  qDebug() << "重量传感器：" << result;
}

// 上料针
void SytRclComm::loadMachineUnpleatSwitch(int id, bool enable) {
  auto request = std::make_shared<syt_msgs::srv::LoadMachineFunction::Request>();
  request->id.data = id;
  request->commands.data = syt_msgs::msg::LoadMachineFunctionCommands::UNPLEAT_SWITCH;
  request->enable = enable;

  syt_msgs::srv::LoadMachineFunction::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::LoadMachineFunction>("/syt/robot_control/load_machine/primal/function", "不除褶", 5000, request, response);
  qDebug() << "不除褶：" << result;
}

// 上料针
void SytRclComm::loadMachineAgingSwitch(int id, bool enable) {
  auto request = std::make_shared<syt_msgs::srv::LoadMachineFunction::Request>();
  request->id.data = id;
  request->commands.data =
      syt_msgs::msg::LoadMachineFunctionCommands::AGING_SWITCH;
  request->enable = enable;

  syt_msgs::srv::LoadMachineFunction::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::LoadMachineFunction>("/syt/robot_control/load_machine/primal/function", "老化", 5000, request, response);
  qDebug() << "老化：" << result;
}

// 参数设置
syt_msgs::srv::ParamManage::Response SytRclComm::loadMachineParam(int behavior, int dtype, std::string field, std::string data, bool is_array) {
  auto request = std::make_shared<syt_msgs::srv::ParamManage::Request>();
  request->behavior.data = behavior;
  request->dtype.data = dtype;
  request->field = field;
  request->data.resize(data.size());
  if (behavior == 0) {
    for (size_t i = 0; i < data.size(); i++) {
      request->data[i] = (uint8_t)(data[i]);
    }
  }
  request->is_array = is_array;
  // std::cout << "DataLen: " << request->data.size() << std::endl;
  // std::cout << "Data: ";
  // for (auto c : data) {
  // std::cout << (int)c << " ";
  //}
  // std::cout << std::endl;

  syt_msgs::srv::ParamManage::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::ParamManage>("/syt/robot_control/load_machine/primal/param_manage", "上料参数", 5000, request, response);
  qDebug() << "参数管理：" << result;
  return response;
}

/* ---------------------------合片机------------------------------ */
// 合片机复位
void SytRclComm::composeMachineReset() {
  auto request =
      std::make_shared<syt_msgs::srv::ComposeMachineReset::Request>();
  request->enable = true;

  syt_msgs::srv::ComposeMachineReset::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::ComposeMachineReset>("/syt/robot_control/compose_machine/primal/reset", "合片复位", 20000, request, response);
  qDebug() << "合片复位：" << result;
}

// 停止
void SytRclComm::composeMachineStop() {
  auto request = std::make_shared<syt_msgs::srv::ComposeMachineFlow::Request>();
  request->flow_state.data = syt_msgs::msg::ComposeMachineFlowState::STOP;

  syt_msgs::srv::ComposeMachineFlow::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::ComposeMachineFlow>("/syt/robot_control/compose_machine/primal/flow", "合片停止", 5000, request, response);
  qDebug() << "合片停止：" << result;
}

// 除褶
void SytRclComm::composeMachineWipeFold() {
  auto request = std::make_shared<syt_msgs::srv::ComposeMachineFunction::Request>();
  request->commands.wipe_fold = true;

  syt_msgs::srv::ComposeMachineFunction::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::ComposeMachineFunction>("/syt/robot_control/compose_machine/primal/function", "除褶", 10000, request, response);
  qDebug() << "除褶：" << result;
}

// 出针
void SytRclComm::composeMachineExtendNeedle() {
  auto request = std::make_shared<syt_msgs::srv::ComposeMachineFunction::Request>();
  request->commands.extend_sucker_needle = true;

  syt_msgs::srv::ComposeMachineFunction::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::ComposeMachineFunction>("/syt/robot_control/compose_machine/primal/function", "出针", 5000, request, response);
  qDebug() << "出针：" << result;
}

// 收针
void SytRclComm::composeMachineWithdrawNeedle() {
  auto request = std::make_shared<syt_msgs::srv::ComposeMachineFunction::Request>();
  request->commands.extend_sucker_needle = false;

  syt_msgs::srv::ComposeMachineFunction::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::ComposeMachineFunction>("/syt/robot_control/compose_machine/primal/function", "收针", 5000, request, response);
  qDebug() << "收针：" << result;
}

// 吹气
void SytRclComm::composeMachineBlowWind() {
  auto request =
      std::make_shared<syt_msgs::srv::ComposeMachineFunction::Request>();
  request->commands.gripper_fixed_shoulder = true;
  request->commands.gripper_fixed_hem = true;
  request->commands.gripper_fixed_armpit = true;
  request->commands.gripper_spring_shoulder = true;
  request->commands.gripper_spring_hem = true;

  syt_msgs::srv::ComposeMachineFunction::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::ComposeMachineFunction>("/syt/robot_control/compose_machine/primal/function", "吹气", 5000, request, response);
  qDebug() << "吹气：" << result;
}

// 停气
void SytRclComm::composeMachineStopBlow() {
  auto request = std::make_shared<syt_msgs::srv::ComposeMachineFunction::Request>();
  request->commands.gripper_fixed_shoulder = false;
  request->commands.gripper_fixed_hem = false;
  request->commands.gripper_fixed_armpit = false;
  request->commands.gripper_spring_shoulder = false;
  request->commands.gripper_spring_hem = false;

  syt_msgs::srv::ComposeMachineFunction::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::ComposeMachineFunction>("/syt/robot_control/compose_machine/primal/function", "停气", 5000, request, response);
  qDebug() << "停气：" << result;
}

// 开吸风台
void SytRclComm::composeMachineFastenSheet() {
  auto request =
      std::make_shared<syt_msgs::srv::ComposeMachineFunction::Request>();
  request->commands.fasten_sheet = true;

  syt_msgs::srv::ComposeMachineFunction::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::ComposeMachineFunction>("/syt/robot_control/compose_machine/primal/function", "开吸风台", 5000, request, response);
  qDebug() << "开吸风台：" << result;
}

// 关吸风台
void SytRclComm::composeMachineUnfastenSheet() {
  auto request =
      std::make_shared<syt_msgs::srv::ComposeMachineFunction::Request>();
  request->commands.fasten_sheet = false;

  syt_msgs::srv::ComposeMachineFunction::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::ComposeMachineFunction>("/syt/robot_control/compose_machine/primal/function", "关吸风台", 5000, request, response);
  qDebug() << "关吸风台：" << result;
}

// 合片抓手移动
void SytRclComm::composeMachineMoveHand(float x, float y, float z, float c) {
  //// TODO: delete
  // emit signComposeMachineMoveHandFinish(true);
  // return;

  auto request =
      std::make_shared<syt_msgs::srv::ComposeMachineMoveHand::Request>();
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
void SytRclComm::composeMachineMoveSucker(
    syt_msgs::msg::ComposeMachineSuckerStates sucker_states) {
  auto request =
      std::make_shared<syt_msgs::srv::ComposeMachineMoveSucker::Request>();
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

// 平面拟合
void SytRclComm::composeMachineFittingPlane() {
  auto request = std::make_shared<syt_msgs::srv::RunCalibration::Request>();
  request->mode.state = request->mode.FITTING_PLANE;

  syt_msgs::srv::RunCalibration::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::RunCalibration>("/syt/calibration_system/calibration_service", "合片台平面拟合", 0, request, response);
  qDebug() << "合片台拟合平面：" << result;
}

// 吹气高度
void SytRclComm::composeMachineBlowHeight(float height) {
  auto request =
      std::make_shared<syt_msgs::srv::ComposeMachineBlowHeight::Request>();
  request->height = height;

  syt_msgs::srv::ComposeMachineBlowHeight::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::ComposeMachineBlowHeight>("/syt/robot_control/compose_machine/primal/blow_height", "合片吹气高度", 5000, request, response);
}

// 台面灯
void SytRclComm::composeMachineTableLight(float ratio) {
  auto request = std::make_shared<syt_msgs::srv::ComposeMachineTableLight::Request>();
  request->ratio = ratio;

  syt_msgs::srv::ComposeMachineTableLight::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::ComposeMachineTableLight>("/syt/robot_control/compose_machine/primal/table_light", "合片台灯", 5000, request, response);
}

// 参数设置
syt_msgs::srv::ParamManage::Response SytRclComm::composeMachineParam(int behavior, int dtype, std::string field, std::string data, bool is_array) {
  auto request = std::make_shared<syt_msgs::srv::ParamManage::Request>();
  request->behavior.data = behavior;
  request->dtype.data = dtype;
  request->field = field;
  request->data.resize(data.size());
  if (behavior == 0) {
    for (size_t i = 0; i < data.size(); i++) {
      request->data[i] = (uint8_t)(data[i]);
    }
  }
  request->is_array = is_array;

  syt_msgs::srv::ParamManage::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::ParamManage>("/syt/robot_control/compose_machine/primal/param_manage", "合片参数", 5000, request, response);
  qDebug() << "参数管理：" << result;
  return response;
}

/* -----------------------------缝纫机---------------------------- */
// 缝纫机复位
void SytRclComm::sewingMachineReset() {
  auto request = std::make_shared<syt_msgs::srv::SewingMachineReset::Request>();
  request->enable = true;

  syt_msgs::srv::SewingMachineReset::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::SewingMachineReset>("/syt/robot_control/sewing_machine/primal/reset", "缝纫复位", 20000, request, response);
  qDebug() << "缝纫复位：" << result;
  switch (result) {
  case CALL_SUCCESS:
    emit signSewingMachineResetFinish(response.success);
    break;
  case CALL_TIMEOUT:
    emit signSewingMachineResetFinish(false);
  case CALL_INTERRUPT:
    emit signSewingMachineResetFinish(false);
  case CALL_DISCONNECT:
    emit signSewingMachineResetFinish(false);
    break;
  }
}

// 移动抓手
void SytRclComm::sewingMachineMoveHand(float x, float y, float c, bool z) {
  auto request = std::make_shared<syt_msgs::srv::SewingMachineMoveHand::Request>();
  request->target.x = x;
  request->target.y = y;
  request->target.c = c;
  request->target.z = z;

  syt_msgs::srv::SewingMachineMoveHand::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::SewingMachineMoveHand>("/syt/robot_control/sewing_machine/move_hand", "移动缝纫抓手", 20000, request, response);
  qDebug() << "移动缝纫抓手：" << result;
}

// 发送关键点
void SytRclComm::sewingMachineSendKeypoints(syt_msgs::msg::ClothKeypoints2f keypoints) {
  auto request = std::make_shared<syt_msgs::srv::SewingMachineKeypoints::Request>();
  request->keypoints = keypoints;

  syt_msgs::srv::SewingMachineKeypoints::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::SewingMachineKeypoints>("/syt/robot_control/sewing_machine/primal/keypoints", "发送关键点", 5000, request, response);
  qDebug() << "发送关键点：" << result;
}

// 发送针长
void SytRclComm::sewingMachineNeedle(float line_1, float line_2, float line_3,
                                     float line_4) {
  auto request = std::make_shared<syt_msgs::srv::SewingMachineNeedle::Request>();
  request->line_1 = line_1;
  request->line_2 = line_2;
  request->line_3 = line_3;
  request->line_4 = line_4;

  syt_msgs::srv::SewingMachineNeedle::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::SewingMachineNeedle>("/syt/robot_control/sewing_machine/primal/needle", "设置针长", 5000, request, response);
  qDebug() << "设置针长：" << result;
}

// 缝纫厚度
void SytRclComm::sewingMachineThickness(float thickness) {
  auto request = std::make_shared<syt_msgs::srv::SewingMachineClothThickness::Request>();
  request->thickness = thickness;

  syt_msgs::srv::SewingMachineClothThickness::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::SewingMachineClothThickness>("/syt/robot_control/sewing_machine/primal/thickness", "缝纫厚度", 5000, request, response);
  qDebug() << "缝纫厚度：" << result;
}

// 水洗标宽度
void SytRclComm::sewingMachineLabelWidth(bool enable, int side, float width, float position) {
  auto request = std::make_shared<syt_msgs::srv::CareLabelMachineWidth::Request>();
  request->enable = enable;
  request->side.data = side;
  request->width = width;
  request->position = position;

  syt_msgs::srv::CareLabelMachineWidth::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::CareLabelMachineWidth>("/syt/robot_control/sewing_machine/primal/label_width", "水洗标宽度", 5000, request, response);
}

// 水洗标复位
void SytRclComm::sewingMachineLabelReset() {
  auto request = std::make_shared<syt_msgs::srv::CareLabelMachineReset::Request>();
  request->enable = true;

  syt_msgs::srv::CareLabelMachineReset::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::CareLabelMachineReset>("/syt/robot_control/sewing_machine/primal/label_reset", "水洗标重置", 60000, request, response);
}

// 缝纫机运行档位
void SytRclComm::sewingMachineSpeed(int speed) {
  auto request = std::make_shared<syt_msgs::srv::SewingMachineSpeed::Request>();
  request->gear.data = speed;

  syt_msgs::srv::SewingMachineSpeed::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::SewingMachineSpeed>("/syt/robot_control/sewing_machine/primal/speed", "缝纫机档位", 5000, request, response);
}

// 转换运行模式
void SytRclComm::sewingMachineMode(int mode) {
  auto request = std::make_shared<syt_msgs::srv::SewingMachineMode::Request>();
  request->mode.data = mode;

  syt_msgs::srv::SewingMachineMode::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::SewingMachineMode>("/syt/robot_control/sewing_machine/primal/mode", "缝纫机模式", 5000, request, response);
}

// 参数设置
syt_msgs::srv::ParamManage::Response SytRclComm::sewingMachineParam(int behavior, int dtype, std::string field, std::string data, bool is_array) {
  auto request = std::make_shared<syt_msgs::srv::ParamManage::Request>();
  request->behavior.data = behavior;
  request->dtype.data = dtype;
  request->field = field;
  request->data.resize(data.size());
  if (behavior == 0) {
    for (size_t i = 0; i < data.size(); i++) {
      request->data[i] = (uint8_t)(data[i]);
    }
  }
  request->is_array = is_array;

  syt_msgs::srv::ParamManage::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::ParamManage>("/syt/robot_control/sewing_machine/primal/param_manage", "缝纫参数", 5000, request, response);
  qDebug() << "参数管理：" << result;
  return response;
}

/* ---------------------------视觉检测------------------------------ */
// 获取衣服信息
void SytRclComm::getClothInfo(uint8_t frame_id, int cloth_type) {
  //// TODO: delete
  // emit signGetClothInfoFinish(true, cloth_type, syt_msgs::msg::ClothInfo());
  // return;

  auto request = std::make_shared<syt_msgs::srv::GetClothInfo::Request>();
  request->frame_id.data = frame_id; // 0 为相机系 1 为合片机 2 为缝纫机
  request->compare_flag = 0;

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

// 获取关键点
void SytRclComm::checkCalibration() {
  auto request = std::make_shared<syt_msgs::srv::GetClothKeypointInfo::Request>();
  request->frame_id.data = 0; // 0 为相机系 1 为合片机 2 为缝纫机
  request->cloth_num = 0;

  syt_msgs::srv::GetClothKeypointInfo::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::GetClothKeypointInfo>("/syt/clothes_detector/get_camera_cloth_keypoint", "检测标定结果", 8000, request, response);
  qDebug() << "检测标定结果：" << result;

  // result           = CALL_SUCCESS;
  // response.success = true;

  switch (result) {
  case CALL_SUCCESS:
    if (response.success) {
      // response.info.left_bottom.x  = 0;
      // response.info.left_bottom.y  = 0;
      // response.info.right_bottom.x = 3;
      // response.info.right_bottom.y = -4;
      // response.info.right_oxter.x  = 3;
      // response.info.right_oxter.y  = 4;

      auto getDistance = [=](syt_msgs::msg::Vector3 p1, syt_msgs::msg::Vector3 p2) -> float {
        return qSqrt(qPow(p1.x - p2.x, 2) + qPow(p1.y - p2.y, 2));
      };
      float bottom_length = getDistance(response.info.left_bottom, response.info.right_bottom);
      float side_length = getDistance(response.info.right_bottom, response.info.right_oxter);

      emit signCheckCalibrationFinish(true, bottom_length, side_length);
    } else {
      emit signCheckCalibrationFinish(false, 0, 0);
    }
    break;
  case CALL_TIMEOUT:
  case CALL_INTERRUPT:
  case CALL_DISCONNECT:
    emit signCheckCalibrationFinish(false, 0, 0);
    break;
  }
}

void SytRclComm::updateParam(std::string node, int type, QString field, QString value, bool is_array) {
  rcl_interfaces::msg::Parameter param;
  param.name = field.toStdString();
  switch (type) {
  case 0:
    param.value.integer_value = value.toInt();
    param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    break;
  case 1:
    param.value.integer_value = value.toUInt();
    param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    break;
  case 2:
    param.value.integer_value = value.toLongLong();
    param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    break;
  case 3:
    param.value.integer_value = value.toULongLong();
    param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    break;
  case 4:
    param.value.double_value = value.toFloat();
    param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    break;
  case 5:
    param.value.double_value = value.toDouble();
    param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    break;
  case 6:
    param.value.integer_value = (char)value.at(0).toLatin1();
    param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    break;
  case 7:
    param.value.integer_value = value.toUInt();
    param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    break;
  case 8:
    param.value.string_value = value.toStdString();
    param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    break;
  }

  auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  request->parameters.emplace_back(param);

  rcl_interfaces::srv::SetParameters::Response response;
  CALL_RESULT result = callService<rcl_interfaces::srv::SetParameters>("/" + node + "/set_parameters", "上位机参数设置", 5000, request, response);
}

std::vector<uchar> SytRclComm::readParam(std::string node, int type, QString field, bool is_array) {
  auto title_ = field.toStdString();
  auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  std::string value;
  std::vector<int64_t> a_value;
  std::vector<double> d_value;
  std::vector<std::string> s_value;
  std::vector<uchar> u_value;

  request->names.emplace_back(title_);
  rcl_interfaces::srv::GetParameters::Response response;
  CALL_RESULT result = callService<rcl_interfaces::srv::GetParameters>("/" + node + "/get_parameters", "rosparam参数读取",  5000, request, response);
  if(!result) {
    if(!is_array) {
      switch(type) {
      case 0:
        value = std::to_string(response.values[0].integer_value);
        for(char c : value){
          u_value.push_back(static_cast<unsigned char>(c));
        }
        break;
      case 1:
        value = std::to_string(response.values[0].integer_value);
        for(char c : value){
          u_value.push_back(static_cast<unsigned char>(c));
        }
        break;
      case 2:
        value = std::to_string(response.values[0].integer_value);
        for(char c : value){
          u_value.push_back(static_cast<unsigned char>(c));
        }
        break;
      case 3:
        value = std::to_string(response.values[0].integer_value);
        for(char c : value){
          u_value.push_back(static_cast<unsigned char>(c));
        }
        break;
      case 4:
        value = std::to_string(response.values[0].double_value);
        for(char c : value){
          u_value.push_back(static_cast<unsigned char>(c));
        }
        break;
      case 5:
        value = std::to_string(response.values[0].double_value);
        for(char c : value){
          u_value.push_back(static_cast<unsigned char>(c));
        }
        break;
      case 6:
        value = std::to_string(response.values[0].integer_value);
        for(char c : value){
          u_value.push_back(static_cast<unsigned char>(c));
        }
        break;
      case 7:
        value = std::to_string(response.values[0].integer_value);
        for(char c : value){
          u_value.push_back(static_cast<unsigned char>(c));
        }
        break;
      case 8:
        value = response.values[0].string_value;
        for(char c : value){
          u_value.push_back(static_cast<unsigned char>(c));
        }
        break;
      }
    }else {
      switch(type){
        case 0:
        a_value = response.values[0].integer_array_value;
        for(int i = 0; i < a_value.size(); i++){
          std::string str = std::to_string(a_value[i]);
          for(char c : str){
            u_value.push_back(static_cast<unsigned char>(c));
          }
        }
        break;
        case 1:
        a_value = response.values[0].integer_array_value;
        for(int i = 0; i < a_value.size(); i++){
          std::string str = std::to_string(a_value[i]);
          for(char c : str){
            u_value.push_back(static_cast<unsigned char>(c));
          }
        }
        break;
        case 2:
        a_value = response.values[0].integer_array_value;
        for(int i = 0; i < a_value.size(); i++){
          std::string str = std::to_string(a_value[i]);
          for(char c : str){
            u_value.push_back(static_cast<unsigned char>(c));
          }
        }
        break;
        case 3:
        a_value = response.values[0].integer_array_value;
        for(int i = 0; i < a_value.size(); i++){
          std::string str = std::to_string(a_value[i]);
          for(char c : str){
            u_value.push_back(static_cast<unsigned char>(c));
          }
        }
        break;
        case 4:
        d_value = response.values[0].double_array_value;
        for(int i = 0; i < d_value.size(); i++){
          std::string str = std::to_string(d_value[i]);
          for(char c : str){
            u_value.push_back(static_cast<unsigned char>(c));
          }
        }
        break;
        case 5:
        d_value = response.values[0].double_array_value;
        for(int i = 0; i < d_value.size(); i++){
          std::string str = std::to_string(d_value[i]);
          for(char c : str){
            u_value.push_back(static_cast<unsigned char>(c));
          }
        }
        break;
        case 6:
        a_value = response.values[0].integer_array_value;
        for(int i = 0; i < a_value.size(); i++){
          std::string str = std::to_string(a_value[i]);
          for(char c : str){
            u_value.push_back(static_cast<unsigned char>(c));
          }
        }
        case 7:
        a_value = response.values[0].integer_array_value;
        for(int i = 0; i < a_value.size(); i++){
          std::string str = std::to_string(a_value[i]);
          for(char c : str){
            u_value.push_back(static_cast<unsigned char>(c));
          }
        }
        case 8:
        s_value = response.values[0].string_array_value;
        for(int i = 0; i < s_value.size(); i++){
          for(char c : s_value[i]){
            u_value.push_back(static_cast<unsigned char>(c));
          }
        }
      }
    }
  }
  return u_value;
}

/* ---------------------------样式相关------------------------------ */
// 创建样式文件
void SytRclComm::createStyle(int mode, QString prefix, syt_msgs::msg::ClothStyle cloth_style_front, syt_msgs::msg::ClothStyle cloth_style_back) {
  //// TODO
  // emit signCreateStyleFinish(true, QString("test"));
  // return;

  auto request = std::make_shared<syt_msgs::srv::CreateStyle::Request>();
  request->mode.data = mode;
  request->prefix = prefix.toStdString();
  request->file_name = "";
  request->cloth_style_front = cloth_style_front;
  request->cloth_style_back = cloth_style_back;

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
  auto request = std::make_shared<syt_msgs::srv::RenameClothStyle::Request>();
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
  auto request = std::make_shared<syt_msgs::srv::SetCurrentClothStyle::Request>();
  request->prefix = prefix.toStdString();
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
  auto request = std::make_shared<syt_msgs::srv::GetClothStyle::Request>();
  request->prefix = prefix.toStdString();
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

// 急停
void SytRclComm::emergencyStop() {
  start_flag_ = true;
  auto request = std::make_shared<syt_msgs::srv::FSMControlFlow::Request>();
  request->control_cmd.command = syt_msgs::msg::FSMFlowControlCommand::STOP;

  syt_msgs::srv::FSMControlFlow::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::FSMControlFlow>("/syt/motion_planner/control_flow", "整机急停", 5000, request, response);
  qDebug() << "整机急停：" << result;
}

// 红灯
void SytRclComm::redLight() {
  auto request = std::make_shared<syt_msgs::srv::WarningLight::Request>();
  request->color.data = syt_msgs::msg::WarningLightColor::RED;

  syt_msgs::srv::WarningLight::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::WarningLight>("/syt/robot_control/compose_machine/primal/warning_light", "亮红灯", 5000, request, response);
  qDebug() << "亮红灯：" << result;
}

// 绿灯
void SytRclComm::greenLight() {
  auto request = std::make_shared<syt_msgs::srv::WarningLight::Request>();
  request->color.data = syt_msgs::msg::WarningLightColor::GREEN;

  syt_msgs::srv::WarningLight::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::WarningLight>("/syt/robot_control/compose_machine/primal/warning_light", "亮绿灯", 5000, request, response);
  qDebug() << "亮绿灯：" << result;
}

// 黄灯
void SytRclComm::yellowLight() {
  auto request = std::make_shared<syt_msgs::srv::WarningLight::Request>();
  request->color.data = syt_msgs::msg::WarningLightColor::YELLOW;

  syt_msgs::srv::WarningLight::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::WarningLight>("/syt/robot_control/compose_machine/primal/warning_light", "亮黄灯", 5000, request, response);
  qDebug() << "亮黄灯：" << result;
}

// 蜂鸣器开
void SytRclComm::bellOpen() {
  auto request = std::make_shared<syt_msgs::srv::WarningLight::Request>();
  request->color.data = syt_msgs::msg::WarningLightColor::NONE;
  request->bell = true;

  syt_msgs::srv::WarningLight::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::WarningLight>("/syt/robot_control/compose_machine/primal/warning_light", "蜂鸣器", 5000, request, response);
  qDebug() << "蜂鸣器开：" << result;
}

// 蜂鸣器关
void SytRclComm::bellClose() {
  auto request = std::make_shared<syt_msgs::srv::WarningLight::Request>();
  request->color.data = syt_msgs::msg::WarningLightColor::NONE;
  request->bell = false;

  syt_msgs::srv::WarningLight::Response response;
  CALL_RESULT result = callService<syt_msgs::srv::WarningLight>("/syt/robot_control/compose_machine/primal/warning_light", "蜂鸣器", 5000, request, response);
  qDebug() << "蜂鸣器关：" << result;
}

bool SytRclComm::setNodeParam(const std::string &node_name, const std::string &param_name, const std::string &param_value) {
  if (param_set_handles_.find(node_name) == param_set_handles_.end()) {
    // 如果没找到对应的设置参数句柄(client)，则需要创建一下
    std::string service_name = "/" + node_name + "/set_parameters";
    param_set_handles_[node_name] = node_->create_client<rcl_interfaces::srv::SetParameters>(service_name);
  }
  if (param_get_handles_.find(node_name) == param_get_handles_.end()) {
    // 如果没找到对应的读取参数句柄(client)，则需要创建一下
    // 通常来说写入就意味着要读取
    std::string service_name = "/" + node_name + "/get_parameters";
    param_get_handles_[node_name] = node_->create_client<rcl_interfaces::srv::GetParameters>(service_name);
  }
  return true;
}
