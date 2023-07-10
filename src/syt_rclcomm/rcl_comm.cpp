//
// Created by jerry on 23-6-21.
//

#include <memory>
#include "syt_rclcomm/rcl_comm.h"


SytRclComm::SytRclComm() {
    m_executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    m_node = rclcpp::Node::make_shared("syt_hmi_node");
    m_executor->add_node(m_node);

    // ota sub
    download_subscription_ = m_node->create_subscription<std_msgs::msg::Int32>("/syt/ota/ftp_topic", 10,
                                                                               std::bind(
                                                                                       &SytRclComm::download_callback,
                                                                                       this,
                                                                                       std::placeholders::_1));

    // 上料机视觉显示回调
    load_cloth_visual_subscription_ = m_node->create_subscription<syt_msgs::msg::LoadClothVisual>(
            "/syt/cloth_edge_pydetect/cloth_edge_visual_topic", 10,
            std::bind(
                    &SytRclComm::loadClothVisualCallback,
                    this,
                    std::placeholders::_1));

    // todo 合片机视觉显示回调
    composer_visual_subscription_ = m_node->create_subscription<syt_msgs::msg::LoadClothVisual>(
            "/syt/comp/comp_visual_topic", 10,
            std::bind(
                    &SytRclComm::loadClothVisualCallback,
                    this,
                    std::placeholders::_1));

    // todo rosout消息回调
    log_subscription = m_node->create_subscription<rcl_interfaces::msg::Log>("/rosout", 10,
                                                                             std::bind(&SytRclComm::logCallback, this,
                                                                                       std::placeholders::_1));


    // todo 开始停止复位
    fsm_flow_control_cmd_publisher = m_node->create_publisher<syt_msgs::msg::FSMFlowControlCommand>(
            "/syt/robot_control/flow_control_cmd", 10);

    fsm_run_mode_publisher = m_node->create_publisher<syt_msgs::msg::FSMRunMode>(
            "/syt/robot_control/running_mode", 10);


//    m_executor.get
    // todo 用于回调视觉显示
//    callback_group_vision =
//            m_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
//    auto sub_vision_obt = rclcpp::SubscriptionOptions();
//    sub_vision_obt.callback_group = callback_group_vision;

    this->start();
    qDebug("init syt hmi node successful!");
}

SytRclComm::~SytRclComm() {
    if (process_ != nullptr) {
        process_->terminate();
        process_->waitForFinished();
        delete process_;
    }


    rclcpp::shutdown();

    killProcesses("syt");

    qDebug("shut down rclcomm.");
}

void SytRclComm::run() {
    rclcpp::WallRate rate(50);

    while (rclcpp::ok()) {
        m_executor->spin_some();
        rate.sleep();
    }
    rclcpp::shutdown();
}

bool SytRclComm::initAllNodes() {
    // todo 一键启动launch的命令 test
//    std::string command = "cd /home/jerry/Documents/syt_ros2_ws && source install/setup.zsh && ros2 launch syt_template 456.launch.py\n";
//    process_ = new QProcess(this);
//    process_->start("zsh\n");
//    process_->waitForStarted();
//    process_->write(command.c_str());
//    process_->waitForReadyRead(1000);
//    auto error_msg = process_->readAllStandardError().toStdString();
//    if (error_msg.empty()) {
//        return true;
//    }
//    QString msg = QString("Fatal: 节点初始化失败.\n错误消息: %1").arg(error_msg.data());
//    emit errorNodeMsgSign(msg);
//    return false;
    return true;
}

void SytRclComm::otaUpdate() {
    total_size = 0;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client = m_node->create_client<std_srvs::srv::SetBool>(
            "/syt/ota/update");

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("SytHmi"),
                         "Interrupted while waiting for the /syt/ota/update service. Exiting.");
        }
        RCLCPP_INFO(rclcpp::get_logger("SytHmi"), "/syt/ota/update not available, waiting again...");
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();

    request->data = true;

    auto result = client->async_send_request(request);

    auto success = result.get()->success;
    auto msg = result.get()->message;
    std::cout << "是否成功: " << success << std::endl;
    if (success) {
        total_size = std::stoi(msg);
        std::cout << "更新包总大小: " << total_size << std::endl;
    }
    emit waitUpdateResultSuccess(success, QString(msg.data()));
}

void SytRclComm::otaDownload() {
    emit processZero();
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client = m_node->create_client<std_srvs::srv::SetBool>(
            "/syt/ota/download");
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;

    auto result = client->async_send_request(request);

    bool success = result.get()->success;

    std::cout << "==========================" << std::endl;
    std::cout << "是否下载成功::" << success << result.get()->message << std::endl;
    std::cout << "==========================" << std::endl;

    emit downloadRes(success, QString(result.get()->message.data()));

}

void SytRclComm::download_callback(const std_msgs::msg::Int32::SharedPtr msg) {
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


void SytRclComm::killProcesses(std::string processPattern) {
    std::vector<int> processIds;

    std::string pgrepCommand = "pgrep -f ";
    pgrepCommand += processPattern;

    FILE *pipe = popen(pgrepCommand.c_str(), "r");
    if (pipe) {
        char buffer[128];
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            int pid = atoi(buffer);
            if (pid != 0) {
                processIds.push_back(pid);
            }
        }

        pclose(pipe);
    }
    std::cerr << "process size: " << processIds.size() << std::endl;

    // 杀死符合模式但排除特定进程的进程
    for (int pid: processIds) {
        if (pid != getpid()) {  // 排除当前进程
            kill(pid, SIGTERM);
        }
    }
}

void SytRclComm::otaInstall() {
    qDebug("install app");
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client = m_node->create_client<std_srvs::srv::SetBool>(
            "/syt/ota/install");
    // todo 除了 hmi 和 ota之外的进程全部关闭
//    killProcesses()

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;
    auto result = client->async_send_request(request);
    auto success = result.get()->success;
    auto msg = result.get()->message;
    emit installRes(success, QString(msg.data()));
}

void SytRclComm::compCalib() {
    qDebug("call 合片台标定");
    rclcpp::Client<syt_msgs::srv::RunCalibration>::SharedPtr client = m_node->create_client<syt_msgs::srv::RunCalibration>(
            "/syt/calibration_system/calibration_service");

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("SytHmi"),
                         "Interrupted while waiting for the /syt/calibration_system/calibration_service service. Exiting.");
        }
        RCLCPP_INFO(rclcpp::get_logger("SytHmi"),
                    "/syt/calibration_system/calibration_service not available, waiting again...");
    }

    auto request = std::make_shared<syt_msgs::srv::RunCalibration::Request>();
    request->mode.state = 3;
    auto result = client->async_send_request(request);
    auto success = result.get()->success;
    std::cout << "合片台标定完成,结果: " << success << std::endl;

    emit compCalibRes(success);
}

void SytRclComm::sewingCalib() {
    qDebug("call 缝纫台标定");

    rclcpp::Client<syt_msgs::srv::RunCalibration>::SharedPtr client = m_node->create_client<syt_msgs::srv::RunCalibration>(
            "/syt/calibration_system/calibration_service");

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("SytHmi"),
                         "Interrupted while waiting for the /syt/calibration_system/calibration_service service. Exiting.");
        }
        RCLCPP_INFO(rclcpp::get_logger("SytHmi"),
                    "/syt/calibration_system/calibration_service not available, waiting again...");
    }

    auto request = std::make_shared<syt_msgs::srv::RunCalibration::Request>();
    request->mode.state = 2;
    auto result = client->async_send_request(request);
    auto success = result.get()->success;

    std::cout << "缝纫台标定完成,结果: " << success << std::endl;

    emit sewingCalibRes(success);
}

void SytRclComm::logCallback(const rcl_interfaces::msg::Log::SharedPtr msg) {
    auto cur_time = getCurrentTime();
    // 枚举详情信息看官方
    auto level_bt = msg->level;
    QString level;
    switch (level_bt) {
        case 10:
            level = "DEBUG";
            break;
        case 20:
            level = "INFO";
            break;
        case 30:
            level = "WARN";
            break;
        case 40:
            level = "ERROR";
            break;
        case 50:
            level = "FATAL";
            break;
    }
    auto _msg = msg->msg.data();
    auto func = msg->function.data();
    auto location = msg->name.data();
    emit signLogPub(QString(cur_time.c_str()), level, QString(location),
                    QString(func), QString(_msg));
}

void SytRclComm::startCmd() {
    auto message = syt_msgs::msg::FSMFlowControlCommand();
    message.command = 1;
    message.state.state_code = 0;
    fsm_flow_control_cmd_publisher->publish(message);
}

void SytRclComm::resetCmd() {
    auto message = syt_msgs::msg::FSMFlowControlCommand();
    message.command = 2;
    message.state.state_code = 0;
    fsm_flow_control_cmd_publisher->publish(message);
}

void SytRclComm::stopCmd() {
    // todo
}
