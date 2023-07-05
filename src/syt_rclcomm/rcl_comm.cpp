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
    // todo upload cloth vision sub   node name not set
    load_cloth_visual_subscription_ = m_node->create_subscription<std_msgs::msg::Int32>(
            "/syt/cloth_edge/cloth_edge_visual_topic", 10,
            std::bind(
                    &SytRclComm::loadClothVisualCallback,
                    this,
                    std::placeholders::_1));


//    m_executor.get
    // todo 用于回调视觉显示
    callback_group_vision =
            m_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_vision_obt = rclcpp::SubscriptionOptions();
    sub_vision_obt.callback_group = callback_group_vision;

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

void SytRclComm::load_cloth_visable(bool f) {
    // todo 向syt cloth edge server 发送允许传输图像指令
    qDebug("允许传输上料的图像");

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
        RCLCPP_INFO(rclcpp::get_logger("SytHmi"), "service not available, waiting again...");
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
    std::cerr << "recv machine id: " << machine_id << " cam id: " << cam_id << std::endl;
    auto img_h = msg.get()->image.height;
    auto img_w = msg.get()->image.width;
    auto encoding = msg.get()->image.encoding;
    auto img_data = msg.get()->image.data;

    QImage image(img_data.data(), img_w, img_h, QImage::Format_RGB888);

    emit visualLoadClothRes(machine_id, cam_id, image);
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

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;
    auto result = client->async_send_request(request);
    auto success = result.get()->success;
    auto msg = result.get()->message;
    emit installRes(success, QString(msg.data()));
}

void SytRclComm::compCalib() {
    qDebug("发布 合片台标定 话题");
    auto pub = m_node->create_publisher<syt_msgs::msg::CalibState>("/syt/calibration_system/calibrator", 10);
    syt_msgs::msg::CalibState calib_state;
    calib_state.state = 3;
    pub->publish(calib_state);
}

void SytRclComm::sewingCalib() {
    qDebug("发布 缝纫台标定 话题");
    auto pub = m_node->create_publisher<syt_msgs::msg::CalibState>("/syt/calibration_system/calibrator", 10);
    syt_msgs::msg::CalibState calib_state;
    calib_state.state = 2;
    pub->publish(calib_state);
}
