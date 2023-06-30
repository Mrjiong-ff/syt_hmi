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
                                                                               std::bind(&SytRclComm::download_callback,
                                                                                         this, std::placeholders::_1));

    // todo 用于回调视觉显示
    callback_group_vision =
            m_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_vision_obt = rclcpp::SubscriptionOptions();
    sub_vision_obt.callback_group = callback_group_vision;

    this->start();
    qDebug("init syt hmi node successful!");
}

SytRclComm::~SytRclComm() {
    process_->terminate();
    process_->waitForFinished();
    delete process_;

    rclcpp::shutdown();
    // todo 杀死所有ros2相关的进程  目前已知为 _ros2_daemon  ros2，感觉有问题，但暂时能解决进程未杀死的问题
    std::string command = "pkill ros2\n";
    int result = std::system(command.c_str());
//    command = "pkill _ros2_daemon\n";
//    result = std::system(command.c_str());
    if (result != 0) {
        emit errorNodeMsgSign("Bug: 进程杀死错误,请检查这部分代码.");
    }
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
    std::string command = "cd /home/jerry/Documents/syt_ros2_ws && source install/setup.zsh && ros2 launch syt_template 456.launch.py\n";
    process_ = new QProcess(this);
    process_->start("zsh\n");
    process_->waitForStarted();
    process_->write(command.c_str());
    process_->waitForReadyRead(1000);
    auto error_msg = process_->readAllStandardError().toStdString();
    if (error_msg.empty()) {
        return true;
    }
    QString msg = QString("Fatal: 节点初始化失败.\n错误消息: %1").arg(error_msg.data());
    emit errorNodeMsgSign(msg);
    return false;
//    return true;
}

void SytRclComm::load_cloth_visable(bool f) {
    // todo 向syt cloth edge server 发送允许传输图像指令
    qDebug("允许传输上料的图像");

}

void SytRclComm::otaUpdate() {
    total_size = 0;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client = m_node->create_client<std_srvs::srv::SetBool>(
            "/syt/ota/update");

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
