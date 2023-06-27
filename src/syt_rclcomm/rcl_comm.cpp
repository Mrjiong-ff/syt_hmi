//
// Created by jerry on 23-6-21.
//

#include <memory>
#include "syt_rclcomm/rcl_comm.h"


SytRclComm::SytRclComm() {
    m_executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    m_node = rclcpp::Node::make_shared("syt_hmi_node");
    m_executor->add_node(m_node);
    this->start();
    qDebug("init syt hmi node successful!");
}

SytRclComm::~SytRclComm() {
    process_->terminate();
    process_->waitForFinished();
    delete process_;
    // todo 杀死所有ros2相关的进程  目前已知为 _ros2_daemon  ros2，感觉有问题，但暂时能解决进程未杀死的问题
    std::string command = "pkill ros2\n";
    int result = std::system(command.c_str());
    if (result != 0) {
        emit errorNodeMsgSign("Bug: 进程杀死错误,请检查这部分代码.");
    }
    rclcpp::shutdown();
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
}
