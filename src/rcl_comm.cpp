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
    qDebug("syt_hmi节点启动成功");
}

SytRclComm::~SytRclComm() {
    rclcpp::shutdown();
}

void SytRclComm::run() {
    rclcpp::WallRate rate(50);

    while (rclcpp::ok()) {
        m_executor->spin_some();
        rate.sleep();
    }
    rclcpp::shutdown();
}
