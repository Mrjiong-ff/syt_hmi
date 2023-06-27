//
// Created by jerry on 23-6-21.
//

#ifndef SYT_HMI_RCL_COMM_H
#define SYT_HMI_RCL_COMM_H

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <QThread>
#include <QWidget>
#include <QProcess>
#include "utils/utils.h"
//#include "syt_msgs/msg/fsm_flow_control_command.hpp"
//#include "syt_msgs/msg/fsm_run_mode.hpp"
//#include <rcl_interfaces/msg/log.hpp>

/**
 * 所有与ros2打交道的，在这里实现
 */


class SytRclComm : public QThread {
Q_OBJECT
public:

    SytRclComm();

    ~SytRclComm() override;

    bool initAllNodes();


protected:
    void run() override;

signals:

    void errorNodeMsgSign(QString msg);

private:

    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> m_executor;

    std::shared_ptr<rclcpp::Node> m_node;

    QProcess *process_;

};


#endif //SYT_HMI_RCL_COMM_H
