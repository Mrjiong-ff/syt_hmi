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
#include <unistd.h>
#include "syt_msgs/srv/get_break_point_y.hpp"
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/int32.hpp>

/**
 * 所有与ros2打交道的，在这里实现
 */


class SytRclComm : public QThread {
Q_OBJECT
public:

    SytRclComm();

    ~SytRclComm() override;

    bool initAllNodes();

    void otaUpdate();

    void otaDownload();

    void load_cloth_visable(bool f);



private:
    void download_callback(const std_msgs::msg::Int32::SharedPtr msg);

    void killProcesses(std::string);

protected:
    void run() override;

signals:

    void errorNodeMsgSign(QString msg);

    void waitUpdateResultSuccess(bool res, QString msg);

    void updateProcess(int, int);

    void processZero();

    void downloadRes(bool,QString);

private:
    // total
    int total_size = 0;

    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> m_executor;

    std::shared_ptr<rclcpp::Node> m_node;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr download_subscription_;

    QProcess *process_;

    rclcpp::CallbackGroup::SharedPtr callback_group_vision;

};


#endif //SYT_HMI_RCL_COMM_H
