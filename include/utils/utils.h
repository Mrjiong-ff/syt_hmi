#pragma once
#include <QIcon>
#include <QMessageBox>
#include <QVariant>
#include <QWidget>
#include <csignal>
#include <fstream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

enum STATE {
  SUCCESS = 0,
  WARN    = 1,
  ERROR
};

extern QMap<QString, int> style_id_map;
extern QMap<int, QString> id_style_map;

/**
 * 一些消息弹窗
 * @param p
 * @param text
 * @param btn_num
 * @param btn_text
 * @return
 */
int showMessageBox(QWidget *p, STATE state, QString text, int btn_num, QVector<QString> btn_text);

/**
 * 获取当前系统时间
 * @return
 */
unsigned long getTickCount();

/**
 * 防止多次连续点击btn
 * @param target
 * @param delayTimeMil
 * @return
 */
bool isFastClick(QObject *target, int delayTimeMil);

/**
 * cvmat 转 用于qt显示的qimage类型
 * @param mat
 * @return
 */
QImage cvMat2QImage(const cv::Mat &mat);

/**
 * qimage 转 cvmat 类型
 * @param image
 * @return
 */
cv::Mat QImage2cvMat(QImage image);

/**
 * 检测是否存在配置文件，没有则自动生成
 */
void checkConfigsExist();

/**
 * 获取配置文件路径
 * @return
 */
std::string getConfigPath();

/**
 * 得到当前时间 精确到 年月日时分秒
 * @return
 */
std::string getCurrentTime();

/**
 * 杀死进程
 * @return
 */
void killProcesses(std::string process_pattern);
