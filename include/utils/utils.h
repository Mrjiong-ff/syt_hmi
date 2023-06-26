//
// Created by jerry on 23-4-28.
//

#ifndef SYT_HMI_UTILS_H
#define SYT_HMI_UTILS_H

#include <QWidget>
#include <QMessageBox>
#include <QVariant>
#include <sys/time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

/**
 * 一些消息弹窗
 * @param p
 * @param text
 * @param btn_num
 * @param btn_text
 * @return
 */
int showMessageBox(QWidget *p, QString text, int btn_num, QVector<QString> btn_text);

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


#endif //SYT_HMI_UTILS_H
