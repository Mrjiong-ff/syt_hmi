//
// Created by jerry on 23-4-28.
//

#ifndef SYT_HMI_UTILS_H
#define SYT_HMI_UTILS_H

#include <QWidget>
#include <QMessageBox>
#include <QVariant>
#include <sys/time.h>

int showMessageBox(QWidget *p, QString text, int btn_num, QVector<QString> btn_text);

unsigned long getTickCount();

/**
 * 防止多次连续点击btn
 * @param target
 * @param delayTimeMil
 * @return
 */
bool isFastClick(QObject *target, int delayTimeMil);

#endif //SYT_HMI_UTILS_H
