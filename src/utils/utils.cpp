//
// Created by jerry on 23-4-28.
//

#include "utils/utils.h"


int showMessageBox(QWidget *p, QString text, int btn_num, QVector<QString> btn_text) {
    auto box = new QMessageBox(p);
    box->setWindowFlags(Qt::FramelessWindowHint | Qt::Dialog);
    box->setText(text);
    for (int i = 0; i < btn_num; ++i) {
        box->setButtonText(i + 1, btn_text[i]);
    }
    box->show();
    return box->exec();
}


unsigned long getTickCount() {
    struct timeval tv;
    if (gettimeofday(&tv, NULL) != 0)
        return 0;
    return (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
}


bool isFastClick(QObject *target, int delayTimeMil) {
    qlonglong lastTick = (target->property("tick").toLongLong());
    qlonglong tick = getTickCount();
    target->setProperty("tick", tick);
    if (tick - lastTick > delayTimeMil) {
        return true;
    }
    return false;
}
