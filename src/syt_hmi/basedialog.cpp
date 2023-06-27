//
// Created by jerry on 23-6-27.
//

#include "syt_hmi/basedialog.h"

BaseDialog::BaseDialog(QWidget *parent) : QDialog(parent) {
    setMouseTracking(true); // 鼠标没有按下时也能捕获移动事件
    // 隐藏默认标题栏
    this->setWindowFlags(Qt::FramelessWindowHint | Qt::Dialog);
}
