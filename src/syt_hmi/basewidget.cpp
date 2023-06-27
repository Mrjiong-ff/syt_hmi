//
// Created by jerry on 23-6-27.
//

#include "syt_hmi/basewidget.h"

BaseWidget::BaseWidget(QWidget *parent) : QWidget(parent) {
    setMouseTracking(true); // 鼠标没有按下时也能捕获移动事件
    // 隐藏默认标题栏
    this->setWindowFlags(Qt::FramelessWindowHint);
}

void BaseWidget::mousePressEvent(QMouseEvent *event) {
    switch (event->button()) {
        case Qt::LeftButton:
            is_mouse_left_press_down_ = true;

            if (dir_ != NONE_) {
                this->mouseGrabber(); //返回当前抓取鼠标输入的窗口
            } else {
                m_mousePos_ = event->globalPos() - this->frameGeometry().topLeft();
            }
            break;
        case Qt::RightButton:
//            this->setWindowState(Qt::WindowMinimized);
            break;
        default:
            return;
    }
}

void BaseWidget::mouseMoveEvent(QMouseEvent *event) {
    QPoint globalPoint = event->globalPos();   //鼠标全局坐标
    QRect rect = this->rect();  //rect == QRect(0,0 1280x720)
    QPoint topLeft = mapToGlobal(rect.topLeft());
    QPoint bottomRight = mapToGlobal(rect.bottomRight());

    if (this->windowState() != Qt::WindowMaximized) {
        if (!is_mouse_left_press_down_)  //没有按下左键时
        {
            this->region(globalPoint); //窗口大小的改变——判断鼠标位置，改变光标形状
        } else {
            if (dir_ != NONE_) {
                QRect newRect(topLeft, bottomRight); //定义一个矩形  拖动后最大1000*1618

                switch (dir_) {
                    case LEFT_:

                        if (bottomRight.x() - globalPoint.x() <= this->minimumWidth()) {
                            newRect.setLeft(topLeft.x());  //小于界面的最小宽度时，设置为左上角横坐标为窗口x
                            //只改变左边界
                        } else {
                            newRect.setLeft(globalPoint.x());
                        }
                        break;
                    case RIGHT_:
                        newRect.setWidth(globalPoint.x() - topLeft.x());  //只能改变右边界
                        break;
                    case UP_:
                        if (bottomRight.y() - globalPoint.y() <= this->minimumHeight()) {
                            newRect.setY(topLeft.y());
                        } else {
                            newRect.setY(globalPoint.y());
                        }
                        break;
                    case DOWN_:
                        newRect.setHeight(globalPoint.y() - topLeft.y());
                        break;
                    case LEFTTOP_:
                        if (bottomRight.x() - globalPoint.x() <= this->minimumWidth()) {
                            newRect.setX(topLeft.x());
                        } else {
                            newRect.setX(globalPoint.x());
                        }

                        if (bottomRight.y() - globalPoint.y() <= this->minimumHeight()) {
                            newRect.setY(topLeft.y());
                        } else {
                            newRect.setY(globalPoint.y());
                        }
                        break;
                    case RIGHTTOP_:
                        if (globalPoint.x() - topLeft.x() >= this->minimumWidth()) {
                            newRect.setWidth(globalPoint.x() - topLeft.x());
                        } else {
                            newRect.setWidth(bottomRight.x() - topLeft.x());
                        }
                        if (bottomRight.y() - globalPoint.y() >= this->minimumHeight()) {
                            newRect.setY(globalPoint.y());
                        } else {
                            newRect.setY(topLeft.y());
                        }
                        break;
                    case LEFTBOTTOM_:
                        if (bottomRight.x() - globalPoint.x() >= this->minimumWidth()) {
                            newRect.setX(globalPoint.x());
                        } else {
                            newRect.setX(topLeft.x());
                        }
                        if (globalPoint.y() - topLeft.y() >= this->minimumHeight()) {
                            newRect.setHeight(globalPoint.y() - topLeft.y());
                        } else {
                            newRect.setHeight(bottomRight.y() - topLeft.y());
                        }
                        break;
                    case RIGHTBOTTOM_:
                        newRect.setWidth(globalPoint.x() - topLeft.x());
                        newRect.setHeight(globalPoint.y() - topLeft.y());
                        break;
                    default:
                        break;
                }
                this->setGeometry(newRect);
            } else {
                move(event->globalPos() - m_mousePos_); //移动窗口
                event->accept();

            }
        }
    }
}

void BaseWidget::region(const QPoint &currentGlobalPoint) {
// 获取窗体在屏幕上的位置区域，topLeft为坐上角点，rightButton为右下角点
    QRect rect = this->rect();

    QPoint topLeft = this->mapToGlobal(rect.topLeft()); //将左上角的(0,0)转化为全局坐标
    QPoint rightButton = this->mapToGlobal(rect.bottomRight());

    int x = currentGlobalPoint.x(); //当前鼠标的坐标
    int y = currentGlobalPoint.y();

    if (((topLeft.x() + PADDING >= x) && (topLeft.x() <= x))
        && ((topLeft.y() + PADDING >= y) && (topLeft.y() <= y))) {
        // 左上角
        dir_ = LEFTTOP_;
        this->setCursor(QCursor(Qt::SizeFDiagCursor));  // 设置光标形状
    } else if (((x >= rightButton.x() - PADDING) && (x <= rightButton.x()))
               && ((y >= rightButton.y() - PADDING) && (y <= rightButton.y()))) {
        // 右下角
        dir_ = RIGHTBOTTOM_;
        this->setCursor(QCursor(Qt::SizeFDiagCursor));
    } else if (((x <= topLeft.x() + PADDING) && (x >= topLeft.x()))
               && ((y >= rightButton.y() - PADDING) && (y <= rightButton.y()))) {
        //左下角
        dir_ = LEFTBOTTOM_;
        this->setCursor(QCursor(Qt::SizeBDiagCursor));
    } else if (((x <= rightButton.x()) && (x >= rightButton.x() - PADDING))
               && ((y >= topLeft.y()) && (y <= topLeft.y() + PADDING))) {
        // 右上角
        dir_ = RIGHTTOP_;
        this->setCursor(QCursor(Qt::SizeBDiagCursor));
    } else if ((x <= topLeft.x() + PADDING) && (x >= topLeft.x())) {
        // 左边
        dir_ = LEFT_;
        this->setCursor(QCursor(Qt::SizeHorCursor));
    } else if ((x <= rightButton.x()) && (x >= rightButton.x() - PADDING)) {
        // 右边
        dir_ = RIGHT_;
        this->setCursor(QCursor(Qt::SizeHorCursor));
    } else if ((y >= topLeft.y()) && (y <= topLeft.y() + PADDING)) {
        // 上边
        dir_ = UP_;
        this->setCursor(QCursor(Qt::SizeVerCursor));
    } else if ((y <= rightButton.y()) && (y >= rightButton.y() - PADDING)) {
        // 下边
        dir_ = DOWN_;
        this->setCursor(QCursor(Qt::SizeVerCursor));
    } else {
        // 默认
        dir_ = NONE_;
        this->setCursor(QCursor(Qt::ArrowCursor));
    }
}

void BaseWidget::mouseReleaseEvent(QMouseEvent *event) {
    if (event->button() == Qt::LeftButton) {
        is_mouse_left_press_down_ = false;
        if (dir_ != NONE_) {
            this->releaseMouse(); //释放鼠标抓取
            this->setCursor(QCursor(Qt::ArrowCursor));
            dir_ = NONE_;
        }
    } else {
        return;
    }
}