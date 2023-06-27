//
// Created by jerry on 23-6-27.
//

#ifndef SYT_HMI_BASEWIDGET_H
#define SYT_HMI_BASEWIDGET_H

#include <QWidget>
#include <QMouseEvent>

#define PADDING 2
// 定义方向枚举，用于判断鼠标在mainWindow的哪个位置
enum DIRECTION {
    UP_ = 0, DOWN_ = 1, LEFT_, RIGHT_, LEFTTOP_, LEFTBOTTOM_, RIGHTBOTTOM_, RIGHTTOP_, NONE_
};

class BaseWidget : public QWidget {
Q_OBJECT

public:
    explicit BaseWidget(QWidget *parent = nullptr);

    void region(const QPoint &currentGlobalPoint);  //用于定位鼠标移动的位置,改变光标


protected:
    void mousePressEvent(QMouseEvent *event) override;

    void mouseMoveEvent(QMouseEvent *event) override;

    void mouseReleaseEvent(QMouseEvent *event) override;


private:
    bool is_mouse_left_press_down_ = false;
    DIRECTION dir_;    // 窗口大小改变时，记录改变方向
    QPoint m_mousePos_;

};


#endif //SYT_HMI_BASEWIDGET_H
