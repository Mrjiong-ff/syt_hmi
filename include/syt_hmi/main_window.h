//
// Created by jerry on 23-4-26.
//

#ifndef SYT_HMI_MAIN_WINDOW_H
#define SYT_HMI_MAIN_WINDOW_H

#include <iostream>
#include <QMainWindow>
#include <QDesktopWidget>
#include <QScreen>
#include <QMenu>
#include <QGraphicsOpacityEffect>


#include "ui_main_window.h"
#include "syt_btn/winclosebutton.h"
#include "syt_btn/winmenubutton.h"
#include "syt_btn/winminbutton.h"
#include "syt_btn/winmaxbutton.h"
#include "syt_hmi/dev_login_window.h"
#include "syt_hmi/dev_window.h"
#include "utils/utils.h"

#include "syt_rclcomm//rcl_comm.h"
//#include "syt_rviz/qrviz.h"


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

#define PADDING 2
// 定义方向枚举，用于判断鼠标在mainWindow的哪个位置
enum Direction {
    UP = 0, DOWN = 1, LEFT, RIGHT, LEFTTOP, LEFTBOTTOM, RIGHTBOTTOM, RIGHTTOP, NONE
};

class MainWindow : public QMainWindow {
Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);

    void region(const QPoint &currentGlobalPoint);  //用于定位鼠标移动的位置,改变光标

    ~MainWindow() override;

protected:

    void mousePressEvent(QMouseEvent *event) override;

    void mouseMoveEvent(QMouseEvent *event) override;

    void mouseReleaseEvent(QMouseEvent *event) override;

    bool eventFilter(QObject *obj, QEvent *ev) override;

    void resizeEvent(QResizeEvent *event) override;

    virtual void keyPressEvent(QKeyEvent *event) override;

private:
    void initNode();

    void initWidget();

    void settingConnection();

    void setAllButtonsEnabled(QWidget *parent, bool enabled, QPushButton *excludedButton = nullptr);


private slots:

    void slotMaxBtnClicked();

    void slotShowDevLoginWindow();

    void slotLockScreen();

    void slotDevWindow();

    void slotPrevPage();

    void slotNextPage();

    void resetBtnClicked();

    void startBtnClicked();

    void stopBtnClicked();

private:
    Ui::MainWindow *ui;

    SytRclComm *rclcomm;

    // 定义的一些bool类型标志位
    bool is_normal_size_ = true;
    bool is_mouse_left_press_down_ = false;
    bool is_lock_screen_ = false;
    int page_btn_w_scale_ = 50;
    int page_btn_h_scale_ = 9;

    // 一些自定义的按钮控件
    WinMenuButton *m_menuBtn_;
    WinMinButton *m_minBtn_;
    WinMaxButton *m_maxBtn_;
    WinCloseButton *m_closeBtn_;


    DevLoginWindow *dev_login_window_;
    InteractiveButtonBase *prev_btn;
    InteractiveButtonBase *next_btn_;
//    QRviz *rviz_widget_;

    // about action
    QMenu *m_titleMenu_;
    QMenu *m_menu_;
    QAction *minAct_;
    QAction *maxAct_;
    QAction *closeAct_;
    QAction *fullAct_;

    QAction *devAct_;
    QAction *helpAct_;
    QAction *aboutAct_;

    QPoint m_mousePos_;
    Direction dir_;    // 窗口大小改变时，记录改变方向
};


#endif //SYT_HMI_MAIN_WINDOW_H
