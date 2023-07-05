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
#include <QtConcurrent/QtConcurrent>
#include <QScrollBar>

#include "ui_main_window.h"
#include "syt_btn/winclosebutton.h"
#include "syt_btn/winmenubutton.h"
#include "syt_btn/winminbutton.h"
#include "syt_btn/winmaxbutton.h"
#include "syt_hmi/dev_login_window.h"
#include "syt_hmi/dev_window.h"
#include "syt_hmi/ota_update_dialog.h"
#include "syt_hmi/lock_dialog.h"
#include "syt_hmi/user_opt_dialog.h"
#include "utils/utils.h"
#include "utils/waitingspinnerwidget.h"
#include "syt_hmi/dev_select_dialog.h"
#include "syt_hmi/head_eye_dialog.h"

#include "syt_rclcomm/rcl_comm.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

#define PADDING 2
// 定义方向枚举，用于判断鼠标在mainWindow的哪个位置
enum Direction {
    UP = 0, DOWN = 1, LEFT, RIGHT, LEFTTOP, LEFTBOTTOM, RIGHTBOTTOM, RIGHTTOP, NONE
};

enum LIGHT_COLOR {
    RED = 0, YELLOW = 1, GREEN, GRAY
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

    void setMutuallyLight(LIGHT_COLOR);

    void deleteAll();

    void initOther();

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

    void errorNodeMsgSlot(QString msg);

    void triggeredOTAUpdate();

    void otaResultShow(bool res, QString msg);

    void slotStartHeadEyeWindow();

    void otaInstallSuccess(bool res, QString msg);

    void slotVisualLoadCloth(int machine_id, int cam_id, QImage image);

signals:

    void signHeadEyeWindowShow();

private:
    Ui::MainWindow *ui;

    SytRclComm *rclcomm = nullptr;

    // 定义的一些bool类型标志位
    bool is_mouse_left_press_down_ = false;
    bool is_load_cloth_on = false;
    bool is_comp_cloth_on = false;

    // 一些自定义的按钮控件
    WinMenuButton *m_menuBtn_;
    WinMinButton *m_hideBtn_;
    WinMaxButton *m_maxBtn_;
    WinCloseButton *m_closeBtn_;


    DevLoginWindow *dev_login_window_;
    InteractiveButtonBase *prev_btn;
    InteractiveButtonBase *next_btn_;

    // about action
    QMenu *m_titleMenu_;
    QMenu *m_menu_;
    QAction *minAct_;
    QAction *maxAct_;
    QAction *closeAct_;
    QAction *fullAct_;

    QAction *updateAct_;
    QAction *helpAct_;
    QAction *aboutAct_;

    WaitingSpinnerWidget *_localPodsSpinnerWidget;

    QPoint m_mousePos_;
    Direction dir_;    // 窗口大小改变时，记录改变方向
};


#endif //SYT_HMI_MAIN_WINDOW_H
