//
// Created by jerry on 23-4-28.
//

#ifndef SYT_HMI_DEV_LOGIN_WINDOW_H
#define SYT_HMI_DEV_LOGIN_WINDOW_H

#include <QDialog>
#include <QMessageBox>
#include "ui_dev_login_window.h"
#include "syt_btn/winclosebutton.h"
#include "utils/utils.h"


QT_BEGIN_NAMESPACE
namespace Ui { class DevLoginWindow; }
QT_END_NAMESPACE

class DevLoginWindow : public QDialog {
Q_OBJECT

public:
    explicit DevLoginWindow(QWidget *parent = nullptr);

    ~DevLoginWindow() override;

//protected:
//    void closeEvent(QCloseEvent *) override;

signals:

    void signDevMode(bool);


private:
    Ui::DevLoginWindow *ui;
    WinCloseButton *m_closeBtn_;
};


#endif //SYT_HMI_DEV_LOGIN_WINDOW_H
