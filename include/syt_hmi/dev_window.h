//
// Created by jerry on 23-5-4.
//

#ifndef SYT_HMI_DEV_WINDOW_H
#define SYT_HMI_DEV_WINDOW_H

#include <QWidget>
#include "ui_dev_window.h"


QT_BEGIN_NAMESPACE
namespace Ui { class DevWindow; }
QT_END_NAMESPACE

class DevWindow : public QWidget {
Q_OBJECT

public:
    explicit DevWindow(QWidget *parent = nullptr);

    ~DevWindow() override;

signals:
    void closeDevWindow();

private:
    Ui::DevWindow *ui;
};


#endif //SYT_HMI_DEV_WINDOW_H
