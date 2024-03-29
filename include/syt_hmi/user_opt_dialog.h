//
// Created by jerry on 23-6-29.
//

#ifndef SYT_HMI_USER_OPT_DIALOG_H
#define SYT_HMI_USER_OPT_DIALOG_H

#include <QDialog>
#include <QFileDialog>
#include "utils/utils.h"
#include <opencv2/core/core.hpp>

QT_BEGIN_NAMESPACE
namespace Ui { class UserOptDialog; }
QT_END_NAMESPACE

class UserOptDialog : public QDialog {
Q_OBJECT

public:
    explicit UserOptDialog(QWidget *parent = nullptr);

    ~UserOptDialog() override;

signals:
    void systemStart();

private:
    void readConfigAndSet();

    Ui::UserOptDialog *ui;
};


#endif //SYT_HMI_USER_OPT_DIALOG_H
