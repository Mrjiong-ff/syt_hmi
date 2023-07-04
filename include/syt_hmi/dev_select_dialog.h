//
// Created by jerry on 23-7-4.
//

#ifndef SYT_HMI_DEV_SELECT_DIALOG_H
#define SYT_HMI_DEV_SELECT_DIALOG_H

#include <QDialog>


QT_BEGIN_NAMESPACE
namespace Ui { class DevSelectDialog; }
QT_END_NAMESPACE

class DevSelectDialog : public QDialog {
Q_OBJECT

public:
    explicit DevSelectDialog(QWidget *parent = nullptr);

    ~DevSelectDialog() override;

private:
    Ui::DevSelectDialog *ui;
};


#endif //SYT_HMI_DEV_SELECT_DIALOG_H
