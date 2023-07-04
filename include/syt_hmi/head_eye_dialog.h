//
// Created by jerry on 23-7-4.
//

#ifndef SYT_HMI_HEAD_EYE_DIALOG_H
#define SYT_HMI_HEAD_EYE_DIALOG_H

#include <QDialog>
#include "utils/utils.h"

QT_BEGIN_NAMESPACE
namespace Ui { class HeadEyeDialog; }
QT_END_NAMESPACE

class HeadEyeDialog : public QDialog {
Q_OBJECT

public:
    explicit HeadEyeDialog(QWidget *parent = nullptr);

    ~HeadEyeDialog() override;

private:
    Ui::HeadEyeDialog *ui;
};


#endif //SYT_HMI_HEAD_EYE_DIALOG_H
