//
// Created by jerry on 23-4-28.
//

#ifndef SYT_HMI_TIP_DIALOG_H
#define SYT_HMI_TIP_DIALOG_H

#include <QDialog>
#include "ui_tip_dialog.h"


QT_BEGIN_NAMESPACE
namespace Ui { class TipDialog; }
QT_END_NAMESPACE

class TipDialog : public QDialog {
Q_OBJECT

public:
    explicit TipDialog(QWidget *parent = nullptr);

    ~TipDialog() override;

private:
    Ui::TipDialog *ui;
};


#endif //SYT_HMI_TIP_DIALOG_H
