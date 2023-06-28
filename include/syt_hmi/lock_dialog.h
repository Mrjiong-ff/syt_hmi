//
// Created by jerry on 23-6-28.
//

#ifndef SYT_HMI_LOCK_DIALOG_H
#define SYT_HMI_LOCK_DIALOG_H

#include <QDialog>


QT_BEGIN_NAMESPACE
namespace Ui { class LockDialog; }
QT_END_NAMESPACE

class LockDialog : public QDialog {
Q_OBJECT

public:
    explicit LockDialog(QWidget *parent = nullptr);

    ~LockDialog() override;

private:
    Ui::LockDialog *ui;
};


#endif //SYT_HMI_LOCK_DIALOG_H
