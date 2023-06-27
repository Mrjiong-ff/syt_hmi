//
// Created by jerry on 23-6-27.
//

#ifndef SYT_HMI_DIALOG_H
#define SYT_HMI_DIALOG_H

#include <QDialog>

class BaseDialog : public QDialog {
Q_OBJECT

public:
    explicit BaseDialog(QWidget *parent = nullptr);

};


#endif //SYT_HMI_DIALOG_H
