//
// Created by jerry on 23-6-28.
//

#ifndef SYT_HMI_OTA_UPDATE_DIALOG_H
#define SYT_HMI_OTA_UPDATE_DIALOG_H

#include <QDialog>


QT_BEGIN_NAMESPACE
namespace Ui { class OtaUpdateDialog; }
QT_END_NAMESPACE

class OtaUpdateDialog : public QDialog {
Q_OBJECT

public:
    explicit OtaUpdateDialog(QWidget *parent = nullptr);

    ~OtaUpdateDialog() override;

signals:


public slots:

    void clearProcessValue();

    void updateProcessValue(int val, int total);

    void getDownloadRes(bool,QString);

private:
    Ui::OtaUpdateDialog *ui;
};


#endif //SYT_HMI_OTA_UPDATE_DIALOG_H
