#pragma once
#include "utils/utils.h"
#include "utils/waitingspinnerwidget.h"
#include <QDialog>

QT_BEGIN_NAMESPACE
namespace Ui {
class HandEyeDialog;
}
QT_END_NAMESPACE

class HandEyeDialog : public QDialog {
  Q_OBJECT

public:
  explicit HandEyeDialog(QWidget *parent = nullptr);

  ~HandEyeDialog() override;

signals:
  void signCompStart();
  void signSewingStart();
  void signFittingStart();

private:
  Ui::HandEyeDialog *ui;

  WaitingSpinnerWidget *waiting_spinner_widget_;

public slots:
  void slotCompCalibRes(bool result);
  void slotSewingCalibRes(bool result);
};
