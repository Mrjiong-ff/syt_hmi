#ifndef SYT_HMI_HEAD_EYE_DIALOG_H
#define SYT_HMI_HEAD_EYE_DIALOG_H

#include "utils/utils.h"
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
  void signSewingStart();

  void signCompStart();

private:
  Ui::HandEyeDialog *ui;
};

#endif // SYT_HMI_HEAD_EYE_DIALOG_H
