#pragma once
#include <QDialog>

QT_BEGIN_NAMESPACE
namespace Ui {
class LockDialog;
}
QT_END_NAMESPACE

class LockDialog : public QDialog {
  Q_OBJECT

public:
  explicit LockDialog(QWidget *parent = nullptr);

  ~LockDialog() override;

private:
  Ui::LockDialog *ui;
};
