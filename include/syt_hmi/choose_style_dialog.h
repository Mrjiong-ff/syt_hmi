#pragma once
#include "utils/utils.h"
#include "utils/waitingspinnerwidget.h"
#include <QDialog>
#include <QFileDialog>
#include <QFileSystemModel>

QT_BEGIN_NAMESPACE
namespace Ui {
class ChooseStyleDialog;
}
QT_END_NAMESPACE

class ChooseStyleDialog : public QDialog {
  Q_OBJECT

private:
  Ui::ChooseStyleDialog *ui;

  WaitingSpinnerWidget *waiting_spinner_widget_;

public:
  QString style_directory_;
  QFileSystemModel style_file_model_;

  explicit ChooseStyleDialog(QWidget *parent = nullptr);
  ~ChooseStyleDialog() override;

signals:
  void signChoseStyle(QString prefix, QString file_name);

public slots:
  void slotSetStylePath();
};
