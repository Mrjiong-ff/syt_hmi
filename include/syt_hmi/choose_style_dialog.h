#pragma once
#include "utils/utils.h"
#include "utils/waitingspinnerwidget.h"
#include <QDialog>
#include <QFileDialog>
#include <QFileSystemModel>
#include <QMenu>

QT_BEGIN_NAMESPACE
namespace Ui {
class ChooseStyleDialog;
}
QT_END_NAMESPACE

class ChooseStyleDialog : public QDialog {
  Q_OBJECT

private:
  Ui::ChooseStyleDialog *ui;

  QMenu *pop_menu_;
  QAction *delete_act_;

  WaitingSpinnerWidget *waiting_spinner_widget_;

public:
  QString style_directory_;
  QFileSystemModel style_file_model_;

  explicit ChooseStyleDialog(QWidget *parent = nullptr);
  ~ChooseStyleDialog() override;

signals:
  void signSetCurrentStyle(QString prefix, QString file_name);

public slots:
  void slotSetStylePath();
  void slotSetCurrentStyleFinish(bool result);

private slots:
  void deleteStyleFile();
  void showContextMenu(const QPoint &pos);
};
