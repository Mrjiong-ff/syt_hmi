#pragma once

#include "syt_btn/interactivebuttonbase.h"
#include <QDialog>
#include <QWizard>
#include <QLabel>

QT_BEGIN_NAMESPACE
namespace Ui {
class ClothStyleDialog;
}
QT_END_NAMESPACE

class ClothStyleDialog : public QDialog {
  Q_OBJECT

public:
  ClothStyleDialog(QWidget *parent = nullptr);
  ~ClothStyleDialog();

signals:
  void signCreateFromCAD(ClothStyleDialog *parent);
  void signAutoCreateStyle(ClothStyleDialog *parent);
  void signManualInputParam(ClothStyleDialog *parent);
  void signCreateFromSource(ClothStyleDialog *parent);

private:
  Ui::ClothStyleDialog *ui;
};
