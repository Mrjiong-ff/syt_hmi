#pragma once
#include <QDialog>

class BaseDialog : public QDialog {
  Q_OBJECT

public:
  explicit BaseDialog(QWidget *parent = nullptr);
};
