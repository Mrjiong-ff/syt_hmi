#pragma once

#include <QDialog>
#include <QTranslator>

namespace Ui {
class TranslateDialog;
}

class TranslateDialog : public QDialog {
  Q_OBJECT

public:
  explicit TranslateDialog(QWidget *parent = nullptr, int index = 0);
  ~TranslateDialog();

signals:
  void confirmLanguage(int index);

private:
  Ui::TranslateDialog *ui;
};
