#pragma once

#include <QDialog>
#include <QTranslator>

namespace Ui {
class TranslateDialog;
}

class TranslateDialog : public QDialog {
  Q_OBJECT

public:
  explicit TranslateDialog(QWidget *parent = nullptr);
  ~TranslateDialog();

signals:
  void confirmLanguage(int index);

private:
  Ui::TranslateDialog *ui;
};
