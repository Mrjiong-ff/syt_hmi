#pragma once
#include <QWidget>

namespace Ui {
class ProgressBar;
}

class ProgressBar : public QWidget {
  Q_OBJECT

public:
  explicit ProgressBar(QWidget *parent = nullptr);
  ~ProgressBar();

  void setPercentage(bool enable);
  void setLabel(QString label);
  void setProgressBar(int numerator, int denominator);

private:
  Ui::ProgressBar *ui;

  bool show_percentage_;

  int numerator_;
  int denominator_;
};
