#include "syt_hmi/progress_bar.h"
#include "ui_progress_bar.h"

ProgressBar::ProgressBar(QWidget *parent) : QWidget(parent), ui(new Ui::ProgressBar),
                                            show_percentage_(false),
                                            numerator_(0),
                                            denominator_(100) {
  ui->setupUi(this);
  ui->label->setAlignment(Qt::AlignCenter);
  ui->label->setStyleSheet(QString("QLabel{font-size: 30px; font-weight: bold;}"));
}

ProgressBar::~ProgressBar() {}

void ProgressBar::setPercentage(bool enable) {
  show_percentage_ = enable;
}

void ProgressBar::setLabel(QString label) {
  ui->label->setText(label);
}

void ProgressBar::setProgressBar(int numerator, int denominator) {
  numerator_   = numerator;
  denominator_ = denominator;
  ui->bar->setValue(100 * numerator_ / denominator_);

  if (show_percentage_) {
    ui->bar->setFormat(QString("%1%").arg(100 * numerator_ / denominator_));
  } else {
    ui->bar->setFormat(QString("%1/%2").arg(numerator_).arg(denominator_));
  }
}
