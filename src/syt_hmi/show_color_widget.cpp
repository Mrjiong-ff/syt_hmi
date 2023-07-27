#include "syt_hmi/show_color_widget.h"

ShowColorWidget::ShowColorWidget(const QString &color_string, QWidget *parent) : QWidget(parent) {
  uint32_t color = color_string.toUInt(nullptr, 16);
  rgb_label      = new QLabel(QString("#%1").arg(color, 6, 16, QLatin1Char('0')), this);
  uint8_t red    = (color & 0xff0000) >> 16;
  uint8_t green  = (color & 0x00ff00) >> 8;
  uint8_t blue   = color & 0x0000ff;
  color_label    = new QLabel(this);
  color_label->setStyleSheet(QString("background-color: rgb(%1, %2, %3);border-width: 1px;border-style: solid;border-color: rgb(%4, %5, %6);").arg(red).arg(green).arg(blue).arg(255 - red).arg(255 - green).arg(255 - blue));
  color_label->setFixedSize(20, 20);

  QHBoxLayout *layout = new QHBoxLayout(this);
  layout->addWidget(rgb_label);
  layout->addWidget(color_label);
  layout->setContentsMargins(2, 2, 2, 2);
  setLayout(layout);
}
