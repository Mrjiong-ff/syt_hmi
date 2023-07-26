#pragma once
#include <QLabel>
#include <QLayout>
#include <QWidget>

class ShowColorWidget : public QWidget {
  Q_OBJECT
public:
  ShowColorWidget(const QString &color_string, QWidget *parent = nullptr);

private:
  QLabel *color_label;
  QLabel *rgb_label;
};
