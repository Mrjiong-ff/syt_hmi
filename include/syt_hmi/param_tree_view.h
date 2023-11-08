#pragma once

#include <QMouseEvent>
#include <QTreeView>
#include <vector>

struct ParamLine {
  QString name;
  QString dtype;
  int length;
  QString min_range;
  QString max_range;
  QString value;
  bool is_array;
  QString comment;
};

class ParamTreeView : public QTreeView {
  Q_OBJECT
public:
  ParamTreeView(QWidget *parent = Q_NULLPTR);

protected:
  void mousePressEvent(QMouseEvent *event) override;
  void keyPressEvent(QKeyEvent *event) override;
};
