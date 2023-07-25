#ifndef WINSIDEBARBUTTON_H
#define WINSIDEBARBUTTON_H

#include "interactivebuttonbase.h"
#include <QObject>
#include <QWidget>

class WinSidebarButton : public InteractiveButtonBase {
  Q_OBJECT
public:
  WinSidebarButton(QWidget *parent = nullptr);

  void setTopLeftRadius(int r);

protected:
  void paintEvent(QPaintEvent *event);
  void slotClicked();

  QPainterPath getBgPainterPath();
  QPainterPath getWaterPainterPath(Water water);

private:
  int tl_radius;
};

#endif // WINSIDEBARBUTTON_H
