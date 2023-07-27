#ifndef WINMENUBUTTON_H
#define WINMENUBUTTON_H

#include "interactivebuttonbase.h"

class WinMenuButton : public InteractiveButtonBase {
  Q_OBJECT
public:
  WinMenuButton(QWidget *parent = nullptr);

protected:
  void paintEvent(QPaintEvent *event);
  void slotClicked();
};

#endif // WINMENUBUTTON_H
