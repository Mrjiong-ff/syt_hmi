#pragma once

#include "utils/utils.h"
#include <QWidget>

namespace Ui {
class ParamSetWidget;
}

class ParamSetWidget : public QWidget {
  Q_OBJECT

public:
  explicit ParamSetWidget(QWidget *parent = nullptr);
  ~ParamSetWidget();

protected:
  bool event(QEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;

private:
  Ui::ParamSetWidget *ui;

  // 移动窗口
  bool is_mouse_left_press_down_;
  QPoint mouse_pos_;

  void setButtonFrame();
  void switchPage();
  void bindSwitchMode();
  void bindSewingMachine();
  void bindComposeMachine();
  void bindLoadMachine();

signals:
  void signSwitchSewingMode(int mode);

  void signSewingMachineReset();
  void signSewingMachineLabelReset();
  void signSewingMachineLabelWidth(bool enable, int side, float width, float position);
  void signSewingMachineNeedle(float line_1, float line_2, float line_3, float line_4);
  void signSewingMachineThickness(float thickness);

  void signComposeMachineReset();
  void signComposeMachineBlowHeight(float height);
  void signComposeMachineTableLight(float ratio);

  void signLoadMachineReset(int id);
  void signLoadMachineThickness(int id, float thickness);
};
