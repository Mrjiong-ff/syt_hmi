#pragma once

#include "syt_btn/interactivebuttonbase.h"
#include "syt_hmi/input_extra_param_widget.h"

#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QLayout>
#include <QRegExpValidator>
#include <QSpacerItem>
#include <QWizardPage>

// 1.移动抓手
class MoveHandPage : public QWizardPage {
  Q_OBJECT
private:
  bool moved_hand_ = false;

  InteractiveButtonBase *move_hand_button;

public:
  MoveHandPage(QWidget *parent = nullptr);
  ~MoveHandPage(){};

  bool validatePage() override;

signals:
  void signMoveHand();

public slots:
  void slotMoveHandResult(bool result);
};

// 2.提醒放置裁片
class RemindClothPage : public QWizardPage {
  Q_OBJECT
public:
  RemindClothPage(QWidget *parent = nullptr, int cloth_type = 0);
  ~RemindClothPage(){};
};

// 3.调用检测服务获取裁片轮廓和关键点
class DetectClothPage : public QWizardPage {
  Q_OBJECT
private:
  int cloth_type_;
  bool detected_result_ = false;

public:
  DetectClothPage(QWidget *parent = nullptr, int cloth_type = 0);
  ~DetectClothPage(){};

  bool validatePage() override;

signals:
  void signDetectCloth(int cloth_type);

public slots:
  void slotDetectClothResult(bool result, int cloth_type);
};

// 4.手动输入额外参数
class InputExtraParamPage : public QWizardPage {
  Q_OBJECT
private:
  int cloth_type_;
  InputExtraParamWidget *input_extra_param_widget;

public:
  InputExtraParamPage(QWidget *parent = nullptr, int cloth_type = 0);
  ~InputExtraParamPage(){};

  bool validatePage() override;

signals:
  void signSetExtraParam(syt_msgs::msg::ClothStyle cloth_style);
};

// 5.调用样式服务
class CreateStylePage : public QWizardPage {
  Q_OBJECT
private:
  int create_result_ = false;

public:
  CreateStylePage(QWidget *parent = nullptr);
  ~CreateStylePage(){};

  bool validatePage() override;

signals:
  void signCreateStyle();
  void signSetRenameEdit();

public slots:
  void slotCreateStyleResult(bool result);
};

// 6.修改样式名
class RenameClothStylePage : public QWizardPage {
  Q_OBJECT
private:
  int rename_result_ = false;

  QLabel *old_name_label;
  QLabel *new_name_label;
  QLineEdit *old_name_line_edit;
  QLineEdit *new_name_line_edit;
  InteractiveButtonBase *rename_btn;

public:
  RenameClothStylePage(QWidget *parent = nullptr);
  ~RenameClothStylePage(){};

  bool validatePage() override;

signals:
  void signRenameClothStyle();

public slots:
  void slotSetRenameEdit(QString file_name);
  void slotRenameClothStyleResult(bool result);
};

//////////////// 从CAD创建(向导) ////////////////
// 选择cad文件page
class ChooseCADPage : public QWizardPage {
  Q_OBJECT
public:
  ChooseCADPage(QWidget *parent = nullptr);
  ~ChooseCADPage(){};
};

