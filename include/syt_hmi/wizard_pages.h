#pragma once

#include "syt_btn/interactivebuttonbase.h"
#include "syt_hmi/input_extra_param_widget.h"
#include "syt_hmi/input_length_param_widget.h"

#include <QComboBox>
#include <QLabel>
#include <QLayout>
#include <QLineEdit>
#include <QMessageBox>
#include <QRegExpValidator>
#include <QSpacerItem>
#include <QWizardPage>

// 1.移动抓手
class MoveHandPage : public QWizardPage {
  Q_OBJECT
private:
  bool moved_hand_ = false;

  InteractiveButtonBase *move_hand_btn_;

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
  InputExtraParamWidget *input_extra_param_widget_;

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
  QLabel *old_name_label_;
  QLabel *new_name_label_;
  QLineEdit *old_name_line_edit_;
  QLineEdit *new_name_line_edit_;
  InteractiveButtonBase *rename_btn_;

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

// 7.选择cad文件page
class ChooseCADPage : public QWizardPage {
  Q_OBJECT
public:
  ChooseCADPage(QWidget *parent = nullptr);
  ~ChooseCADPage(){};
};

// 8.输入长度参数
class InputLengthParamPage : public QWizardPage {
  Q_OBJECT
private:
  // QLabel *cloth_length_label;
  // QLabel *bottom_length_label;
  // QLabel *oxter_length_label;
  // QLabel *shoulder_length_label;
  // QLabel *side_length_label;

  // QLineEdit *cloth_length_line_edit;
  // QLineEdit *bottom_length_line_edit;
  // QLineEdit *oxter_length_line_edit;
  // QLineEdit *shoulder_length_line_edit;
  // QLineEdit *side_length_line_edit;

  int cloth_type_;
  InputLengthParamWidget *input_length_param_widget;

public:
  InputLengthParamPage(QWidget *parent = nullptr, int cloth_type = 0);
  ~InputLengthParamPage(){};

  bool validatePage() override;

signals:
  void signSetLengthParam(syt_msgs::msg::ClothStyle cloth_style);
};
