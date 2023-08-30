#pragma once

#include "syt_btn/interactivebuttonbase.h"
#include "syt_hmi/input_extra_param_widget.h"
#include "syt_hmi/input_length_param_widget.h"
#include "syt_hmi/input_tolerance_param_widget.h"
#include "syt_hmi/style_display_widget.h"

#include <QCheckBox>
#include <QComboBox>
#include <QFileDialog>
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

public:
  MoveHandPage(QWidget *parent = nullptr);
  ~MoveHandPage(){};

  bool validatePage() override;

signals:
  void signMoveHand();

public slots:
  void slotMoveHandResult(bool result);
};

// 2.调用检测服务获取裁片轮廓和关键点
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

// 3.手动输入额外参数
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

// 4.调用样式服务
class CreateStylePage : public QWizardPage {
  Q_OBJECT
private:
  int create_result_ = false;
  QString prefix_    = "/home/syt/style";

public:
  CreateStylePage(QWidget *parent = nullptr);
  ~CreateStylePage(){};

  bool validatePage() override;

signals:
  void signCreateStyle(QString prefix_);
  void signSetRenameEdit();

public slots:
  void slotCreateStyleResult(bool result);
};

// 5.修改样式名
class RenameClothStylePage : public QWizardPage {
  Q_OBJECT
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

// 6.选择cad文件page
class ChooseCADPage : public QWizardPage {
  Q_OBJECT
public:
  ChooseCADPage(QWidget *parent = nullptr);
  ~ChooseCADPage(){};
};

// 7.输入长度参数
class InputLengthParamPage : public QWizardPage {
  Q_OBJECT
private:
  int cloth_type_;
  InputLengthParamWidget *input_length_param_widget_;

public:
  InputLengthParamPage(QWidget *parent = nullptr, int cloth_type = 0);
  ~InputLengthParamPage(){};

  bool validatePage() override;

signals:
  void signSetLengthParam(syt_msgs::msg::ClothStyle cloth_style);
};

// 8.输入误差参数
class InputToleranceParamPage : public QWizardPage {
  Q_OBJECT
private:
  InputToleranceParamWidget *input_tolerance_param_widget_;

public:
  InputToleranceParamPage(QWidget *parent = nullptr);
  ~InputToleranceParamPage(){};

  bool validatePage() override;

signals:
  void signSetToleranceParam(syt_msgs::msg::ClothStyle cloth_style);
};

// 9.显示所有参数
class StyleDisplayPage : public QWizardPage {
  Q_OBJECT
private:
  int cloth_type_;
  StyleDisplayWidget *style_display_widget_;

public:
  StyleDisplayPage(QWidget *parent = nullptr, int cloth_type = 0);
  ~StyleDisplayPage(){};

  bool validatePage() override;

signals:
  void signSetFullParam(syt_msgs::msg::ClothStyle cloth_style);

public slots:
  void slotSetFullParam(syt_msgs::msg::ClothStyle cloth_style);
};

// 10.选择样式文件
class ChooseStylePage : public QWizardPage {
  Q_OBJECT
private:
  bool get_style_ = false;

public:
  ChooseStylePage(QWidget *parent = nullptr);
  ~ChooseStylePage(){};

  bool validatePage() override;

signals:
  void signGetClothStyle(QString prefix, QString style_name);

public slots:
  void slotGetClothStyleResult(bool result);
};
