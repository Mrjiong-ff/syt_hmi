#pragma once

#include "syt_btn/interactivebuttonbase.h"
#include "syt_hmi/input_extra_param_widget.h"
#include <QDialog>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QRegExpValidator>
#include <QSpacerItem>
#include <QWizard>
#include <QWizardPage>
#include <opencv2/opencv.hpp>

QT_BEGIN_NAMESPACE
namespace Ui {
class ClothStyleDialog;
}
QT_END_NAMESPACE

class ClothStyleDialog : public QDialog {
  Q_OBJECT

public:
  ClothStyleDialog(QWidget *parent = nullptr);
  ~ClothStyleDialog();

signals:
  void signCreateFromCAD(ClothStyleDialog *parent);
  void signAutoCreateStyle(ClothStyleDialog *parent);
  void signManualInputParam(ClothStyleDialog *parent);
  void signCreateFromSource(ClothStyleDialog *parent);

private:
  Ui::ClothStyleDialog *ui;
};

//////////////// 从CAD创建(向导) ////////////////
class CreateFromCADWizard : public QWizard {
  Q_OBJECT
public:
  CreateFromCADWizard(QWidget *parent = nullptr);
  ~CreateFromCADWizard(){};

private slots:
};

// 选择cad文件page
class ChooseCADPage : public QWizardPage {
  Q_OBJECT
public:
  ChooseCADPage(QWidget *parent = nullptr);
  ~ChooseCADPage(){};
};

//////////////// 自动创建(向导) ////////////////
class AutoCreateStyleWizard : public QWizard {
  Q_OBJECT
private:
  std::vector<cv::Point2i> front_contour_;
  std::vector<cv::Point2i> back_contour_;
  std::vector<cv::Point2i> front_keypoints_;
  std::vector<cv::Point2i> back_keypoints_;
  syt_msgs::msg::ClothStyle cloth_style_front_;
  syt_msgs::msg::ClothStyle cloth_style_back_;
  std::string file_name_;

public:
  AutoCreateStyleWizard(QWidget *parent = nullptr);
  ~AutoCreateStyleWizard(){};

signals:
  void signMoveHand();
  void signMoveHandResult(bool result);
  void signDetectCloth(int cloth_type);
  void signDetectClothResult(bool result, int cloth_type);
  void signCreateStyle(syt_msgs::msg::ClothStyle cloth_style_front, syt_msgs::msg::ClothStyle cloth_style_back);
  void signCreateStyleResult(bool result);
  void signSetRenameEdit(QString file_name);
  void signRenameClothStyle(std::string old_name, std::string new_name);
  void signRenameClothStyleResult(bool result);

public slots:
  void slotMoveHand();
  void slotMoveHandResult(bool result);
  void slotDetectCloth(int cloth_type);
  void slotDetectClothResult(bool result, int cloth_type, std::vector<cv::Point2i> contour, std::vector<cv::Point2i> keypoints);
  void slotSetExtraParam(syt_msgs::msg::ClothStyle cloth_style);
  void slotCreateStyle();
  void slotCreateStyleResult(bool result, std::string file_name);
  void slotSetRenameEdit();
  void slotRenameClothStyle();
  void slotRenameClothStyleResult(bool result);
};

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

//////////////// 手动创建(向导) ////////////////
class ManualInputParamWizard : public QWizard {
  Q_OBJECT
public:
  ManualInputParamWizard(QWidget *parent = nullptr);
  ~ManualInputParamWizard(){};
};

//////////////// 从已有文件创建(向导) ////////////////
class CreateFromSourceWizard : public QWizard {
  Q_OBJECT
public:
  CreateFromSourceWizard(QWidget *parent = nullptr);
  ~CreateFromSourceWizard(){};
};
