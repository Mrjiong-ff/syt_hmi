#include "syt_hmi/auto_create_wizard.h"

AutoCreateStyleWizard::AutoCreateStyleWizard(QWidget *parent) : QWizard(parent) {
  setWindowTitle("自动创建样式文件");
  setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
  setButtonText(WizardButton::BackButton, "上一步");
  setButtonText(WizardButton::NextButton, "下一步");
  setButtonText(WizardButton::CancelButton, "取消");
  setButtonText(WizardButton::FinishButton, "完成");
  setModal(true);

  waiting_spinner_widget_ = new WaitingSpinnerWidget(this);

  // ----- wizard流程 -----
  // 1.移动抓手
  MoveHandPage *move_hand_and_notify_page = new MoveHandPage(parent);
  addPage(move_hand_and_notify_page);

  // 2.1提醒放置前片
  RemindClothPage *remind_cloth_front_page = new RemindClothPage(parent, 0);
  addPage(remind_cloth_front_page);

  // 2.2前片轮廓关键点检测
  DetectClothPage *detect_cloth_front_page = new DetectClothPage(parent, 0);
  addPage(detect_cloth_front_page);

  // 2.3手动输入前片额外参数
  InputExtraParamPage *input_extra_param_front_page = new InputExtraParamPage(parent, 0);
  addPage(input_extra_param_front_page);

  // 3.1提醒放置后片
  RemindClothPage *remind_cloth_back_page = new RemindClothPage(parent, 1);
  addPage(remind_cloth_back_page);

  // 3.2前片轮廓关键点检测
  DetectClothPage *detect_cloth_back_page = new DetectClothPage(parent, 1);
  addPage(detect_cloth_back_page);

  // 3.3手动输入后片额外参数
  InputExtraParamPage *input_extra_param_back_page = new InputExtraParamPage(parent, 1);
  addPage(input_extra_param_back_page);

  // 4.调用样式服务
  CreateStylePage *create_style_page = new CreateStylePage(parent);
  addPage(create_style_page);

  // 5.修改样式名
  RenameClothStylePage *rename_cloth_style_page = new RenameClothStylePage(parent);
  addPage(rename_cloth_style_page);

  //////////////////// 信号处理 ////////////////////
  // 1.移动合片抓手至安全位置
  connect(move_hand_and_notify_page, &MoveHandPage::signMoveHand, this, &AutoCreateStyleWizard::slotMoveHand);
  connect(this, &AutoCreateStyleWizard::signMoveHandResult, move_hand_and_notify_page, &MoveHandPage::slotMoveHandResult);

  // 2.调用摄像头检测模板
  connect(detect_cloth_front_page, &DetectClothPage::signDetectCloth, this, &AutoCreateStyleWizard::slotDetectCloth);
  connect(this, &AutoCreateStyleWizard::signDetectClothResult, detect_cloth_front_page, &DetectClothPage::slotDetectClothResult);

  // 3.调用摄像头检测模板
  connect(detect_cloth_back_page, &DetectClothPage::signDetectCloth, this, &AutoCreateStyleWizard::slotDetectCloth);
  connect(this, &AutoCreateStyleWizard::signDetectClothResult, detect_cloth_back_page, &DetectClothPage::slotDetectClothResult);

  // 4.输入额外参数
  qRegisterMetaType<syt_msgs::msg::ClothStyle>("syt_msgs::msg::ClothStyle");
  connect(input_extra_param_front_page, &InputExtraParamPage::signSetExtraParam, this, &AutoCreateStyleWizard::slotSetExtraParam); // 前片
  connect(input_extra_param_back_page, &InputExtraParamPage::signSetExtraParam, this, &AutoCreateStyleWizard::slotSetExtraParam);  // 后片

  // 5.调用创建样式服务
  qRegisterMetaType<syt_msgs::msg::ClothInfo>("syt_msgs::msg::ClothInfo");
  connect(create_style_page, &CreateStylePage::signCreateStyle, this, &AutoCreateStyleWizard::slotCreateStyle);
  connect(this, &AutoCreateStyleWizard::signCreateStyleResult, create_style_page, &CreateStylePage::slotCreateStyleResult);

  // 6.重命名服务
  connect(create_style_page, &CreateStylePage::signSetRenameEdit, this, &AutoCreateStyleWizard::slotSetRenameEdit);
  connect(this, &AutoCreateStyleWizard::signSetRenameEdit, rename_cloth_style_page, &RenameClothStylePage::slotSetRenameEdit);
  connect(rename_cloth_style_page, &RenameClothStylePage::signRenameClothStyle, this, &AutoCreateStyleWizard::slotRenameClothStyle);
  connect(this, &AutoCreateStyleWizard::signRenameClothStyleResult, rename_cloth_style_page, &RenameClothStylePage::slotRenameClothStyleResult);
}

void AutoCreateStyleWizard::slotMoveHand() {
  waiting_spinner_widget_->start();
  emit signMoveHand();
}

void AutoCreateStyleWizard::slotMoveHandResult(bool result) {
  waiting_spinner_widget_->stop();
  emit signMoveHandResult(result);
}

void AutoCreateStyleWizard::slotDetectCloth(int cloth_type) {
  waiting_spinner_widget_->start();
  emit signDetectCloth(cloth_type);
}

void AutoCreateStyleWizard::slotDetectClothResult(bool result, int cloth_type, syt_msgs::msg::ClothInfo cloth_info) {
  waiting_spinner_widget_->stop();
  if (cloth_type == 0) {
    cloth_info_front_.cloth_contour = cloth_info.cloth_contour;
    cloth_info_front_.keypoint_info = cloth_info.keypoint_info;
  } else if (cloth_type == 1) {
    cloth_info_back_.cloth_contour = cloth_info.cloth_contour;
    cloth_info_back_.keypoint_info = cloth_info.keypoint_info;
  }
  emit signDetectClothResult(result, cloth_type);
}

void AutoCreateStyleWizard::slotSetExtraParam(syt_msgs::msg::ClothStyle cloth_style) {
  if (cloth_style.cloth_type == 0) {
    cloth_style_front_ = cloth_style;
  } else if (cloth_style.cloth_type == 1) {
    cloth_style_back_ = cloth_style;
  }
}

void AutoCreateStyleWizard::slotCreateStyle() {
  waiting_spinner_widget_->start();
  cloth_style_front_.cloth_contour = cloth_info_front_.cloth_contour;
  cloth_style_front_.keypoint_info = cloth_info_front_.keypoint_info;
  cloth_style_back_.cloth_contour  = cloth_info_back_.cloth_contour;
  cloth_style_back_.keypoint_info  = cloth_info_back_.keypoint_info;
  emit signCreateStyle(0, cloth_style_front_, cloth_style_back_);
}

void AutoCreateStyleWizard::slotCreateStyleResult(bool result, QString file_name) {
  waiting_spinner_widget_->stop();
  if (result) {
    file_name_ = file_name;
    qDebug() << file_name;
  }
  emit signCreateStyleResult(result);
}

void AutoCreateStyleWizard::slotSetRenameEdit() {
  emit signSetRenameEdit(file_name_);
}

void AutoCreateStyleWizard::slotRenameClothStyle() {
  waiting_spinner_widget_->start();
  emit signRenameClothStyle(field("old_name").toString(), field("new_name").toString());
}

void AutoCreateStyleWizard::slotRenameClothStyleResult(bool result) {
  waiting_spinner_widget_->stop();
  emit signRenameClothStyleResult(result);
}
