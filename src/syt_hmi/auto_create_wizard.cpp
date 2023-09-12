#include "syt_hmi/auto_create_wizard.h"

AutoCreateStyleWizard::AutoCreateStyleWizard(QWidget *parent) : QWizard(parent) {
  setWindowTitle("自动创建样式文件");
  setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
  setButtonText(WizardButton::BackButton, "上一步");
  setButtonText(WizardButton::NextButton, "下一步");
  setButtonText(WizardButton::CancelButton, "取消");
  setButtonText(WizardButton::FinishButton, "完成");
  setButtonText(WizardButton::CommitButton, "提交");
  setFixedSize(700, 600);
  setModal(true);

  waiting_spinner_widget_ = new WaitingSpinnerWidget(this);

  // ----- wizard流程 -----
  // 1.移动抓手
  MoveHandPage *move_hand_and_notify_page = new MoveHandPage(parent);
  setPage(MOVE_HAND_PAGE, move_hand_and_notify_page);

  // 2.1前片轮廓关键点检测
  DetectClothPage *detect_cloth_front_page = new DetectClothPage(parent, 0);
  setPage(DETECT_FRONT_PAGE, detect_cloth_front_page);

  // 2.2手动输入前片额外参数
  InputExtraParamPage *input_extra_param_front_page = new InputExtraParamPage(parent, 0);
  setPage(EXTRA_FRONT_PAGE, input_extra_param_front_page);

  // 3.1前片轮廓关键点检测
  DetectClothPage *detect_cloth_back_page = new DetectClothPage(parent, 1);
  setPage(DETECT_BACK_PAGE, detect_cloth_back_page);

  // 3.2手动输入后片额外参数
  InputExtraParamPage *input_extra_param_back_page = new InputExtraParamPage(parent, 1);
  setPage(EXTRA_BACK_PAGE, input_extra_param_back_page);

  // 4.输入误差参数
  InputToleranceParamPage *input_tolerance_param_page = new InputToleranceParamPage(parent);
  setPage(TOLERANCE_PAGE, input_tolerance_param_page);

  // 5.调用样式服务
  CreateStylePage *create_style_page = new CreateStylePage(parent);
  setPage(CREATE_STYLE_PAGE, create_style_page);

  // 6.修改样式名
  RenameClothStylePage *rename_cloth_style_page = new RenameClothStylePage(parent);
  setPage(RENAME_STYLE_PAGE, rename_cloth_style_page);

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

  // 5.输入误差参数
  connect(input_tolerance_param_page, &InputToleranceParamPage::signSetToleranceParam, this, &AutoCreateStyleWizard::slotSetToleranceParam);

  // 6.调用创建样式服务
  qRegisterMetaType<syt_msgs::msg::ClothInfo>("syt_msgs::msg::ClothInfo");
  connect(create_style_page, &CreateStylePage::signCreateStyle, this, &AutoCreateStyleWizard::slotCreateStyle);
  connect(this, &AutoCreateStyleWizard::signCreateStyleResult, create_style_page, &CreateStylePage::slotCreateStyleResult);

  // 7.重命名服务
  connect(create_style_page, &CreateStylePage::signSetRenameEdit, this, &AutoCreateStyleWizard::slotSetRenameEdit);
  connect(this, &AutoCreateStyleWizard::signSetRenameEdit, rename_cloth_style_page, &RenameClothStylePage::slotSetRenameEdit);
  connect(rename_cloth_style_page, &RenameClothStylePage::signRenameClothStyle, this, &AutoCreateStyleWizard::slotRenameClothStyle);
  connect(this, &AutoCreateStyleWizard::signRenameClothStyleResult, rename_cloth_style_page, &RenameClothStylePage::slotRenameClothStyleResult);

  // 取消按键
  QAbstractButton *cancel_btn = this->button(QWizard::CancelButton);
  connect(cancel_btn, &QPushButton::clicked, this, [=]() {
    if (!file_name_.isEmpty()) {
      QFile::remove(QDir::homePath() + QDir::separator() + QString("style") + QDir::separator() + file_name_ + QString(".sty"));
    }
  });

  // 设置每页按钮
  connect(this, &QWizard::currentIdChanged, [=](int id) {
    switch (id) {
    case MOVE_HAND_PAGE:
    case CREATE_STYLE_PAGE: {
      QList<QWizard::WizardButton> button_layout;
      button_layout << QWizard::Stretch << QWizard::NextButton << QWizard::CancelButton;
      setButtonLayout(button_layout);
      break;
    }
    case DETECT_FRONT_PAGE:
    case EXTRA_FRONT_PAGE:
    case DETECT_BACK_PAGE:
    case EXTRA_BACK_PAGE: {
      QList<QWizard::WizardButton> button_layout;
      button_layout << QWizard::Stretch << QWizard::BackButton << QWizard::NextButton << QWizard::CancelButton;
      setButtonLayout(button_layout);
      break;
    }
    case TOLERANCE_PAGE: {
      QList<QWizard::WizardButton> button_layout;
      button_layout << QWizard::Stretch << QWizard::BackButton << QWizard::CommitButton << QWizard::CancelButton;
      setButtonLayout(button_layout);
      break;
    }
    case RENAME_STYLE_PAGE: {
      QList<QWizard::WizardButton> button_layout;
      button_layout << QWizard::Stretch << QWizard::FinishButton << QWizard::CancelButton;
      setButtonLayout(button_layout);
      break;
    }
    }
  });
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

void AutoCreateStyleWizard::slotSetToleranceParam(syt_msgs::msg::ClothStyle cloth_style) {
  cloth_style_front_.cloth_length_tolerance  = cloth_style.cloth_length_tolerance;
  cloth_style_front_.bottom_length_tolerance = cloth_style.bottom_length_tolerance;
  cloth_style_front_.oxter_length_tolerance  = cloth_style.oxter_length_tolerance;
  cloth_style_front_.matching_level          = cloth_style.matching_level;
  cloth_style_back_.cloth_length_tolerance   = cloth_style.cloth_length_tolerance;
  cloth_style_back_.bottom_length_tolerance  = cloth_style.bottom_length_tolerance;
  cloth_style_back_.oxter_length_tolerance   = cloth_style.oxter_length_tolerance;
  cloth_style_back_.matching_level           = cloth_style.matching_level;
}

void AutoCreateStyleWizard::slotCreateStyle(QString prefix) {
  waiting_spinner_widget_->start();
  cloth_style_front_.cloth_contour = cloth_info_front_.cloth_contour;
  cloth_style_front_.keypoint_info = cloth_info_front_.keypoint_info;
  cloth_style_back_.cloth_contour  = cloth_info_back_.cloth_contour;
  cloth_style_back_.keypoint_info  = cloth_info_back_.keypoint_info;
  emit signCreateStyle(0, prefix, cloth_style_front_, cloth_style_back_); // 0 代表自动创建模式
}

void AutoCreateStyleWizard::slotCreateStyleResult(bool result, QString file_name) {
  waiting_spinner_widget_->stop();
  if (result) {
    file_name_ = file_name;
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
  if (result) {
    file_name_ = field("new_name").toString();
  }
  emit signRenameClothStyleResult(result);
}
