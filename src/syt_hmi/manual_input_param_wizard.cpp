#include "syt_hmi/manual_input_param_wizard.h"

ManualInputParamWizard::ManualInputParamWizard(QWidget *parent) : QWizard(parent) {
  setWindowTitle("手动创建样式文件");
  setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
  setButtonText(WizardButton::BackButton, tr("上一步"));
  setButtonText(WizardButton::NextButton, tr("下一步"));
  setButtonText(WizardButton::CancelButton, tr("取消"));
  setButtonText(WizardButton::FinishButton, tr("完成"));
  setButtonText(WizardButton::CommitButton, tr("提交"));
  setFixedSize(700, 600);
  setModal(true);

  waiting_spinner_widget_ = new WaitingSpinnerWidget(this);

  // ----- wizard流程 -----
  // 1.1手动输入前片长度参数
  InputLengthParamPage *input_length_param_front_page = new InputLengthParamPage(parent, 0);
  setPage(LENGTH_FRONT_PAGE, input_length_param_front_page);

  // 1.2手动输入前片额外参数
  InputExtraParamPage *input_extra_param_front_page = new InputExtraParamPage(parent, 0);
  setPage(EXTRA_FRONT_PAGE, input_extra_param_front_page);

  // 2.1手动输入后片长度参数
  InputLengthParamPage *input_length_param_back_page = new InputLengthParamPage(parent, 1);
  setPage(LENGTH_BACK_PAGE, input_length_param_back_page);

  // 2.2手动输入后片额外参数
  InputExtraParamPage *input_extra_param_back_page = new InputExtraParamPage(parent, 1);
  setPage(EXTRA_BACK_PAGE, input_extra_param_back_page);

  // 3.输入误差参数
  InputToleranceParamPage *input_tolerance_param_page = new InputToleranceParamPage(parent);
  setPage(TOLERANCE_PAGE, input_tolerance_param_page);

  // 3.创建样式
  CreateStylePage *create_style_page = new CreateStylePage(parent);
  setPage(CREATE_STYLE_PAGE, create_style_page);

  // 4.修改样式名
  RenameClothStylePage *rename_cloth_style_page = new RenameClothStylePage(parent);
  setPage(RENAME_STYLE_PAGE, rename_cloth_style_page);

  //////////////////// 信号处理 ////////////////////
  // 1.输入长度参数
  qRegisterMetaType<syt_msgs::msg::ClothStyle>("syt_msgs::msg::ClothStyle");
  connect(input_length_param_front_page, &InputLengthParamPage::signSetLengthParam, this, &ManualInputParamWizard::slotSetLengthParam); // 前片
  connect(input_length_param_back_page, &InputLengthParamPage::signSetLengthParam, this, &ManualInputParamWizard::slotSetLengthParam);  // 后片

  // 2.输入额外参数
  connect(input_extra_param_front_page, &InputExtraParamPage::signSetExtraParam, this, &ManualInputParamWizard::slotSetExtraParam); // 前片
  connect(input_extra_param_back_page, &InputExtraParamPage::signSetExtraParam, this, &ManualInputParamWizard::slotSetExtraParam);  // 后片

  // 3.输入误差参数
  connect(input_tolerance_param_page, &InputToleranceParamPage::signSetToleranceParam, this, &ManualInputParamWizard::slotSetToleranceParam);

  // 4.调用创建样式服务
  connect(create_style_page, &CreateStylePage::signCreateStyle, this, &ManualInputParamWizard::slotCreateStyle);
  connect(this, &ManualInputParamWizard::signCreateStyleResult, create_style_page, &CreateStylePage::slotCreateStyleResult);

  // 5.重命名服务
  connect(create_style_page, &CreateStylePage::signSetRenameEdit, this, &ManualInputParamWizard::slotSetRenameEdit);
  connect(this, &ManualInputParamWizard::signSetRenameEdit, rename_cloth_style_page, &RenameClothStylePage::slotSetRenameEdit);
  connect(rename_cloth_style_page, &RenameClothStylePage::signRenameClothStyle, this, &ManualInputParamWizard::slotRenameClothStyle);
  connect(this, &ManualInputParamWizard::signRenameClothStyleResult, rename_cloth_style_page, &RenameClothStylePage::slotRenameClothStyleResult);

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
    case LENGTH_FRONT_PAGE:
    case CREATE_STYLE_PAGE: {
      QList<QWizard::WizardButton> button_layout;
      button_layout << QWizard::Stretch << QWizard::NextButton << QWizard::CancelButton;
      setButtonLayout(button_layout);
      break;
    }
    case EXTRA_FRONT_PAGE:
    case LENGTH_BACK_PAGE:
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

void ManualInputParamWizard::slotSetLengthParam(syt_msgs::msg::ClothStyle cloth_style) {
  auto setValue = [=](syt_msgs::msg::ClothStyle &chosen_cloth_style) {
    chosen_cloth_style.cloth_length = cloth_style.cloth_length;
    chosen_cloth_style.bottom_length = cloth_style.bottom_length;
    chosen_cloth_style.oxter_length = cloth_style.oxter_length;
    chosen_cloth_style.shoulder_length = cloth_style.shoulder_length;
    chosen_cloth_style.side_length = cloth_style.side_length;
  };

  if (cloth_style.cloth_type == 0) {
    setValue(cloth_style_front_);
  } else if (cloth_style.cloth_type == 1) {
    setValue(cloth_style_back_);
  }
}

void ManualInputParamWizard::slotSetExtraParam(syt_msgs::msg::ClothStyle cloth_style) {
  auto setValue = [=](syt_msgs::msg::ClothStyle &chosen_cloth_style) {
    chosen_cloth_style.cloth_type = cloth_style.cloth_type;
    chosen_cloth_style.elasticity_level = cloth_style.elasticity_level;
    chosen_cloth_style.thickness_level = cloth_style.thickness_level;
    chosen_cloth_style.cloth_color = cloth_style.cloth_color;
    chosen_cloth_style.have_printings = cloth_style.have_printings;
    chosen_cloth_style.cloth_size = cloth_style.cloth_size;
    chosen_cloth_style.glossiness_level = cloth_style.glossiness_level;
    chosen_cloth_style.cloth_weight = cloth_style.cloth_weight;
  };

  if (cloth_style.cloth_type == 0) {
    setValue(cloth_style_front_);
  } else if (cloth_style.cloth_type == 1) {
    setValue(cloth_style_back_);
  }
}

void ManualInputParamWizard::slotSetToleranceParam(syt_msgs::msg::ClothStyle cloth_style) {
  cloth_style_front_.cloth_length_tolerance = cloth_style.cloth_length_tolerance;
  cloth_style_front_.bottom_length_tolerance = cloth_style.bottom_length_tolerance;
  cloth_style_front_.oxter_length_tolerance = cloth_style.oxter_length_tolerance;
  cloth_style_front_.matching_level = cloth_style.matching_level;
  cloth_style_back_.cloth_length_tolerance = cloth_style.cloth_length_tolerance;
  cloth_style_back_.bottom_length_tolerance = cloth_style.bottom_length_tolerance;
  cloth_style_back_.oxter_length_tolerance = cloth_style.oxter_length_tolerance;
  cloth_style_back_.matching_level = cloth_style.matching_level;
}

void ManualInputParamWizard::slotCreateStyle(QString prefix) {
  waiting_spinner_widget_->start();
  emit signCreateStyle(1, prefix, cloth_style_front_, cloth_style_back_); // 1 表示手动创建
}

void ManualInputParamWizard::slotCreateStyleResult(bool result, QString file_name) {
  waiting_spinner_widget_->stop();
  if (result) {
    file_name_ = file_name;
  }
  emit signCreateStyleResult(result);
}

void ManualInputParamWizard::slotSetRenameEdit() {
  emit signSetRenameEdit(file_name_);
}

void ManualInputParamWizard::slotRenameClothStyle() {
  waiting_spinner_widget_->start();
  emit signRenameClothStyle(field("old_name").toString(), field("new_name").toString());
}

void ManualInputParamWizard::slotRenameClothStyleResult(bool result) {
  waiting_spinner_widget_->stop();
  if (result) {
    file_name_ = field("new_name").toString();
  }
  emit signRenameClothStyleResult(result);
}
