#include "syt_hmi/manual_input_param_wizard.h"

ManualInputParamWizard::ManualInputParamWizard(QWidget *parent) : QWizard(parent) {
  setWindowTitle("自动创建样式文件");
  setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
  setButtonText(WizardButton::BackButton, "上一步");
  setButtonText(WizardButton::NextButton, "下一步");
  setButtonText(WizardButton::CancelButton, "取消");
  setButtonText(WizardButton::FinishButton, "完成");
  setButtonText(WizardButton::CommitButton, "提交");
  setModal(true);

  // ----- wizard流程 -----
  // 1.1手动输入前片长度参数
  InputLengthParamPage *input_length_param_front_page = new InputLengthParamPage(parent, 0);
  addPage(input_length_param_front_page);

  // 1.2手动输入前片额外参数
  InputExtraParamPage *input_extra_param_front_page = new InputExtraParamPage(parent, 0);
  addPage(input_extra_param_front_page);

  // 2.1手动输入后片长度参数
  InputLengthParamPage *input_length_param_back_page = new InputLengthParamPage(parent, 1);
  addPage(input_length_param_back_page);

  // 2.2手动输入后片额外参数
  InputExtraParamPage *input_extra_param_back_page = new InputExtraParamPage(parent, 1);
  addPage(input_extra_param_back_page);

  // 3.调用样式服务
  CreateStylePage *create_style_page = new CreateStylePage(parent);
  addPage(create_style_page);

  // 4.修改样式名
  RenameClothStylePage *rename_cloth_style_page = new RenameClothStylePage(parent);
  addPage(rename_cloth_style_page);

  //////////////////// 信号处理 ////////////////////
  // 1.输入长度参数
  qRegisterMetaType<syt_msgs::msg::ClothStyle>("syt_msgs::msg::ClothStyle");
  connect(input_length_param_front_page, &InputLengthParamPage::signSetLengthParam, this, &ManualInputParamWizard::slotSetLengthParam); // 前片
  connect(input_length_param_back_page, &InputLengthParamPage::signSetLengthParam, this, &ManualInputParamWizard::slotSetLengthParam);  // 后片

  // 2.输入额外参数
  connect(input_extra_param_front_page, &InputExtraParamPage::signSetExtraParam, this, &ManualInputParamWizard::slotSetExtraParam); // 前片
  connect(input_extra_param_back_page, &InputExtraParamPage::signSetExtraParam, this, &ManualInputParamWizard::slotSetExtraParam);  // 后片

  // 3.调用创建样式服务
  connect(create_style_page, &CreateStylePage::signCreateStyle, this, &ManualInputParamWizard::slotCreateStyle);
  connect(this, &ManualInputParamWizard::signCreateStyleResult, create_style_page, &CreateStylePage::slotCreateStyleResult);

  // 4.重命名服务
  connect(create_style_page, &CreateStylePage::signSetRenameEdit, this, &ManualInputParamWizard::slotSetRenameEdit);
  connect(this, &ManualInputParamWizard::signSetRenameEdit, rename_cloth_style_page, &RenameClothStylePage::slotSetRenameEdit);
  connect(rename_cloth_style_page, &RenameClothStylePage::signRenameClothStyle, this, &ManualInputParamWizard::slotRenameClothStyle);
  connect(this, &ManualInputParamWizard::signRenameClothStyleResult, rename_cloth_style_page, &RenameClothStylePage::slotRenameClothStyleResult);

  // 取消按键
  QAbstractButton *cancel_btn = this->button(QWizard::CancelButton);
  connect(cancel_btn, &QPushButton::clicked, this, [=]() {
    if (!file_name_.isEmpty()) {
      QFile::remove(QString("/home/syt/style") + QDir::separator() + file_name_ + QString(".sty"));
    }
  });
}

void ManualInputParamWizard::slotSetLengthParam(syt_msgs::msg::ClothStyle cloth_style) {
  auto setValue = [=](syt_msgs::msg::ClothStyle &chosen_cloth_style) {
    chosen_cloth_style.cloth_length    = cloth_style.cloth_length;
    chosen_cloth_style.bottom_length   = cloth_style.bottom_length;
    chosen_cloth_style.oxter_length    = cloth_style.oxter_length;
    chosen_cloth_style.shoulder_length = cloth_style.shoulder_length;
    chosen_cloth_style.side_length     = cloth_style.side_length;
  };

  if (cloth_style.cloth_type == 0) {
    setValue(cloth_style_front_);
  } else if (cloth_style.cloth_type == 1) {
    setValue(cloth_style_back_);
  }
}

void ManualInputParamWizard::slotSetExtraParam(syt_msgs::msg::ClothStyle cloth_style) {
  auto setValue = [=](syt_msgs::msg::ClothStyle &chosen_cloth_style) {
    chosen_cloth_style.cloth_type       = cloth_style.cloth_type;
    chosen_cloth_style.elasticity_level = cloth_style.elasticity_level;
    chosen_cloth_style.thickness_level  = cloth_style.thickness_level;
    chosen_cloth_style.cloth_color      = cloth_style.cloth_color;
    chosen_cloth_style.have_printings   = cloth_style.have_printings;
    chosen_cloth_style.cloth_size       = cloth_style.cloth_size;
    chosen_cloth_style.glossiness_level = cloth_style.glossiness_level;
    chosen_cloth_style.cloth_weight     = cloth_style.cloth_weight;
  };
  if (cloth_style.cloth_type == 0) {
    setValue(cloth_style_front_);
  } else if (cloth_style.cloth_type == 1) {
    setValue(cloth_style_back_);
  }
}

void ManualInputParamWizard::slotCreateStyle() {
  emit signCreateStyle(1, cloth_style_front_, cloth_style_back_);
}

void ManualInputParamWizard::slotCreateStyleResult(bool result, QString file_name) {
  if (result) {
    file_name_ = file_name;
    qDebug() << file_name;
  }
  emit signCreateStyleResult(result);
}

void ManualInputParamWizard::slotSetRenameEdit() {
  emit signSetRenameEdit(file_name_);
}

void ManualInputParamWizard::slotRenameClothStyle() {
  emit signRenameClothStyle(field("old_name").toString(), field("new_name").toString());
}

void ManualInputParamWizard::slotRenameClothStyleResult(bool result) {
  emit signRenameClothStyleResult(result);
}
