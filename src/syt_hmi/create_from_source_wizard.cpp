#include "syt_hmi/create_from_source_wizard.h"

CreateFromSourceWizard::CreateFromSourceWizard(QWidget *parent) : QWizard(parent) {
  setWindowTitle("根据已有文件创建样式文件");
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
  // 1.选择文件
  ChooseStylePage *choose_style_page = new ChooseStylePage(this);
  setPage(CHOOSE_STYLE_PAGE, choose_style_page);

  // 2.修改前片参数
  StyleDisplayPage *style_display_front_page = new StyleDisplayPage(this, 0);
  setPage(DISPLAY_FRONT_PAGE, style_display_front_page);

  // 3.修改后片参数
  StyleDisplayPage *style_display_back_page = new StyleDisplayPage(this, 1);
  setPage(DISPLAY_BACK_PAGE, style_display_back_page);

  // 4.创建样式
  CreateStylePage *create_style_page = new CreateStylePage(parent);
  setPage(CREATE_STYLE_PAGE, create_style_page);

  // 5.修改样式名
  RenameClothStylePage *rename_cloth_style_page = new RenameClothStylePage(parent);
  setPage(RENAME_STYLE_PAGE, rename_cloth_style_page);

  //////////////////// 信号处理 ////////////////////
  // 1.获取样式信息
  qRegisterMetaType<syt_msgs::msg::ClothStyle>("syt_msgs::msg::ClothStyle");
  connect(choose_style_page, &ChooseStylePage::signGetClothStyle, this, &CreateFromSourceWizard::slotGetClothStyle);
  connect(this, &CreateFromSourceWizard::signGetClothStyleResult, choose_style_page, &ChooseStylePage::slotGetClothStyleResult);

  // 2.返回修改后的参数
  connect(this, &CreateFromSourceWizard::signSetFullParam, style_display_front_page, &StyleDisplayPage::slotSetFullParam); // 前片
  connect(this, &CreateFromSourceWizard::signSetFullParam, style_display_back_page, &StyleDisplayPage::slotSetFullParam);  // 后片
  connect(style_display_front_page, &StyleDisplayPage::signSetFullParam, this, &CreateFromSourceWizard::slotSetFullParam); // 前片
  connect(style_display_back_page, &StyleDisplayPage::signSetFullParam, this, &CreateFromSourceWizard::slotSetFullParam);  // 后片

  // 3.调用创建样式服务
  connect(create_style_page, &CreateStylePage::signCreateStyle, this, &CreateFromSourceWizard::slotCreateStyle);
  connect(this, &CreateFromSourceWizard::signCreateStyleResult, create_style_page, &CreateStylePage::slotCreateStyleResult);

  // 4.重命名服务
  connect(create_style_page, &CreateStylePage::signSetRenameEdit, this, &CreateFromSourceWizard::slotSetRenameEdit);
  connect(this, &CreateFromSourceWizard::signSetRenameEdit, rename_cloth_style_page, &RenameClothStylePage::slotSetRenameEdit);
  connect(rename_cloth_style_page, &RenameClothStylePage::signRenameClothStyle, this, &CreateFromSourceWizard::slotRenameClothStyle);
  connect(this, &CreateFromSourceWizard::signRenameClothStyleResult, rename_cloth_style_page, &RenameClothStylePage::slotRenameClothStyleResult);

  // 取消按键
  QAbstractButton *cancel_btn = this->button(QWizard::CancelButton);
  connect(cancel_btn, &QPushButton::clicked, this, [=]() {
    if (!file_name_.isEmpty()) {
      QFile::remove(QString("/home/syt/style") + QDir::separator() + file_name_ + QString(".sty"));
    }
  });

  // 设置每页按钮
  connect(this, &QWizard::currentIdChanged, [=](int id) {
    switch (id) {
    case CHOOSE_STYLE_PAGE:
    case CREATE_STYLE_PAGE: {
      QList<QWizard::WizardButton> button_layout;
      button_layout << QWizard::Stretch << QWizard::NextButton << QWizard::CancelButton;
      setButtonLayout(button_layout);
      break;
    }
    case DISPLAY_FRONT_PAGE: {
      QList<QWizard::WizardButton> button_layout;
      button_layout << QWizard::Stretch << QWizard::BackButton << QWizard::NextButton << QWizard::CancelButton;
      setButtonLayout(button_layout);
      break;
    }
    case DISPLAY_BACK_PAGE: {
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

void CreateFromSourceWizard::slotGetClothStyle(QString prefix, QString style_name) {
  waiting_spinner_widget_->start();
  emit signGetClothStyle(prefix, style_name);
}

void CreateFromSourceWizard::slotGetClothStyleResult(bool result, syt_msgs::msg::ClothStyle style_front, syt_msgs::msg::ClothStyle style_back) {
  waiting_spinner_widget_->stop();
  emit signGetClothStyleResult(result);
  if (result) {
    emit signSetFullParam(style_front);
    emit signSetFullParam(style_back);
  }
}

void CreateFromSourceWizard::slotSetFullParam(syt_msgs::msg::ClothStyle cloth_style) {
  auto setClothStyle = [=](syt_msgs::msg::ClothStyle &cloth_style_to_set) {
    cloth_style_to_set.cloth_type              = cloth_style.cloth_type;
    cloth_style_to_set.cloth_color             = cloth_style.cloth_color;
    cloth_style_to_set.cloth_size              = cloth_style.cloth_size;
    cloth_style_to_set.elasticity_level        = cloth_style.elasticity_level;
    cloth_style_to_set.thickness_level         = cloth_style.thickness_level;
    cloth_style_to_set.glossiness_level        = cloth_style.glossiness_level;
    cloth_style_to_set.cloth_weight            = cloth_style.cloth_weight;
    cloth_style_to_set.cloth_length            = cloth_style.cloth_length;
    cloth_style_to_set.bottom_length           = cloth_style.bottom_length;
    cloth_style_to_set.oxter_length            = cloth_style.oxter_length;
    cloth_style_to_set.shoulder_length         = cloth_style.shoulder_length;
    cloth_style_to_set.side_length             = cloth_style.side_length;
    cloth_style_to_set.cloth_length_tolerance  = cloth_style.cloth_length_tolerance;
    cloth_style_to_set.bottom_length_tolerance = cloth_style.bottom_length_tolerance;
    cloth_style_to_set.oxter_length_tolerance  = cloth_style.oxter_length_tolerance;
    cloth_style_to_set.matching_level          = cloth_style.matching_level;
    cloth_style_to_set.have_printings          = cloth_style.have_printings;
  };

  if (cloth_style.cloth_type == 0) {
    setClothStyle(cloth_style_front_);
  } else {
    setClothStyle(cloth_style_back_);
  }
}

void CreateFromSourceWizard::slotCreateStyle(QString prefix) {
  waiting_spinner_widget_->start();
  emit signCreateStyle(1, prefix, cloth_style_front_, cloth_style_back_); // 1 表示手动创建
}

void CreateFromSourceWizard::slotCreateStyleResult(bool result, QString file_name) {
  waiting_spinner_widget_->stop();
  if (result) {
    file_name_ = file_name;
  }
  emit signCreateStyleResult(result);
}

void CreateFromSourceWizard::slotSetRenameEdit() {
  emit signSetRenameEdit(file_name_);
}

void CreateFromSourceWizard::slotRenameClothStyle() {
  waiting_spinner_widget_->start();
  emit signRenameClothStyle(field("old_name").toString(), field("new_name").toString());
}

void CreateFromSourceWizard::slotRenameClothStyleResult(bool result) {
  waiting_spinner_widget_->stop();
  if (result) {
    file_name_ = field("new_name").toString();
  }
  emit signRenameClothStyleResult(result);
}
