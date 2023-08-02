#include "syt_hmi/wizard_pages.h"

//////////////// 自动创建(向导) ////////////////
// 1.移动抓手
MoveHandPage::MoveHandPage(QWidget *parent) : QWizardPage(parent) {
  setTitle("移动抓手至安全位置");
  move_hand_button    = new InteractiveButtonBase;
  QSpacerItem *spacer = new QSpacerItem(20, 100, QSizePolicy::Expanding);
  move_hand_button->setText("点击移动抓手");
  connect(move_hand_button, &QPushButton::clicked, this, [=]() {
    // 未移动抓手时发送
    if (!moved_hand_) {
      emit signMoveHand();
    }
  });
  QVBoxLayout *layout = new QVBoxLayout;
  layout->addItem(spacer);
  layout->addWidget(move_hand_button);
  setLayout(layout);
}

bool MoveHandPage::validatePage() {
  if (moved_hand_) {
    return true;
  }
  showMessageBox(this, WARN, "请点击移动按钮并等待合片抓手移动完毕...", 1, {"确认"});
  return false;
}

void MoveHandPage::slotMoveHandResult(bool result) {
  moved_hand_ = result;
  if (result == false) {
    showMessageBox(this, WARN, "移动合片抓手失败!", 1, {"确认"});
    return;
  }
  showMessageBox(this, SUCCESS, "移动合片抓手成功，请点击下一步。", 1, {"确认"});
}

// 2.提醒放置裁片
RemindClothPage::RemindClothPage(QWidget *parent, int cloth_type) : QWizardPage(parent) {
  setTitle(QString("请放置%1模板").arg(cloth_type ? "后片" : "前片"));
  QLabel *label1 = new QLabel(QString("请将待检测的%1模板平稳放置在合片机台面下，确保放置完成后点击“下一步”").arg(cloth_type ? "后片" : "前片"));
  label1->setWordWrap(true);
  QVBoxLayout *layout = new QVBoxLayout;
  layout->addWidget(label1);
  setLayout(layout);
}

// 3.调用检测服务获取裁片轮廓和关键点
DetectClothPage::DetectClothPage(QWidget *parent, int cloth_type) : QWizardPage(parent), cloth_type_(cloth_type) {
  setTitle(QString("检测%1裁片轮廓和关键点").arg(cloth_type_ ? "后片" : "前片"));

  InteractiveButtonBase *detect_cloth_btn_ = new InteractiveButtonBase;
  QSpacerItem *spacer                      = new QSpacerItem(20, 100, QSizePolicy::Expanding);

  detect_cloth_btn_->setText(QString("点击开始检测%1").arg(cloth_type_ ? "后片" : "前片"));
  connect(detect_cloth_btn_, &QPushButton::clicked, this, [=]() {
    if (!detected_result_) {
      emit signDetectCloth(cloth_type_);
    }
  });
  QVBoxLayout *layout = new QVBoxLayout;
  layout->addItem(spacer);
  layout->addWidget(detect_cloth_btn_);
  setLayout(layout);
}

bool DetectClothPage::validatePage() {
  if (detected_result_) {
    return true;
  }
  showMessageBox(this, WARN, "请点击检测按钮并等待检测完毕...", 1, {"确认"});
  return false;
}

void DetectClothPage::slotDetectClothResult(bool result, int cloth_type) {
  if (cloth_type != cloth_type_) {
    return;
  }
  detected_result_ = result;
  if (result == false) {
    showMessageBox(this, WARN, QString("%1检测失败!").arg(cloth_type_ ? "后片" : "前片"), 1, {"确认"});
    return;
  }
  showMessageBox(this, SUCCESS, QString("%1检测成功，请点击下一步。").arg(cloth_type_ ? "后片" : "前片"), 1, {"确认"});
}

// 4.手动输入额外参数
InputExtraParamPage::InputExtraParamPage(QWidget *parent, int cloth_type) : QWizardPage(parent), cloth_type_(cloth_type) {
  setTitle(QString("输入%1额外参数").arg(cloth_type_ ? "后片" : "前片"));

  input_extra_param_widget = new InputExtraParamWidget(parent);
  input_extra_param_widget->setClothType(cloth_type_);

  QVBoxLayout *layout = new QVBoxLayout;
  layout->addWidget(input_extra_param_widget);
  setLayout(layout);
}

bool InputExtraParamPage::validatePage() {
  syt_msgs::msg::ClothStyle cloth_style = input_extra_param_widget->getClothStyle();

  emit signSetExtraParam(cloth_style);
  return true;
}

// 5.调用创建样式服务
CreateStylePage::CreateStylePage(QWidget *parent) : QWizardPage(parent) {
  setTitle("创建样式");

  InteractiveButtonBase *create_style_button = new InteractiveButtonBase;
  QSpacerItem *spacer                        = new QSpacerItem(20, 100, QSizePolicy::Expanding);

  create_style_button->setText("点击生成样式");
  connect(create_style_button, &QPushButton::clicked, this, [=]() {
    if (!create_result_) {
      emit signCreateStyle();
    }
  });
  QVBoxLayout *layout = new QVBoxLayout;
  layout->addItem(spacer);
  layout->addWidget(create_style_button);
  setLayout(layout);
}

bool CreateStylePage::validatePage() {
  if (create_result_) {
    emit signSetRenameEdit();
    return true;
  }
  showMessageBox(this, WARN, "请点击创建按钮并等待样式文件创建完毕...", 1, {"确认"});
  return false;
}

void CreateStylePage::slotCreateStyleResult(bool result) {
  create_result_ = result;
  if (result == false) {
    showMessageBox(this, WARN, "样式文件创建失败!", 1, {"确认"});
    return;
  }
  showMessageBox(this, SUCCESS, "样式文件创建成功，请点击下一步。", 1, {"确认"});
}

// 6. 修改样式名
RenameClothStylePage::RenameClothStylePage(QWidget *parent) : QWizardPage(parent) {
  setTitle("修改样式名(可选)");
  old_name_label     = new QLabel(tr("旧文件名"));
  new_name_label     = new QLabel(tr("新文件名"));
  old_name_line_edit = new QLineEdit;
  new_name_line_edit = new QLineEdit;
  rename_btn         = new InteractiveButtonBase("重命名");

  connect(rename_btn, &QPushButton::clicked, this, [=]() {
    if (old_name_line_edit->text() != new_name_line_edit->text()) {
      emit signRenameClothStyle();
    } else {
      showMessageBox(this, WARN, "如需重命名，请修改新文件名。", 1, {"确认"});
    }
  });

  registerField("old_name", old_name_line_edit);
  registerField("new_name", new_name_line_edit);

  old_name_line_edit->setEnabled(false);
  QRegExp exp("[a-zA-Z0-9_]+.syt");
  QValidator *validator = new QRegExpValidator(exp);
  new_name_line_edit->setValidator(validator);

  QGridLayout *layout = new QGridLayout;
  layout->addWidget(old_name_label, 0, 0);
  layout->addWidget(old_name_line_edit, 0, 1);
  layout->addWidget(new_name_label, 1, 0);
  layout->addWidget(new_name_line_edit, 1, 1);
  layout->addWidget(rename_btn, 2, 1);
  setLayout(layout);
}

bool RenameClothStylePage::validatePage() {
  if (old_name_line_edit->text() == new_name_line_edit->text()) {
    return true;
  }

  showMessageBox(this, WARN, "请点击重命名并等待重命名完毕...", 1, {"确认"});
  return false;
}

void RenameClothStylePage::slotSetRenameEdit(QString file_name) {
  old_name_line_edit->setText(file_name);
  new_name_line_edit->setText(file_name);
}

void RenameClothStylePage::slotRenameClothStyleResult(bool result) {
  if (result == false) {
    showMessageBox(this, WARN, "重命名失败!", 1, {"确认"});
    return;
  }
  slotSetRenameEdit(new_name_line_edit->text());
  showMessageBox(this, SUCCESS, "重命名成功，请点击完成。", 1, {"确认"});
}

// 7.选择cad文件page
ChooseCADPage::ChooseCADPage(QWidget *parent) : QWizardPage(parent) {
  setTitle("选择CAD文件");
  QLabel *label1 = new QLabel("page1.");
  label1->setWordWrap(true);
  QVBoxLayout *layout = new QVBoxLayout;
  layout->addWidget(label1);
  setLayout(layout);
}

// 8.输入长度参数
InputLengthParamPage::InputLengthParamPage(QWidget *parent, int cloth_type) : QWizardPage(parent), cloth_type_(cloth_type) {
  setTitle(QString("输入%1长度参数").arg(cloth_type_ ? "后片" : "前片"));

  input_length_param_widget = new InputLengthParamWidget(parent);

  QVBoxLayout *layout = new QVBoxLayout;
  layout->addWidget(input_length_param_widget);
  setLayout(layout);
}

bool InputLengthParamPage::validatePage() {
  if (input_length_param_widget->filled()) {
    syt_msgs::msg::ClothStyle cloth_style = input_length_param_widget->getClothStyle();
    cloth_style.cloth_type                = cloth_type_;

    emit signSetLengthParam(cloth_style);
    return true;
  }
  return false;
}
