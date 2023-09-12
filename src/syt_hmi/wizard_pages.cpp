#include "syt_hmi/wizard_pages.h"

// 1.移动抓手
MoveHandPage::MoveHandPage(QWidget *parent) : QWizardPage(parent) {
  setTitle("请确认台面上无人员后，移动抓手至检测位置");
  InteractiveButtonBase *move_hand_btn = new InteractiveButtonBase("点击移动抓手", this);
  move_hand_btn->setParentEnabled(true);
  move_hand_btn->setForeEnabled(false);
  move_hand_btn->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");
  move_hand_btn->setFixedSize(250, 170);
  move_hand_btn->setFontSize(14);
  QSpacerItem *spacer_left = new QSpacerItem(200, 20, QSizePolicy::Preferred);
  QSpacerItem *spacer_right = new QSpacerItem(200, 20, QSizePolicy::Preferred);
  QHBoxLayout *btn_layout = new QHBoxLayout;
  btn_layout->addItem(spacer_left);
  btn_layout->addWidget(move_hand_btn);
  btn_layout->addItem(spacer_right);

  connect(move_hand_btn, &QPushButton::clicked, this, [=]() {
    // 未移动抓手时发送
    if (!moved_hand_) {
      emit signMoveHand();
    } else {
      showMessageBox(this, SUCCESS, "抓手已移动成功，请点击下一步。", 1, {"确认"});
    }
  });

  QSpacerItem *spacer_up = new QSpacerItem(20, 200, QSizePolicy::Preferred);
  QSpacerItem *spacer_down = new QSpacerItem(20, 200, QSizePolicy::Preferred);
  QVBoxLayout *layout = new QVBoxLayout;
  layout->addItem(spacer_up);
  layout->addLayout(btn_layout);
  layout->addItem(spacer_down);
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
    showMessageBox(this, ERROR, "移动合片抓手失败!", 1, {"确认"});
    return;
  }
  showMessageBox(this, SUCCESS, "移动合片抓手成功，请点击下一步。", 1, {"确认"});
}

// 2.调用检测服务获取裁片轮廓和关键点
DetectClothPage::DetectClothPage(QWidget *parent, int cloth_type) : QWizardPage(parent), cloth_type_(cloth_type) {
  setTitle(QString("确保放置%1裁片平整后点击按钮开始检测").arg(cloth_type_ ? "后片" : "前片"));

  InteractiveButtonBase *detect_cloth_btn = new InteractiveButtonBase;
  detect_cloth_btn->setParentEnabled(true);
  detect_cloth_btn->setForeEnabled(false);
  detect_cloth_btn->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");
  detect_cloth_btn->setFixedSize(250, 170);
  detect_cloth_btn->setFontSize(14);
  QSpacerItem *spacer_left = new QSpacerItem(200, 20, QSizePolicy::Preferred);
  QSpacerItem *spacer_right = new QSpacerItem(200, 20, QSizePolicy::Preferred);
  QHBoxLayout *btn_layout = new QHBoxLayout;
  btn_layout->addItem(spacer_left);
  btn_layout->addWidget(detect_cloth_btn);
  btn_layout->addItem(spacer_right);

  detect_cloth_btn->setText(QString("点击开始检测%1").arg(cloth_type_ ? "后片" : "前片"));
  connect(detect_cloth_btn, &QPushButton::clicked, this, [=]() {
    emit signDetectCloth(cloth_type_);
  });

  QSpacerItem *spacer_up = new QSpacerItem(20, 200, QSizePolicy::Preferred);
  QSpacerItem *spacer_down = new QSpacerItem(20, 200, QSizePolicy::Preferred);
  QVBoxLayout *layout = new QVBoxLayout(this);
  layout->addItem(spacer_up);
  layout->addLayout(btn_layout);
  layout->addItem(spacer_down);
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
    showMessageBox(this, ERROR, QString("%1检测失败!").arg(cloth_type_ ? "后片" : "前片"), 1, {"确认"});
    return;
  }
  showMessageBox(this, SUCCESS, QString("%1检测成功，请点击下一步。").arg(cloth_type_ ? "后片" : "前片"), 1, {"确认"});
}

// 3.手动输入额外参数
InputExtraParamPage::InputExtraParamPage(QWidget *parent, int cloth_type) : QWizardPage(parent), cloth_type_(cloth_type) {
  setTitle(QString("输入%1额外参数").arg(cloth_type_ ? "后片" : "前片"));

  input_extra_param_widget_ = new InputExtraParamWidget(parent);
  input_extra_param_widget_->setClothType(cloth_type_);

  QSpacerItem *spacer_up = new QSpacerItem(20, 50, QSizePolicy::Preferred);
  QSpacerItem *spacer_down = new QSpacerItem(20, 50, QSizePolicy::Preferred);
  QVBoxLayout *layout = new QVBoxLayout;
  layout->addItem(spacer_up);
  layout->addWidget(input_extra_param_widget_);
  layout->addItem(spacer_down);
  setLayout(layout);
}

bool InputExtraParamPage::validatePage() {
  syt_msgs::msg::ClothStyle cloth_style = input_extra_param_widget_->getClothStyle();
  cloth_style.cloth_type = cloth_type_;
  emit signSetExtraParam(cloth_style);
  return true;
}

// 4.调用创建样式服务
CreateStylePage::CreateStylePage(QWidget *parent) : QWizardPage(parent) {
  setTitle("创建样式");

  // QHBoxLayout *h_layout            = new QHBoxLayout;
  // QSpacerItem *spacer_left         = new QSpacerItem(40, 10, QSizePolicy::Expanding);
  // QCheckBox *save_to_usb_check_box = new QCheckBox(this);
  // QSpacerItem *spacer_right        = new QSpacerItem(40, 10, QSizePolicy::Expanding);

  // save_to_usb_check_box->setText("存入U盘");
  // h_layout->addItem(spacer_left);
  // h_layout->addWidget(save_to_usb_check_box);
  // h_layout->addItem(spacer_right);

  // connect(save_to_usb_check_box, &QCheckBox::toggled, [=]() {
  //// TODO: 检测U盘，多于一个的时候提示拔出剩余U盘
  //});

  // 生成样式
  InteractiveButtonBase *create_style_btn = new InteractiveButtonBase;
  create_style_btn->setParentEnabled(true);
  create_style_btn->setForeEnabled(false);
  create_style_btn->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");
  create_style_btn->setFixedSize(250, 170);
  create_style_btn->setFontSize(14);
  QSpacerItem *spacer_left = new QSpacerItem(200, 20, QSizePolicy::Preferred);
  QSpacerItem *spacer_right = new QSpacerItem(200, 20, QSizePolicy::Preferred);
  QHBoxLayout *btn_layout = new QHBoxLayout;
  btn_layout->addItem(spacer_left);
  btn_layout->addWidget(create_style_btn);
  btn_layout->addItem(spacer_right);

  create_style_btn->setText("点击生成样式");
  connect(create_style_btn, &QPushButton::clicked, this, [=]() {
    if (!create_result_) {
      emit signCreateStyle(prefix_);
    } else {
      showMessageBox(this, SUCCESS, "样式已创建成功，请点击下一步。", 1, {"确认"});
    }
  });

  QSpacerItem *spacer_up = new QSpacerItem(20, 200, QSizePolicy::Preferred);
  QSpacerItem *spacer_down = new QSpacerItem(20, 200, QSizePolicy::Preferred);
  QGridLayout *layout = new QGridLayout;
  layout->addItem(spacer_up, 0, 0);
  // layout->addLayout(h_layout, 1, 0);
  layout->addLayout(btn_layout, 1, 0);
  layout->addItem(spacer_down, 2, 0);
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
    showMessageBox(this, ERROR, "样式文件创建失败!", 1, {"确认"});
    return;
  }
  showMessageBox(this, SUCCESS, "样式文件创建成功，请点击下一步。", 1, {"确认"});
}

// 5. 修改样式名
RenameClothStylePage::RenameClothStylePage(QWidget *parent) : QWizardPage(parent) {
  setTitle("修改样式名(可不修改)");
  QLabel *old_name_label = new QLabel(tr("旧文件名"), this);
  QLabel *new_name_label_ = new QLabel(tr("新文件名"), this);
  QLineEdit *old_name_line_edit = new QLineEdit(this);
  QLineEdit *new_name_line_edit = new QLineEdit(this);
  InteractiveButtonBase *rename_btn = new InteractiveButtonBase("重命名", this);
  rename_btn->setParentEnabled(true);
  rename_btn->setForeEnabled(false);
  rename_btn->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");
  rename_btn->setFixedSize(250, 70);
  rename_btn->setFontSize(14);
  QSpacerItem *spacer_left = new QSpacerItem(200, 20, QSizePolicy::Preferred);
  QSpacerItem *spacer_right = new QSpacerItem(200, 20, QSizePolicy::Preferred);
  QHBoxLayout *btn_layout = new QHBoxLayout;
  btn_layout->addItem(spacer_left);
  btn_layout->addWidget(rename_btn);
  btn_layout->addItem(spacer_right);

  connect(rename_btn, &QPushButton::clicked, this, [=]() {
    if (old_name_line_edit->text() != new_name_line_edit->text()) {
      QString file_path = QDir::homePath() + QDir::separator() + QString("style") + QDir::separator() + new_name_line_edit->text() + ".sty";
      QFileInfo new_file(file_path);
      if (new_file.exists()) {
        showMessageBox(this, WARN, "该文件已存在，请修改文件名。", 1, {"确认"});
      } else {
        emit signRenameClothStyle();
      }
    } else {
      showMessageBox(this, WARN, "如需重命名，请修改新文件名。", 1, {"确认"});
    }
  });

  registerField("old_name", old_name_line_edit);
  registerField("new_name", new_name_line_edit);

  old_name_line_edit->setEnabled(false);
  QRegExp exp("[a-zA-Z0-9_.]+");
  QValidator *validator = new QRegExpValidator(exp, this);
  new_name_line_edit->setValidator(validator);

  QSpacerItem *spacer_up = new QSpacerItem(20, 200, QSizePolicy::Preferred);
  QSpacerItem *spacer_down = new QSpacerItem(20, 200, QSizePolicy::Preferred);
  QGridLayout *layout = new QGridLayout;
  layout->addItem(spacer_up, 0, 0);
  layout->addWidget(old_name_label, 1, 0);
  layout->addWidget(old_name_line_edit, 1, 1);
  layout->addWidget(new_name_label_, 2, 0);
  layout->addWidget(new_name_line_edit, 2, 1);
  layout->addLayout(btn_layout, 3, 1);
  layout->addItem(spacer_down, 4, 0);
  setLayout(layout);
}

bool RenameClothStylePage::validatePage() {
  if (field("old_name").toString() == field("new_name").toString()) {
    return true;
  }

  showMessageBox(this, WARN, "请点击重命名并等待重命名完毕...", 1, {"确认"});
  return false;
}

void RenameClothStylePage::slotSetRenameEdit(QString file_name) {
  setField("old_name", file_name);
  setField("new_name", file_name);
}

void RenameClothStylePage::slotRenameClothStyleResult(bool result) {
  if (result == false) {
    showMessageBox(this, ERROR, "重命名失败!", 1, {"确认"});
    return;
  }
  slotSetRenameEdit(field("new_name").toString());
  showMessageBox(this, SUCCESS, "重命名成功，请点击完成。", 1, {"确认"});
}

// 6.选择cad文件page
ChooseCADPage::ChooseCADPage(QWidget *parent) : QWizardPage(parent) {
  setTitle("选择CAD文件");
  QLabel *label1 = new QLabel("page1.");
  label1->setWordWrap(true);

  QSpacerItem *spacer_up = new QSpacerItem(20, 200, QSizePolicy::Preferred);
  QSpacerItem *spacer_down = new QSpacerItem(20, 200, QSizePolicy::Preferred);
  QVBoxLayout *layout = new QVBoxLayout;
  layout->addItem(spacer_up);
  layout->addWidget(label1);
  layout->addItem(spacer_down);
  setLayout(layout);
}

// 7.输入长度参数
InputLengthParamPage::InputLengthParamPage(QWidget *parent, int cloth_type) : QWizardPage(parent), cloth_type_(cloth_type) {
  setTitle(QString("输入%1长度参数(单位毫米)").arg(cloth_type_ ? "后片" : "前片"));

  input_length_param_widget_ = new InputLengthParamWidget(parent);

  QSpacerItem *spacer_up = new QSpacerItem(20, 50, QSizePolicy::Preferred);
  QSpacerItem *spacer_down = new QSpacerItem(20, 50, QSizePolicy::Preferred);
  QVBoxLayout *layout = new QVBoxLayout;
  layout->addItem(spacer_up);
  layout->addWidget(input_length_param_widget_);
  layout->addItem(spacer_down);
  setLayout(layout);
}

bool InputLengthParamPage::validatePage() {
  if (input_length_param_widget_->filled()) {
    syt_msgs::msg::ClothStyle cloth_style = input_length_param_widget_->getClothStyle();
    cloth_style.cloth_type = cloth_type_;
    emit signSetLengthParam(cloth_style);
    return true;
  }
  return false;
}

// 8.输入误差参数
InputToleranceParamPage::InputToleranceParamPage(QWidget *parent) : QWizardPage(parent) {
  setTitle(QString("输入误差参数(单位毫米)"));
  setSubTitle("提交前请检查参数无误，提交后将不可修改。");
  setCommitPage(true);

  input_tolerance_param_widget_ = new InputToleranceParamWidget(parent);
  QSpacerItem *spacer_up = new QSpacerItem(20, 10, QSizePolicy::Preferred);
  QSpacerItem *spacer_down = new QSpacerItem(20, 10, QSizePolicy::Preferred);
  QVBoxLayout *layout = new QVBoxLayout;
  layout->addItem(spacer_up);
  layout->addWidget(input_tolerance_param_widget_);
  layout->addItem(spacer_down);
  setLayout(layout);
}

bool InputToleranceParamPage::validatePage() {
  syt_msgs::msg::ClothStyle cloth_style = input_tolerance_param_widget_->getClothStyle();
  emit signSetToleranceParam(cloth_style);
  return true;
}

// 9.显示所有参数
StyleDisplayPage::StyleDisplayPage(QWidget *parent, int cloth_type) : QWizardPage(parent), cloth_type_(cloth_type) {
  setTitle(QString("修改%1样式参数(长度类单位为毫米)").arg(cloth_type_ ? "后片" : "前片"));
  if (cloth_type_) {
    setCommitPage(true);
  }

  style_display_widget_ = new StyleDisplayWidget(parent);
  QSpacerItem *spacer_up = new QSpacerItem(20, 5, QSizePolicy::Preferred);
  QSpacerItem *spacer_down = new QSpacerItem(20, 5, QSizePolicy::Preferred);
  QVBoxLayout *layout = new QVBoxLayout;
  layout->addItem(spacer_up);
  layout->addWidget(style_display_widget_);
  layout->addItem(spacer_down);
  setLayout(layout);
}

bool StyleDisplayPage::validatePage() {
  syt_msgs::msg::ClothStyle cloth_style = style_display_widget_->getClothStyle();
  cloth_style.cloth_type = cloth_type_;
  emit signSetWizardFullParam(cloth_style);
  return true;
}

void StyleDisplayPage::slotSetPageFullParam(syt_msgs::msg::ClothStyle cloth_style) {
  if (cloth_type_ == cloth_style.cloth_type) {
    style_display_widget_->setClothStyle(cloth_style);
  }
}

// 10.选择样式文件
ChooseStylePage::ChooseStylePage(QWidget *parent) : QWizardPage(parent) {
  setTitle(QString("选择样式文件"));

  // 样式文件选项
  QComboBox *file_combo_box = new QComboBox(this);
  QDir style_dir(QDir::homePath() + QDir::separator() + "style");
  if (style_dir.exists()) {
    QFileInfoList file_info_list = style_dir.entryInfoList(QStringList() << "*.sty");
    for (auto file_info : file_info_list) {
      file_combo_box->addItem(file_info.fileName());
    }
  }

  void (QComboBox::*index_change_signal)(int index) = &QComboBox::currentIndexChanged;
  connect(file_combo_box, index_change_signal, [=]() {
    get_style_ = false;
  });

  QSpacerItem *spacer_left_com = new QSpacerItem(80, 20, QSizePolicy::Preferred);
  QSpacerItem *spacer_right_com = new QSpacerItem(80, 20, QSizePolicy::Preferred);
  QHBoxLayout *com_layout = new QHBoxLayout;
  com_layout->addItem(spacer_left_com);
  com_layout->addWidget(file_combo_box);
  com_layout->addItem(spacer_right_com);

  InteractiveButtonBase *get_style_btn = new InteractiveButtonBase("获取样式信息", this);
  get_style_btn->setParentEnabled(true);
  get_style_btn->setForeEnabled(false);
  get_style_btn->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");
  get_style_btn->setFixedSize(250, 70);
  get_style_btn->setFontSize(14);
  QSpacerItem *spacer_left = new QSpacerItem(200, 20, QSizePolicy::Preferred);
  QSpacerItem *spacer_right = new QSpacerItem(200, 20, QSizePolicy::Preferred);
  QHBoxLayout *btn_layout = new QHBoxLayout;
  btn_layout->addItem(spacer_left);
  btn_layout->addWidget(get_style_btn);
  btn_layout->addItem(spacer_right);

  connect(get_style_btn, &QPushButton::clicked, [=]() {
    emit signGetClothStyle(QDir::homePath() + QDir::separator() + "style", file_combo_box->currentText());
  });

  QSpacerItem *spacer_up = new QSpacerItem(20, 200, QSizePolicy::Preferred);
  QSpacerItem *spacer_down = new QSpacerItem(20, 200, QSizePolicy::Preferred);
  QVBoxLayout *layout = new QVBoxLayout;
  layout->addItem(spacer_up);
  layout->addLayout(com_layout);
  layout->addLayout(btn_layout);
  layout->addItem(spacer_down);
  setLayout(layout);
}

bool ChooseStylePage::validatePage() {
  if (get_style_) {
    return true;
  }
  showMessageBox(this, WARN, "请点击获取按钮并等待获取样式信息完毕...", 1, {"确认"});
  return false;
}

void ChooseStylePage::slotGetClothStyleResult(bool result) {
  get_style_ = result;
  if (result == false) {
    showMessageBox(this, ERROR, "获取样式信息失败!", 1, {"确认"});
    return;
  }
  showMessageBox(this, SUCCESS, "获取样式信息成功，请点击下一步。", 1, {"确认"});
}
