#include "syt_hmi/cloth_style_dialog.h"
#include "ui_cloth_style_dialog.h"

ClothStyleDialog::ClothStyleDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::ClothStyleDialog) {
  ui->setupUi(this);
  this->setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
  this->setModal(true); // 设置为模态 父对象控件不可选

  ui->create_from_cad_btn->setParentEnabled(true);
  ui->create_from_cad_btn->setForeEnabled(false);
  ui->create_from_cad_btn->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");

  ui->auto_create_style_btn->setParentEnabled(true);
  ui->auto_create_style_btn->setForeEnabled(false);
  ui->auto_create_style_btn->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");

  ui->manual_input_param_btn->setParentEnabled(true);
  ui->manual_input_param_btn->setForeEnabled(false);
  ui->manual_input_param_btn->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");

  ui->create_from_source_btn->setParentEnabled(true);
  ui->create_from_source_btn->setForeEnabled(false);
  ui->create_from_source_btn->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");

  connect(ui->create_from_cad_btn, &QPushButton::clicked, [=] {
    emit signCreateFromCAD(this);
  });

  connect(ui->auto_create_style_btn, &QPushButton::clicked, [=] {
    emit signAutoCreateStyle(this);
  });

  connect(ui->manual_input_param_btn, &QPushButton::clicked, [=] {
    emit signManualInputParam(this);
  });

  connect(ui->create_from_source_btn, &QPushButton::clicked, [=] {
    emit signCreateFromCAD(this);
  });

  connect(ui->min_btn, &QPushButton::clicked, [=] {
    parent->showMinimized();
  });

  connect(ui->close_btn, &QPushButton::clicked, [=] {
    this->close();
  });
}

ClothStyleDialog::~ClothStyleDialog() {
  delete ui;
}

//////////////// 从CAD创建(向导) ////////////////
CreateFromCADWizard::CreateFromCADWizard(QWidget *parent) : QWizard(parent) {
  setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
  setModal(true);
  ChooseCADPage *choose_cad_page = new ChooseCADPage(parent);
  addPage(choose_cad_page);
}

// 选择cad文件page
ChooseCADPage::ChooseCADPage(QWidget *parent) : QWizardPage(parent) {
  setTitle("选择CAD文件");
  QLabel *label1 = new QLabel("page1.");
  label1->setWordWrap(true);
  QVBoxLayout *layout = new QVBoxLayout;
  layout->addWidget(label1);
  setLayout(layout);
}

//////////////// 自动创建(向导) ////////////////
AutoCreateStyleWizard::AutoCreateStyleWizard(QWidget *parent) : QWizard(parent) {
  setWindowTitle("自动创建样式文件");
  setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
  setButtonText(WizardButton::BackButton, "上一步");
  setButtonText(WizardButton::NextButton, "下一步");
  setButtonText(WizardButton::CancelButton, "取消");
  setButtonText(WizardButton::FinishButton, "完成");
  setModal(true);

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
  qRegisterMetaType<std::string>("std::string");
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
  emit signMoveHand();
}

void AutoCreateStyleWizard::slotMoveHandResult(bool result) {
  emit signMoveHandResult(result);
}

void AutoCreateStyleWizard::slotDetectCloth(int cloth_type) {
  emit signDetectCloth(cloth_type);
}

void AutoCreateStyleWizard::slotDetectClothResult(bool result, int cloth_type, syt_msgs::msg::ClothInfo cloth_info) {
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
  cloth_style_front_.cloth_contour = cloth_info_front_.cloth_contour;
  cloth_style_front_.keypoint_info = cloth_info_front_.keypoint_info;
  cloth_style_back_.keypoint_info = cloth_info_back_.keypoint_info;
  cloth_style_back_.keypoint_info = cloth_info_back_.keypoint_info;
  emit signCreateStyle(cloth_style_front_, cloth_style_back_);
}

void AutoCreateStyleWizard::slotCreateStyleResult(bool result, std::string file_name) {
  if (result) {
    file_name_ = file_name;
    qDebug() << QString(file_name.c_str());
  }
  emit signCreateStyleResult(result);
}

void AutoCreateStyleWizard::slotSetRenameEdit() {
  emit signSetRenameEdit(QString(file_name_.c_str()));
}

void AutoCreateStyleWizard::slotRenameClothStyle() {
  emit signRenameClothStyle(field("old_name").toString().toStdString(), field("new_name").toString().toStdString());
}

void AutoCreateStyleWizard::slotRenameClothStyleResult(bool result) {
  emit signRenameClothStyleResult(result);
}

// 1.移动抓手
MoveHandPage::MoveHandPage(QWidget *parent) : QWizardPage(parent) {
  setTitle("移动抓手至安全位置");
  move_hand_button = new InteractiveButtonBase;
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
  QMessageBox::warning(this, "提示", "请点击移动按钮并等待合片抓手移动完毕...", "确认");
  return false;
}

void MoveHandPage::slotMoveHandResult(bool result) {
  moved_hand_ = result;
  if (result == false) {
    QMessageBox::warning(this, "警告", "移动合片抓手失败!", "确认");
    return;
  }
  QMessageBox::information(this, "提示", "移动合片抓手成功，请点击下一步.", "确认");
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
  QSpacerItem *spacer = new QSpacerItem(20, 100, QSizePolicy::Expanding);

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
  QMessageBox::warning(this, "提示", "请点击检测按钮并等待检测完毕...", "确认");
  return false;
}

void DetectClothPage::slotDetectClothResult(bool result, int cloth_type) {
  if (cloth_type != cloth_type_) {
    return;
  }
  detected_result_ = result;
  if (result == false) {
    QMessageBox::warning(this, "警告", QString("%1检测失败!").arg(cloth_type_ ? "后片" : "前片"), "确认");
    return;
  }
  QMessageBox::information(this, "提示", QString("%1检测成功，请点击下一步.").arg(cloth_type_ ? "后片" : "前片"), "确认");
}

// 4.手动输入额外参数
InputExtraParamPage::InputExtraParamPage(QWidget *parent, int cloth_type) : QWizardPage(parent), cloth_type_(cloth_type) {
  setTitle(QString("输入额外参数%1").arg(cloth_type_ ? "后片" : "前片"));

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
  QSpacerItem *spacer = new QSpacerItem(20, 100, QSizePolicy::Expanding);

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
  QMessageBox::warning(this, "提示", "请点击创建按钮并等待样式文件创建完毕...", "确认");
  return false;
}

void CreateStylePage::slotCreateStyleResult(bool result) {
  create_result_ = result;
  if (result == false) {
    QMessageBox::warning(this, "警告", "样式文件创建失败!", "确认");
    return;
  }
  QMessageBox::information(this, "提示", "样式文件创建成功，请点击下一步.", "确认");
}

// 6. 修改样式名
RenameClothStylePage::RenameClothStylePage(QWidget *parent) : QWizardPage(parent) {
  setTitle("修改样式名(可选)");
  old_name_label = new QLabel(tr("旧文件名"));
  new_name_label = new QLabel(tr("新文件名"));
  old_name_line_edit = new QLineEdit;
  new_name_line_edit = new QLineEdit;
  rename_btn = new InteractiveButtonBase("重命名");

  connect(rename_btn, &QPushButton::clicked, this, [=]() {
    if (old_name_line_edit->text() != new_name_line_edit->text()) {
      emit signRenameClothStyle();
    } else {
      QMessageBox::information(this, "提示", "如需重命名，请修改新文件名", "确认");
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

  if (rename_result_) {
    return true;
  }
  QMessageBox::warning(this, "提示", "请点击重命名并等待重命名完毕...", "确认");
  return false;
}

void RenameClothStylePage::slotSetRenameEdit(QString file_name) {
  old_name_line_edit->setText(file_name);
  new_name_line_edit->setText(file_name);
}

void RenameClothStylePage::slotRenameClothStyleResult(bool result) {
  rename_result_ = result;
  if (result == false) {
    QMessageBox::warning(this, "警告", "重命名失败!", "确认");
    return;
  }
  QMessageBox::information(this, "提示", "重命名成功，请点击完成.", "确认");
}

//////////////// 手动创建(向导) ////////////////
ManualInputParamWizard::ManualInputParamWizard(QWidget *parent) : QWizard(parent) {
  setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
  setModal(true);
}

//////////////// 从已有文件创建(向导) ////////////////
CreateFromSourceWizard::CreateFromSourceWizard(QWidget *parent) : QWizard(parent) {
  setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
  setModal(true);
}
