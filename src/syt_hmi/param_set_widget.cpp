#include "syt_hmi/param_set_widget.h"
#include "ui_param_set_widget.h"

ParamSetWidget::ParamSetWidget(QWidget *parent) : QWidget(parent),
                                                  ui(new Ui::ParamSetWidget) {
  ui->setupUi(this);
  setWindowFlags(Qt::FramelessWindowHint | Qt::Dialog);
  setButtonFrame();
  ui->current_page_label->setText(ui->switch_mode_btn->getText());

  // 窗口操作
  connect(ui->close_btn, &QPushButton::clicked, this, &ParamSetWidget::close);

  // 切换页面
  connect(ui->switch_mode_btn, &QPushButton::clicked, this, &ParamSetWidget::switchPage, Qt::UniqueConnection);
  connect(ui->sewing_setting_btn, &QPushButton::clicked, this, &ParamSetWidget::switchPage, Qt::UniqueConnection);
  connect(ui->compose_setting_btn, &QPushButton::clicked, this, &ParamSetWidget::switchPage, Qt::UniqueConnection);
  connect(ui->load_setting_btn, &QPushButton::clicked, this, &ParamSetWidget::switchPage, Qt::UniqueConnection);

  bindSwitchMode();
  bindSewingMachine();
  bindComposeMachine();
  bindLoadMachine();
}

ParamSetWidget::~ParamSetWidget() {
  delete ui;
}

bool ParamSetWidget::event(QEvent *event) {
  if (event->type() == QEvent::LanguageChange) {
    ui->retranslateUi(this);
  }
  return QWidget::event(event);
}

void ParamSetWidget::mousePressEvent(QMouseEvent *event) {
  switch (event->button()) {
  case Qt::LeftButton:
    is_mouse_left_press_down_ = true;
    mouse_pos_ = event->globalPos() - this->frameGeometry().topLeft();
    break;
  default:
    break;
  }
}

void ParamSetWidget::mouseMoveEvent(QMouseEvent *event) {
  if (is_mouse_left_press_down_) {
    move(event->globalPos() - mouse_pos_); // 移动窗口
    event->accept();
  }
}

void ParamSetWidget::mouseReleaseEvent(QMouseEvent *event) {
  switch (event->button()) {
  case Qt::LeftButton:
    is_mouse_left_press_down_ = false;
    break;
  default:
    break;
  }
}

void ParamSetWidget::setButtonFrame() {
  auto setFrame = [&](WaterZoomButton *button) {
    button->setParentEnabled(true);
    button->setForeEnabled(false);
    button->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");
  };

  setFrame(ui->switch_mode_btn);
  setFrame(ui->sewing_setting_btn);
  setFrame(ui->compose_setting_btn);
  setFrame(ui->load_setting_btn);

  setFrame(ui->set_mode_btn);
  setFrame(ui->label_reset_btn);
  setFrame(ui->sewing_reset_btn);
  setFrame(ui->set_needle_btn);
  setFrame(ui->set_label_btn);
  setFrame(ui->sewing_thickness_btn);

  setFrame(ui->blow_height_btn);
  setFrame(ui->table_light_btn);
  setFrame(ui->compose_reset_btn);

  setFrame(ui->thickness_btn_B);
  setFrame(ui->load_reset_btn_B);
  setFrame(ui->thickness_btn_A);
  setFrame(ui->load_reset_btn_A);
}

void ParamSetWidget::switchPage() {
  if (sender() == ui->switch_mode_btn) {
    ui->current_page_label->setText(ui->switch_mode_btn->getText());
    ui->stackedWidget->setCurrentIndex(0);
  }
  if (sender() == ui->sewing_setting_btn) {
    ui->current_page_label->setText(ui->sewing_setting_btn->getText());
    ui->stackedWidget->setCurrentIndex(1);
  }
  if (sender() == ui->compose_setting_btn) {
    ui->current_page_label->setText(ui->compose_setting_btn->getText());
    ui->stackedWidget->setCurrentIndex(2);
  }
  if (sender() == ui->load_setting_btn) {
    ui->current_page_label->setText(ui->load_setting_btn->getText());
    ui->stackedWidget->setCurrentIndex(3);
  }
}

void ParamSetWidget::bindSwitchMode() {
  // 切换模式
  connect(ui->set_mode_btn, &QPushButton::clicked, [=]() {
    emit signSwitchSewingMode(ui->mode_combo_box->currentIndex());
  });
}

void ParamSetWidget::bindSewingMachine() {
  // 由于裁片厚度与合片处功能重叠 暂时将按钮隐藏
  ui->sewing_thickness_btn->hide();
  ui->sewing_thickness_spinbox->hide();

  // 缝纫复位
  connect(ui->sewing_reset_btn, &QPushButton::clicked, [=]() {
    emit signSewingMachineReset();
  });

  // 水洗标复位
  connect(ui->label_reset_btn, &QPushButton::clicked, [=]() {
    emit signSewingMachineLabelReset();
  });

  // 水洗标参数
  connect(ui->set_label_btn, &QPushButton::clicked, [=]() {
    int side = 0;
    if (ui->left_label_radio_box->isChecked()) {
      side = 0;
    } else if (ui->right_label_radio_box->isChecked()) {
      side = 1;
    }
    emit signSewingMachineLabelWidth(ui->enable_label_check_box->isChecked(), side, ui->label_width_spin_box->value(), ui->label_position_spin_box->value());
  });

  // 针距
  connect(ui->set_needle_btn, &QPushButton::clicked, [=]() {
    emit signSewingMachineNeedle(ui->line_1_spin_box->value(), ui->line_2_spin_box->value(), ui->line_3_spin_box->value(), ui->line_4_spin_box->value());
  });

  // 厚度
  connect(ui->sewing_thickness_btn, &QPushButton::clicked, [=]() {
    emit signSewingMachineThickness(ui->sewing_thickness_spinbox->value());
  });

}

void ParamSetWidget::bindComposeMachine() {
  // 复位
  connect(ui->compose_reset_btn, &QPushButton::clicked, [=]() {
    emit signComposeMachineReset();
  });

  // 吹气高度
  connect(ui->blow_height_btn, &QPushButton::clicked, [=]() {
    emit signComposeMachineBlowHeight(ui->blow_height_spin_box->value());
  });

  // 背景灯
  connect(ui->table_light_btn, &QPushButton::clicked, [=]() {
    emit signComposeMachineTableLight(ui->table_light_spin_box->value());
  });
}

void ParamSetWidget::bindLoadMachine() {
  // 复位B
  connect(ui->load_reset_btn_B, &QPushButton::clicked, [=]() {
    emit signLoadMachineReset(1);
  });

  // 厚度B
  connect(ui->thickness_btn_B, &QPushButton::clicked, [=]() {
    emit signLoadMachineThickness(0, ui->thickness_spinbox_B->value());
  });

  // 复位A
  connect(ui->load_reset_btn_A, &QPushButton::clicked, [=]() {
    emit signLoadMachineReset(0);
  });

  // 厚度A
  connect(ui->thickness_btn_A, &QPushButton::clicked, [=]() {
    emit signLoadMachineThickness(1, ui->thickness_spinbox_A->value());
  });
}
