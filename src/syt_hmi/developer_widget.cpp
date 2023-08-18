#include "syt_hmi/developer_widget.h"
#include "ui_developer_widget.h"

DeveloperWidget::DeveloperWidget(QWidget *parent) : QWidget(parent),
                                                    ui(new Ui::DeveloperWidget) {
  ui->setupUi(this);
  setWindowFlags(Qt::FramelessWindowHint | Qt::Dialog);
  setButtonFrame();
  ui->current_page_label->setText(ui->switch_load_btn->getText());

  // 窗口操作
  connect(ui->close_btn, &QPushButton::clicked, this, &DeveloperWidget::close);
  connect(ui->max_btn, &QPushButton::clicked, this, [=]() {
    if (this->isMaximized()) {
      this->showNormal();
    } else {
      this->showMaximized();
    }
  });

  // 切换页面
  connect(ui->switch_load_btn, &QPushButton::clicked, this, &DeveloperWidget::switchPage, Qt::UniqueConnection);
  connect(ui->switch_compose_btn, &QPushButton::clicked, this, &DeveloperWidget::switchPage, Qt::UniqueConnection);
  connect(ui->switch_sewing_btn, &QPushButton::clicked, this, &DeveloperWidget::switchPage, Qt::UniqueConnection);
  connect(ui->other_btn, &QPushButton::clicked, this, &DeveloperWidget::switchPage, Qt::UniqueConnection);

  // 上料机功能绑定
  bindLoadMachine();

  // 合片机功能
  bindComposeMachine();

  // 缝纫机功能
  bindSewingMachine();

  // 模式切换
  setChooseMode();
}

DeveloperWidget::~DeveloperWidget() {
  delete ui;
}

void DeveloperWidget::setChooseMode() {
  // 模式选择界面
  // ui->choose_mode_combo_box->insertItem(-1, "缝纫模式");
  ui->choose_mode_combo_box->insertItem(-1, "单次模式");
  ui->choose_mode_combo_box->insertItem(-1, "循环模式");
  ui->choose_mode_combo_box->insertItem(-1, "合片模式");

  void (QComboBox::*index_change_signal)(int index) = &QComboBox::currentIndexChanged;
  connect(ui->choose_mode_combo_box, index_change_signal, [=]() {
    if ("单次模式" == ui->choose_mode_combo_box->currentText()) {
      emit signChooseMode(syt_msgs::msg::FSMRunMode::LOOP_ONCE);
    }
    if ("循环模式" == ui->choose_mode_combo_box->currentText()) {
      emit signChooseMode(syt_msgs::msg::FSMRunMode::LOOP);
    }
    if ("合片模式" == ui->choose_mode_combo_box->currentText()) {
      emit signChooseMode(syt_msgs::msg::FSMRunMode::COMPOSE_CLOTH);
    }
  });
}

void DeveloperWidget::mousePressEvent(QMouseEvent *event) {
  switch (event->button()) {
  case Qt::LeftButton:
    is_mouse_left_press_down_ = true;
    mouse_pos_                = event->globalPos() - this->frameGeometry().topLeft();
    break;
  default:
    break;
  }
}

void DeveloperWidget::mouseMoveEvent(QMouseEvent *event) {
  if (is_mouse_left_press_down_) {
    move(event->globalPos() - mouse_pos_); // 移动窗口
    event->accept();
  }
}

void DeveloperWidget::mouseReleaseEvent(QMouseEvent *event) {
  switch (event->button()) {
  case Qt::LeftButton:
    is_mouse_left_press_down_ = false;
    break;
  default:
    break;
  }
}

void DeveloperWidget::setButtonFrame() {
  auto setFrame = [&](WaterZoomButton *button) {
    button->setParentEnabled(true);
    button->setForeEnabled(false);
    button->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");
  };

  setFrame(ui->switch_load_btn);
  setFrame(ui->switch_compose_btn);
  setFrame(ui->switch_sewing_btn);
  setFrame(ui->other_btn);

  // 上料机界面
  setFrame(ui->load_reset_btn_B);
  setFrame(ui->add_cloth_btn_B);
  setFrame(ui->clear_table_btn_B);
  setFrame(ui->set_cloth_size_btn_B);
  setFrame(ui->set_load_distance_btn_B);
  setFrame(ui->set_tray_gap_btn_B);
  setFrame(ui->set_offset_btn_B);
  setFrame(ui->hold_cloth_btn_B);
  setFrame(ui->grab_cloth_btn_B);
  setFrame(ui->pre_setup_btn_B);
  setFrame(ui->visual_align_btn_B);

  setFrame(ui->load_reset_btn_A);
  setFrame(ui->add_cloth_btn_A);
  setFrame(ui->clear_table_btn_A);
  setFrame(ui->set_cloth_size_btn_A);
  setFrame(ui->set_load_distance_btn_A);
  setFrame(ui->set_tray_gap_btn_A);
  setFrame(ui->set_offset_btn_A);
  setFrame(ui->hold_cloth_btn_A);
  setFrame(ui->grab_cloth_btn_A);
  setFrame(ui->pre_setup_btn_A);
  setFrame(ui->visual_align_btn_A);

  // 合片机界面
  setFrame(ui->compose_reset_btn);
  setFrame(ui->compose_stop_btn);
  setFrame(ui->wipe_fold_btn);
  setFrame(ui->extend_needle_btn);
  setFrame(ui->withdraw_needle_btn);
  setFrame(ui->blow_wind_btn);
  setFrame(ui->stop_blow_btn);
  setFrame(ui->set_current_compose_hand_btn);
  setFrame(ui->compose_move_hand_btn);
  setFrame(ui->sucker_dilate_btn);
  setFrame(ui->sucker_shrink_btn);

  // 缝纫机界面
  setFrame(ui->sewing_reset_btn);
  setFrame(ui->sewing_stop_btn);
  setFrame(ui->set_current_sewing_hand_btn);
  setFrame(ui->sewing_move_hand_btn);
  setFrame(ui->send_keypoints_btn);
}

void DeveloperWidget::switchPage() {
  if (sender() == ui->switch_load_btn) {
    ui->current_page_label->setText(ui->switch_load_btn->getText());
    ui->stackedWidget->setCurrentIndex(0);
  } else if (sender() == ui->switch_compose_btn) {
    ui->current_page_label->setText(ui->switch_compose_btn->getText());
    ui->stackedWidget->setCurrentIndex(1);
  } else if (sender() == ui->switch_sewing_btn) {
    ui->current_page_label->setText(ui->switch_sewing_btn->getText());
    ui->stackedWidget->setCurrentIndex(2);
  } else if (sender() == ui->other_btn) {
    ui->current_page_label->setText(ui->other_btn->getText());
    ui->stackedWidget->setCurrentIndex(3);
  }
}

// 绑定上料机功能
void DeveloperWidget::bindLoadMachine() {
  // 上料台B
  connect(ui->load_reset_btn_B, &QPushButton::clicked, [=]() {
    emit signLoadMachineReset(0);
  });

  connect(ui->add_cloth_btn_B, &QPushButton::clicked, [=]() {
    emit signLoadMachineAddCloth(0);
  });

  connect(ui->clear_table_btn_B, &QPushButton::clicked, [=]() {
    emit signLoadMachineClearTable(0);
  });

  connect(ui->set_cloth_size_btn_B, &QPushButton::clicked, [=]() {
    emit signLoadMachineClothSize(0, ui->cloth_width_spinbox_B->value(), ui->cloth_length_spinbox_B->value());
  });

  connect(ui->set_load_distance_btn_B, &QPushButton::clicked, [=]() {
    emit signLoadMachineLoadDistance(0, ui->load_distance_spinbox_B->value());
  });

  connect(ui->set_tray_gap_btn_B, &QPushButton::clicked, [=]() {
    emit signLoadMachineTrayGap(0, ui->tray_gap_spinbox_B->value());
  });

  connect(ui->set_offset_btn_B, &QPushButton::clicked, [=]() {
    emit signLoadMachineOffset(0, ui->offset_spinbox_B->value());
  });

  connect(ui->hold_cloth_btn_B, &QPushButton::clicked, [=]() {
    emit signLoadMachineHoldCloth(0);
  });

  connect(ui->grab_cloth_btn_B, &QPushButton::clicked, [=]() {
    emit signLoadMachineGrabCloth(0);
  });

  connect(ui->pre_setup_btn_B, &QPushButton::clicked, [=]() {
    emit signLoadMachinePreSetup(0);
  });

  connect(ui->visual_align_btn_B, &QPushButton::clicked, [=]() {
    emit signLoadMachineVisualAlign(0);
  });

  // 上料台A
  connect(ui->load_reset_btn_A, &QPushButton::clicked, [=]() {
    emit signLoadMachineReset(1);
  });

  connect(ui->add_cloth_btn_A, &QPushButton::clicked, [=]() {
    emit signLoadMachineAddCloth(1);
  });

  connect(ui->clear_table_btn_A, &QPushButton::clicked, [=]() {
    emit signLoadMachineClearTable(1);
  });

  connect(ui->set_cloth_size_btn_A, &QPushButton::clicked, [=]() {
    emit signLoadMachineClothSize(1, ui->cloth_width_spinbox_A->value(), ui->cloth_length_spinbox_A->value());
  });

  connect(ui->set_load_distance_btn_A, &QPushButton::clicked, [=]() {
    emit signLoadMachineLoadDistance(1, ui->load_distance_spinbox_A->value());
  });

  connect(ui->set_tray_gap_btn_A, &QPushButton::clicked, [=]() {
    emit signLoadMachineTrayGap(1, ui->tray_gap_spinbox_A->value());
  });

  connect(ui->set_offset_btn_A, &QPushButton::clicked, [=]() {
    emit signLoadMachineOffset(1, ui->offset_spinbox_A->value());
  });

  connect(ui->hold_cloth_btn_A, &QPushButton::clicked, [=]() {
    emit signLoadMachineHoldCloth(1);
  });

  connect(ui->grab_cloth_btn_A, &QPushButton::clicked, [=]() {
    emit signLoadMachineGrabCloth(1);
  });

  connect(ui->pre_setup_btn_A, &QPushButton::clicked, [=]() {
    emit signLoadMachinePreSetup(1);
  });

  connect(ui->visual_align_btn_A, &QPushButton::clicked, [=]() {
    emit signLoadMachineVisualAlign(0);
  });
}

// 绑定合片机功能
void DeveloperWidget::bindComposeMachine() {
  connect(ui->set_current_compose_hand_btn, &QPushButton::clicked, [=]() {
    ui->compose_hand_x_spinbox->setValue(ui->compose_hand_x_line_edit->text().toFloat());
    ui->compose_hand_y_spinbox->setValue(ui->compose_hand_y_line_edit->text().toFloat());
    ui->compose_hand_z_spinbox->setValue(ui->compose_hand_z_line_edit->text().toFloat());
    ui->compose_hand_c_spinbox->setValue(ui->compose_hand_c_line_edit->text().toFloat());
  });

  connect(ui->compose_reset_btn, &QPushButton::clicked, [=]() {
    emit signComposeMachineReset();
  });

  connect(ui->compose_stop_btn, &QPushButton::clicked, [=]() {
    emit signComposeMachineStop();
  });

  connect(ui->wipe_fold_btn, &QPushButton::clicked, [=]() {
    emit signComposeMachineWipeFold();
  });

  connect(ui->extend_needle_btn, &QPushButton::clicked, [=]() {
    emit signComposeMachineExtendNeedle();
  });

  connect(ui->withdraw_needle_btn, &QPushButton::clicked, [=]() {
    emit signComposeMachineWithdrawNeedle();
  });

  connect(ui->blow_wind_btn, &QPushButton::clicked, [=]() {
    emit signComposeMachineBlowWind();
  });

  connect(ui->stop_blow_btn, &QPushButton::clicked, [=]() {
    emit signComposeMachineStopBlow();
  });

  connect(ui->compose_move_hand_btn, &QPushButton::clicked, [=]() {
    emit signComposeMachineMoveHand(ui->compose_hand_x_spinbox->value(), ui->compose_hand_y_spinbox->value(), ui->compose_hand_z_spinbox->value(), ui->compose_hand_c_spinbox->value());
  });

  qRegisterMetaType<syt_msgs::msg::ComposeMachineSuckerStates>("syt_msgs::msg::ComposeMachineSuckerStates");
  connect(ui->sucker_dilate_btn, &QPushButton::clicked, [=]() {
    syt_msgs::msg::ComposeMachineSuckerStates sucker_states;
    sucker_states.left_bottom.x    = ui->sucker_left_bottom_x_line_edit->text().toFloat() + 5;
    sucker_states.left_bottom.y    = ui->sucker_left_bottom_y_line_edit->text().toFloat() + 5;
    sucker_states.left_oxter.x     = ui->sucker_left_oxter_x_line_edit->text().toFloat() + 5;
    sucker_states.left_oxter.y     = ui->sucker_left_oxter_y_line_edit->text().toFloat() + 5;
    sucker_states.left_shoulder.x  = ui->sucker_left_shoulder_x_line_edit->text().toFloat() + 5;
    sucker_states.left_shoulder.y  = ui->sucker_left_shoulder_y_line_edit->text().toFloat() + 5;
    sucker_states.left_shoulder.c  = ui->sucker_left_shoulder_c_line_edit->text().toFloat() + 5;
    sucker_states.right_shoulder.x = ui->sucker_right_shoulder_x_line_edit->text().toFloat() + 5;
    sucker_states.right_shoulder.y = ui->sucker_right_shoulder_y_line_edit->text().toFloat() + 5;
    sucker_states.right_shoulder.c = ui->sucker_right_shoulder_c_line_edit->text().toFloat() + 5;
    sucker_states.right_oxter.x    = ui->sucker_right_oxter_x_line_edit->text().toFloat() + 5;
    sucker_states.right_oxter.y    = ui->sucker_right_oxter_y_line_edit->text().toFloat() + 5;
    sucker_states.right_bottom.x   = ui->sucker_right_bottom_x_line_edit->text().toFloat() + 5;
    sucker_states.right_bottom.y   = ui->sucker_right_bottom_y_line_edit->text().toFloat() + 5;
    emit signComposeMachineMoveSucker(sucker_states);
  });

  connect(ui->sucker_shrink_btn, &QPushButton::clicked, [=]() {
    syt_msgs::msg::ComposeMachineSuckerStates sucker_states;
    sucker_states.left_bottom.x    = ui->sucker_left_bottom_x_line_edit->text().toFloat() - 5;
    sucker_states.left_bottom.y    = ui->sucker_left_bottom_y_line_edit->text().toFloat() - 5;
    sucker_states.left_oxter.x     = ui->sucker_left_oxter_x_line_edit->text().toFloat() - 5;
    sucker_states.left_oxter.y     = ui->sucker_left_oxter_y_line_edit->text().toFloat() - 5;
    sucker_states.left_shoulder.x  = ui->sucker_left_shoulder_x_line_edit->text().toFloat() - 5;
    sucker_states.left_shoulder.y  = ui->sucker_left_shoulder_y_line_edit->text().toFloat() - 5;
    sucker_states.left_shoulder.c  = ui->sucker_left_shoulder_c_line_edit->text().toFloat() - 5;
    sucker_states.right_shoulder.x = ui->sucker_right_shoulder_x_line_edit->text().toFloat() - 5;
    sucker_states.right_shoulder.y = ui->sucker_right_shoulder_y_line_edit->text().toFloat() - 5;
    sucker_states.right_shoulder.c = ui->sucker_right_shoulder_c_line_edit->text().toFloat() - 5;
    sucker_states.right_oxter.x    = ui->sucker_right_oxter_x_line_edit->text().toFloat() - 5;
    sucker_states.right_oxter.y    = ui->sucker_right_oxter_y_line_edit->text().toFloat() - 5;
    sucker_states.right_bottom.x   = ui->sucker_right_bottom_x_line_edit->text().toFloat() - 5;
    sucker_states.right_bottom.y   = ui->sucker_right_bottom_y_line_edit->text().toFloat() - 5;
    emit signComposeMachineMoveSucker(sucker_states);
  });
}

// 绑定缝纫机功能
void DeveloperWidget::bindSewingMachine() {
  connect(ui->set_current_sewing_hand_btn, &QPushButton::clicked, [=]() {
    ui->sewing_hand_x_spinbox->setValue(ui->sewing_hand_x_line_edit->text().toFloat());
    ui->sewing_hand_y_spinbox->setValue(ui->sewing_hand_y_line_edit->text().toFloat());
    ui->sewing_hand_c_spinbox->setValue(ui->sewing_hand_c_line_edit->text().toFloat());
  });

  connect(ui->sewing_reset_btn, &QPushButton::clicked, [=]() {
    emit signSewingMachineReset();
  });

  connect(ui->sewing_stop_btn, &QPushButton::clicked, [=]() {
    emit signSewingMachineStop();
  });

  connect(ui->sewing_move_hand_btn, &QPushButton::clicked, [=]() {
    emit signSewingMachineMoveHand(ui->sewing_hand_x_spinbox->value(), ui->sewing_hand_y_spinbox->value(), ui->sewing_hand_c_spinbox->value(), ui->sewing_hand_z_spinbox->value());
  });

  qRegisterMetaType<syt_msgs::msg::ClothKeypoints2f>("syt_msgs::msg::ClothKeypoints2f");
  connect(ui->send_keypoints_btn, &QPushButton::clicked, [=]() {
    syt_msgs::msg::ClothKeypoints2f keypoints;
    keypoints.left_bottom.x    = ui->sewing_left_bottom_x_line_edit->text().toFloat();
    keypoints.left_bottom.y    = ui->sewing_left_bottom_y_line_edit->text().toFloat();
    keypoints.left_oxter.x     = ui->sewing_left_oxter_x_line_edit->text().toFloat();
    keypoints.left_oxter.y     = ui->sewing_left_oxter_y_line_edit->text().toFloat();
    keypoints.left_shoulder.x  = ui->sewing_left_shoulder_x_line_edit->text().toFloat();
    keypoints.left_shoulder.y  = ui->sewing_left_shoulder_y_line_edit->text().toFloat();
    keypoints.left_collar.x    = ui->sewing_left_collar_x_line_edit->text().toFloat();
    keypoints.left_collar.y    = ui->sewing_left_collar_y_line_edit->text().toFloat();
    keypoints.right_collar.x   = ui->sewing_right_collar_x_line_edit->text().toFloat();
    keypoints.right_collar.y   = ui->sewing_right_collar_y_line_edit->text().toFloat();
    keypoints.right_shoulder.x = ui->sewing_right_shoulder_x_line_edit->text().toFloat();
    keypoints.right_shoulder.y = ui->sewing_right_shoulder_y_line_edit->text().toFloat();
    keypoints.right_oxter.x    = ui->sewing_right_oxter_x_line_edit->text().toFloat();
    keypoints.right_oxter.y    = ui->sewing_right_oxter_y_line_edit->text().toFloat();
    keypoints.right_bottom.x   = ui->sewing_right_bottom_x_line_edit->text().toFloat();
    keypoints.right_bottom.y   = ui->sewing_right_bottom_y_line_edit->text().toFloat();
    emit signSewingMachineSendKeypoints(keypoints);
  });
}

void DeveloperWidget::setComposeMachineState(syt_msgs::msg::ComposeMachineState state) {
  ui->compose_hand_x_line_edit->setText(QString::number(state.hand_position.x));
  ui->compose_hand_y_line_edit->setText(QString::number(state.hand_position.y));
  ui->compose_hand_z_line_edit->setText(QString::number(state.hand_position.z));
  ui->compose_hand_c_line_edit->setText(QString::number(state.hand_position.c));

  ui->sucker_left_bottom_x_line_edit->setText(QString::number(state.suckers_position.left_bottom.x));
  ui->sucker_left_bottom_y_line_edit->setText(QString::number(state.suckers_position.left_bottom.y));
  ui->sucker_left_oxter_x_line_edit->setText(QString::number(state.suckers_position.left_oxter.x));
  ui->sucker_left_oxter_y_line_edit->setText(QString::number(state.suckers_position.left_oxter.y));
  ui->sucker_left_shoulder_x_line_edit->setText(QString::number(state.suckers_position.left_shoulder.x));
  ui->sucker_left_shoulder_y_line_edit->setText(QString::number(state.suckers_position.left_shoulder.y));
  ui->sucker_left_shoulder_c_line_edit->setText(QString::number(state.suckers_position.left_shoulder.c));
  ui->sucker_right_shoulder_x_line_edit->setText(QString::number(state.suckers_position.right_shoulder.x));
  ui->sucker_right_shoulder_y_line_edit->setText(QString::number(state.suckers_position.right_shoulder.y));
  ui->sucker_right_shoulder_c_line_edit->setText(QString::number(state.suckers_position.right_shoulder.c));
  ui->sucker_right_oxter_x_line_edit->setText(QString::number(state.suckers_position.right_oxter.x));
  ui->sucker_right_oxter_y_line_edit->setText(QString::number(state.suckers_position.right_oxter.y));
  ui->sucker_right_bottom_x_line_edit->setText(QString::number(state.suckers_position.right_bottom.x));
  ui->sucker_right_bottom_y_line_edit->setText(QString::number(state.suckers_position.right_bottom.y));
}

void DeveloperWidget::setSewingMachineState(syt_msgs::msg::SewingMachineState state) {
  ui->sewing_hand_x_line_edit->setText(QString::number(state.hand_position.x));
  ui->sewing_hand_y_line_edit->setText(QString::number(state.hand_position.y));
  ui->sewing_hand_c_line_edit->setText(QString::number(state.hand_position.c));
}
