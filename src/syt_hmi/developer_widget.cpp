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

  // 切换页面
  connect(ui->switch_load_btn, &QPushButton::clicked, this, &DeveloperWidget::switchPage, Qt::UniqueConnection);
  connect(ui->switch_compose_btn, &QPushButton::clicked, this, &DeveloperWidget::switchPage, Qt::UniqueConnection);
  connect(ui->switch_sewing_btn, &QPushButton::clicked, this, &DeveloperWidget::switchPage, Qt::UniqueConnection);
  connect(ui->other_btn, &QPushButton::clicked, this, &DeveloperWidget::switchPage, Qt::UniqueConnection);

  // 设置一些参数
  setParam();

  // 上料机功能绑定
  bindLoadMachine();

  // 合片机功能
  bindComposeMachine();

  // 缝纫机功能
  bindSewingMachine();

  // 其他功能
  bindOther();
}

DeveloperWidget::~DeveloperWidget() {
  delete ui;
}

bool DeveloperWidget::event(QEvent *event) {
  if (event->type() == QEvent::LanguageChange) {
    ui->retranslateUi(this);
  }
  return QWidget::event(event);
}

void DeveloperWidget::mousePressEvent(QMouseEvent *event) {
  switch (event->button()) {
  case Qt::LeftButton:
    is_mouse_left_press_down_ = true;
    mouse_pos_ = event->globalPos() - this->frameGeometry().topLeft();
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
    button->setStyleSheet("qproperty-press_color: rgba(0,0,120,0.5);");
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
  setFrame(ui->set_tray_offset_btn_B);
  setFrame(ui->set_offset_btn_B);
  setFrame(ui->hold_cloth_btn_B);
  setFrame(ui->grab_cloth_btn_B);
  setFrame(ui->pre_setup_btn_B);
  setFrame(ui->visual_align_btn_B);
  setFrame(ui->rough_align_btn_B);
  setFrame(ui->thickness_btn_B);
  setFrame(ui->pop_needle_btn_B);
  setFrame(ui->withdraw_needle_btn_B);

  setFrame(ui->load_reset_btn_A);
  setFrame(ui->add_cloth_btn_A);
  setFrame(ui->clear_table_btn_A);
  setFrame(ui->set_cloth_size_btn_A);
  setFrame(ui->set_load_distance_btn_A);
  setFrame(ui->set_tray_gap_btn_A);
  setFrame(ui->set_tray_offset_btn_A);
  setFrame(ui->set_offset_btn_A);
  setFrame(ui->hold_cloth_btn_A);
  setFrame(ui->grab_cloth_btn_A);
  setFrame(ui->pre_setup_btn_A);
  setFrame(ui->visual_align_btn_A);
  setFrame(ui->rough_align_btn_A);
  setFrame(ui->thickness_btn_A);
  setFrame(ui->pop_needle_btn_A);
  setFrame(ui->withdraw_needle_btn_A);

  // 合片机界面
  setFrame(ui->compose_reset_btn);
  setFrame(ui->compose_stop_btn);
  setFrame(ui->wipe_fold_btn);
  setFrame(ui->extend_needle_btn);
  setFrame(ui->withdraw_needle_btn);
  setFrame(ui->blow_wind_btn);
  setFrame(ui->stop_blow_btn);
  setFrame(ui->fasten_sheet_btn);
  setFrame(ui->unfasten_sheet_btn);
  setFrame(ui->set_current_compose_hand_btn);
  setFrame(ui->compose_move_hand_btn);
  setFrame(ui->sucker_dilate_btn);
  setFrame(ui->sucker_shrink_btn);
  setFrame(ui->blow_height_btn);
  setFrame(ui->table_light_btn);

  // 缝纫机界面
  setFrame(ui->sewing_reset_btn);
  setFrame(ui->sewing_stop_btn);
  setFrame(ui->set_current_sewing_hand_btn);
  setFrame(ui->sewing_move_hand_btn);
  setFrame(ui->send_keypoints_btn);
  setFrame(ui->needle_length_btn);
  setFrame(ui->label_width_btn);
  setFrame(ui->label_reset_btn);

  // 其他界面
  setFrame(ui->choose_bin_btn);
  setFrame(ui->flash_btn);
  setFrame(ui->emergency_stop_btn);
  setFrame(ui->red_light_btn);
  setFrame(ui->green_light_btn);
  setFrame(ui->yellow_light_btn);
  setFrame(ui->bell_open_btn);
  setFrame(ui->bell_close_btn);
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

  connect(ui->set_tray_offset_btn_B, &QPushButton::clicked, [=]() {
    emit signLoadMachineTrayOffset(0, ui->tray_offset_spinbox_B->value());
  });

  connect(ui->rough_align_btn_B, &QPushButton::clicked, [=]() {
    emit signLoadMachineRoughAlign(0);
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

  connect(ui->thickness_btn_B, &QPushButton::clicked, [=]() {
    emit signLoadMachineThickness(0, ui->thickness_spinbox_B->value());
  });

  connect(ui->pop_needle_btn_B, &QPushButton::clicked, [=]() {
    emit signLoadMachineExtendNeedle(0, 1);
  });

  connect(ui->withdraw_needle_btn_B, &QPushButton::clicked, [=]() {
    emit signLoadMachineExtendNeedle(0, 0);
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

  connect(ui->set_tray_offset_btn_A, &QPushButton::clicked, [=]() {
    emit signLoadMachineTrayOffset(1, ui->tray_offset_spinbox_A->value());
  });

  connect(ui->rough_align_btn_A, &QPushButton::clicked, [=]() {
    emit signLoadMachineRoughAlign(1);
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
    emit signLoadMachineVisualAlign(1);
  });

  connect(ui->thickness_btn_A, &QPushButton::clicked, [=]() {
    emit signLoadMachineThickness(1, ui->thickness_spinbox_A->value());
  });

  connect(ui->pop_needle_btn_A, &QPushButton::clicked, [=]() {
    emit signLoadMachineExtendNeedle(1, 1);
  });

  connect(ui->withdraw_needle_btn_A, &QPushButton::clicked, [=]() {
    emit signLoadMachineExtendNeedle(1, 0);
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

  connect(ui->fasten_sheet_btn, &QPushButton::clicked, [=]() {
    emit signComposeMachineFastenSheet();
  });

  connect(ui->unfasten_sheet_btn, &QPushButton::clicked, [=]() {
    emit signComposeMachineUnfastenSheet();
  });

  connect(ui->compose_move_hand_btn, &QPushButton::clicked, [=]() {
    emit signComposeMachineMoveHand(ui->compose_hand_x_spinbox->value(), ui->compose_hand_y_spinbox->value(), ui->compose_hand_z_spinbox->value(), ui->compose_hand_c_spinbox->value());
  });

  qRegisterMetaType<syt_msgs::msg::ComposeMachineSuckerStates>("syt_msgs::msg::ComposeMachineSuckerStates");
  connect(ui->sucker_dilate_btn, &QPushButton::clicked, [=]() {
    syt_msgs::msg::ComposeMachineSuckerStates sucker_states;
    sucker_states.left_bottom.x = ui->sucker_left_bottom_x_line_edit->text().toFloat() + 5 - left_bottom_init_x_;
    sucker_states.left_bottom.y = ui->sucker_left_bottom_y_line_edit->text().toFloat() + 5 - left_bottom_init_y_;
    sucker_states.left_oxter.x = ui->sucker_left_oxter_x_line_edit->text().toFloat() + 5 - left_oxter_init_x_;
    sucker_states.left_oxter.y = ui->sucker_left_oxter_y_line_edit->text().toFloat() + 5 - left_oxter_init_y_;
    sucker_states.left_shoulder.x = ui->sucker_left_shoulder_x_line_edit->text().toFloat() + 5 - left_shoulder_init_x_;
    sucker_states.left_shoulder.y = ui->sucker_left_shoulder_y_line_edit->text().toFloat() + 5 - left_shoulder_init_y_;
    sucker_states.left_shoulder.c = ui->sucker_left_shoulder_c_line_edit->text().toFloat() + 5 - left_shoulder_init_c_;
    sucker_states.right_shoulder.x = ui->sucker_right_shoulder_x_line_edit->text().toFloat() + 5 - right_shoulder_init_x_;
    sucker_states.right_shoulder.y = ui->sucker_right_shoulder_y_line_edit->text().toFloat() + 5 - right_shoulder_init_y_;
    sucker_states.right_shoulder.c = ui->sucker_right_shoulder_c_line_edit->text().toFloat() + 5 - right_shoulder_init_c_;
    sucker_states.right_oxter.x = ui->sucker_right_oxter_x_line_edit->text().toFloat() + 5 - right_oxter_init_x_;
    sucker_states.right_oxter.y = ui->sucker_right_oxter_y_line_edit->text().toFloat() + 5 - right_oxter_init_y_;
    sucker_states.right_bottom.x = ui->sucker_right_bottom_x_line_edit->text().toFloat() + 5 - right_bottom_init_x_;
    sucker_states.right_bottom.y = ui->sucker_right_bottom_y_line_edit->text().toFloat() + 5 - right_bottom_init_y_;
    emit signComposeMachineMoveSucker(sucker_states);
  });

  connect(ui->sucker_shrink_btn, &QPushButton::clicked, [=]() {
    syt_msgs::msg::ComposeMachineSuckerStates sucker_states;
    sucker_states.left_bottom.x = ui->sucker_left_bottom_x_line_edit->text().toFloat() - 5 - left_bottom_init_x_;
    sucker_states.left_bottom.y = ui->sucker_left_bottom_y_line_edit->text().toFloat() - 5 - left_bottom_init_y_;
    sucker_states.left_oxter.x = ui->sucker_left_oxter_x_line_edit->text().toFloat() - 5 - left_oxter_init_x_;
    sucker_states.left_oxter.y = ui->sucker_left_oxter_y_line_edit->text().toFloat() - 5 - left_oxter_init_y_;
    sucker_states.left_shoulder.x = ui->sucker_left_shoulder_x_line_edit->text().toFloat() - 5 - left_shoulder_init_x_;
    sucker_states.left_shoulder.y = ui->sucker_left_shoulder_y_line_edit->text().toFloat() - 5 - left_shoulder_init_y_;
    sucker_states.left_shoulder.c = ui->sucker_left_shoulder_c_line_edit->text().toFloat() - 5 - left_shoulder_init_c_;
    sucker_states.right_shoulder.x = ui->sucker_right_shoulder_x_line_edit->text().toFloat() - 5 - right_shoulder_init_x_;
    sucker_states.right_shoulder.y = ui->sucker_right_shoulder_y_line_edit->text().toFloat() - 5 - right_shoulder_init_y_;
    sucker_states.right_shoulder.c = ui->sucker_right_shoulder_c_line_edit->text().toFloat() - 5 - right_shoulder_init_c_;
    sucker_states.right_oxter.x = ui->sucker_right_oxter_x_line_edit->text().toFloat() - 5 - right_oxter_init_x_;
    sucker_states.right_oxter.y = ui->sucker_right_oxter_y_line_edit->text().toFloat() - 5 - right_oxter_init_y_;
    sucker_states.right_bottom.x = ui->sucker_right_bottom_x_line_edit->text().toFloat() - 5 - right_bottom_init_x_;
    sucker_states.right_bottom.y = ui->sucker_right_bottom_y_line_edit->text().toFloat() - 5 - right_bottom_init_y_;
    emit signComposeMachineMoveSucker(sucker_states);
  });

  connect(ui->fitting_plane_btn, &QPushButton::clicked, [=]() {
    emit signComposeMachineFittingPlane();
  });

  connect(ui->blow_height_btn, &QPushButton::clicked, [=]() {
    emit signComposeMachineBlowHeight(ui->blow_height_spin_box->value());
  });

  connect(ui->table_light_btn, &QPushButton::clicked, [=]() {
    emit signComposeMachineTableLight(ui->table_light_spin_box->value());
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
    keypoints.left_bottom.x = ui->sewing_left_bottom_x_line_edit->text().toFloat();
    keypoints.left_bottom.y = ui->sewing_left_bottom_y_line_edit->text().toFloat();
    keypoints.left_oxter.x = ui->sewing_left_oxter_x_line_edit->text().toFloat();
    keypoints.left_oxter.y = ui->sewing_left_oxter_y_line_edit->text().toFloat();
    keypoints.left_shoulder.x = ui->sewing_left_shoulder_x_line_edit->text().toFloat();
    keypoints.left_shoulder.y = ui->sewing_left_shoulder_y_line_edit->text().toFloat();
    keypoints.left_collar.x = ui->sewing_left_collar_x_line_edit->text().toFloat();
    keypoints.left_collar.y = ui->sewing_left_collar_y_line_edit->text().toFloat();
    keypoints.right_collar.x = ui->sewing_right_collar_x_line_edit->text().toFloat();
    keypoints.right_collar.y = ui->sewing_right_collar_y_line_edit->text().toFloat();
    keypoints.right_shoulder.x = ui->sewing_right_shoulder_x_line_edit->text().toFloat();
    keypoints.right_shoulder.y = ui->sewing_right_shoulder_y_line_edit->text().toFloat();
    keypoints.right_oxter.x = ui->sewing_right_oxter_x_line_edit->text().toFloat();
    keypoints.right_oxter.y = ui->sewing_right_oxter_y_line_edit->text().toFloat();
    keypoints.right_bottom.x = ui->sewing_right_bottom_x_line_edit->text().toFloat();
    keypoints.right_bottom.y = ui->sewing_right_bottom_y_line_edit->text().toFloat();
    emit signSewingMachineSendKeypoints(keypoints);
  });

  connect(ui->needle_length_btn, &QPushButton::clicked, [=]() {
    emit signSewingMachineNeedle(ui->line_1_spin_box->value(), ui->line_2_spin_box->value(), ui->line_3_spin_box->value(), ui->line_4_spin_box->value());
  });

  connect(ui->label_width_btn, &QPushButton::clicked, [=]() {
    int side = 0;
    if (ui->left_label_radio_box->isChecked()) {
      side = 0;
    } else if (ui->right_label_radio_box->isChecked()) {
      side = 1;
    }
    emit signSewingMachineLabelWidth(ui->enable_label_check_box->isChecked(), side, ui->label_width_spin_box->value(), ui->label_position_spin_box->value());
  });

  connect(ui->label_reset_btn, &QPushButton::clicked, [=]() {
    emit signSewingMachineLabelReset();
  });

  connect(ui->sewing_machine_speed_btn, &QPushButton::clicked, [=]() {
    switch (ui->sewing_machine_speed_combo_box->currentIndex()) {
    case 0:
      emit signSewingMachineSpeed(0);
      break;
    case 1:
      emit signSewingMachineSpeed(1);
      break;
    case 2:
      emit signSewingMachineSpeed(2);
      break;
    }
  });
}

void DeveloperWidget::bindOther() {
  // 模式切换
  setChooseMode();

  // 固件更新
  setUpdateBin();

  // 是否带缝纫
  setUseSewing();

  // 是否老化测试
  setPressureTest();

  // 称重传感器启用
  setWeightSwitch();

  // 称重传感器启用
  setUnpleatSwitch();

  // 曲线缝纫
  setCurveSewing();

  // 检查标定效果
  setCheckCalibration();

  // 急停
  connect(ui->emergency_stop_btn, &QPushButton::clicked, [=]() {
    emit signEmergencyStop();
  });

  // 红灯
  connect(ui->red_light_btn, &QPushButton::clicked, [=]() {
    emit signRedLight();
  });

  // 绿灯
  connect(ui->green_light_btn, &QPushButton::clicked, [=]() {
    emit signGreenLight();
  });

  // 黄灯
  connect(ui->yellow_light_btn, &QPushButton::clicked, [=]() {
    emit signYellowLight();
  });

  // 蜂鸣器开
  connect(ui->bell_open_btn, &QPushButton::clicked, [=]() {
    emit signBellOpen();
  });

  // 蜂鸣器关
  connect(ui->bell_close_btn, &QPushButton::clicked, [=]() {
    emit signBellClose();
  });
}

void DeveloperWidget::setComposeMachineState(syt_msgs::msg::ComposeMachineState state) {
  ui->compose_hand_x_line_edit->setText(QString::number(state.hand_position.x));
  ui->compose_hand_y_line_edit->setText(QString::number(state.hand_position.y));
  ui->compose_hand_z_line_edit->setText(QString::number(state.hand_position.z));
  ui->compose_hand_c_line_edit->setText(QString::number(state.hand_position.c));

  if (ui->switch_sucker_axis_ratio_button->isChecked()) {
    ui->sucker_left_bottom_x_line_edit->setText(QString::number(state.suckers_position.left_bottom.x - left_bottom_init_x_));
    ui->sucker_left_bottom_y_line_edit->setText(QString::number(state.suckers_position.left_bottom.y - left_bottom_init_y_));
    ui->sucker_left_oxter_x_line_edit->setText(QString::number(state.suckers_position.left_oxter.x - left_oxter_init_x_));
    ui->sucker_left_oxter_y_line_edit->setText(QString::number(state.suckers_position.left_oxter.y - left_oxter_init_y_));
    ui->sucker_left_shoulder_x_line_edit->setText(QString::number(state.suckers_position.left_shoulder.x - left_shoulder_init_x_));
    ui->sucker_left_shoulder_y_line_edit->setText(QString::number(state.suckers_position.left_shoulder.y - left_shoulder_init_y_));
    ui->sucker_left_shoulder_c_line_edit->setText(QString::number(state.suckers_position.left_shoulder.c - left_shoulder_init_c_));
    ui->sucker_right_shoulder_x_line_edit->setText(QString::number(state.suckers_position.right_shoulder.x - right_shoulder_init_x_));
    ui->sucker_right_shoulder_y_line_edit->setText(QString::number(state.suckers_position.right_shoulder.y - right_shoulder_init_y_));
    ui->sucker_right_shoulder_c_line_edit->setText(QString::number(state.suckers_position.right_shoulder.c - right_shoulder_init_c_));
    ui->sucker_right_oxter_x_line_edit->setText(QString::number(state.suckers_position.right_oxter.x - right_oxter_init_x_));
    ui->sucker_right_oxter_y_line_edit->setText(QString::number(state.suckers_position.right_oxter.y - right_oxter_init_y_));
    ui->sucker_right_bottom_x_line_edit->setText(QString::number(state.suckers_position.right_bottom.x - right_bottom_init_x_));
    ui->sucker_right_bottom_y_line_edit->setText(QString::number(state.suckers_position.right_bottom.y - right_bottom_init_y_));
  } else {
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
}

void DeveloperWidget::setSewingMachineState(syt_msgs::msg::SewingMachineState state) {
  ui->sewing_hand_x_line_edit->setText(QString::number(state.hand_position.x));
  ui->sewing_hand_y_line_edit->setText(QString::number(state.hand_position.y));
  ui->sewing_hand_c_line_edit->setText(QString::number(state.hand_position.c));
}

void DeveloperWidget::setParam() {
  std::string config_path = std::string(getenv("ENV_ROBOT_ETC")) + "/syt_robot_control/control.yaml";
  cv::FileStorage config_fs(config_path, cv::FileStorage::READ);

  int max_load_distance = 0;
  int min_load_distance = 0;
  config_fs["load_machine"]["max_load_distance"] >> max_load_distance;
  config_fs["load_machine"]["min_load_distance"] >> min_load_distance;
  ui->load_distance_spinbox_A->setMaximum(max_load_distance);
  ui->load_distance_spinbox_A->setMinimum(min_load_distance);
  ui->load_distance_spinbox_B->setMaximum(max_load_distance);
  ui->load_distance_spinbox_B->setMinimum(min_load_distance);

  int max_cloth_width = 0;
  int min_cloth_width = 0;
  config_fs["load_machine"]["max_cloth_width"] >> max_cloth_width;
  config_fs["load_machine"]["min_cloth_width"] >> min_cloth_width;
  ui->cloth_width_spinbox_A->setMaximum(max_cloth_width);
  ui->cloth_width_spinbox_A->setMinimum(min_cloth_width);
  ui->cloth_width_spinbox_B->setMaximum(max_cloth_width);
  ui->cloth_width_spinbox_B->setMinimum(min_cloth_width);

  // 缝纫机
  config_fs["compose_machine"]["suckers_init_position_in_compose_hand_axis"]["left_bottom"]["x"] >> left_bottom_init_x_;
  config_fs["compose_machine"]["suckers_init_position_in_compose_hand_axis"]["left_bottom"]["y"] >> left_bottom_init_y_;
  config_fs["compose_machine"]["suckers_init_position_in_compose_hand_axis"]["left_oxter"]["x"] >> left_oxter_init_x_;
  config_fs["compose_machine"]["suckers_init_position_in_compose_hand_axis"]["left_oxter"]["y"] >> left_oxter_init_y_;
  config_fs["compose_machine"]["suckers_init_position_in_compose_hand_axis"]["left_shoulder"]["x"] >> left_shoulder_init_x_;
  config_fs["compose_machine"]["suckers_init_position_in_compose_hand_axis"]["left_shoulder"]["y"] >> left_shoulder_init_y_;
  config_fs["compose_machine"]["suckers_init_position_in_compose_hand_axis"]["left_shoulder"]["c"] >> left_shoulder_init_c_;
  config_fs["compose_machine"]["suckers_init_position_in_compose_hand_axis"]["right_shoulder"]["x"] >> right_shoulder_init_x_;
  config_fs["compose_machine"]["suckers_init_position_in_compose_hand_axis"]["right_shoulder"]["y"] >> right_shoulder_init_y_;
  config_fs["compose_machine"]["suckers_init_position_in_compose_hand_axis"]["right_shoulder"]["c"] >> right_shoulder_init_c_;
  config_fs["compose_machine"]["suckers_init_position_in_compose_hand_axis"]["right_oxter"]["x"] >> right_oxter_init_x_;
  config_fs["compose_machine"]["suckers_init_position_in_compose_hand_axis"]["right_oxter"]["y"] >> right_oxter_init_y_;
  config_fs["compose_machine"]["suckers_init_position_in_compose_hand_axis"]["right_bottom"]["x"] >> right_bottom_init_x_;
  config_fs["compose_machine"]["suckers_init_position_in_compose_hand_axis"]["right_bottom"]["y"] >> right_bottom_init_y_;

  float compose_max_hand_x = 0;
  float compose_min_hand_x = -120;
  float compose_max_hand_y = 1168;
  float compose_min_hand_y = 0;
  float compose_max_hand_z = 0;
  float compose_min_hand_z = -162;
  float compose_max_hand_c = 0.35;
  float compose_min_hand_c = -0.94;

  config_fs["compose_machine"]["hand_limit_position"]["x_max"] >> compose_max_hand_x;
  config_fs["compose_machine"]["hand_limit_position"]["x_min"] >> compose_min_hand_x;
  ui->compose_hand_x_spinbox->setMaximum(compose_max_hand_x);
  ui->compose_hand_x_spinbox->setMinimum(compose_min_hand_x);

  config_fs["compose_machine"]["hand_limit_position"]["y_max"] >> compose_max_hand_y;
  config_fs["compose_machine"]["hand_limit_position"]["y_min"] >> compose_min_hand_y;
  ui->compose_hand_y_spinbox->setMaximum(compose_max_hand_y);
  ui->compose_hand_y_spinbox->setMinimum(compose_min_hand_y);

  config_fs["compose_machine"]["hand_limit_position"]["z_max"] >> compose_max_hand_z;
  config_fs["compose_machine"]["hand_limit_position"]["z_min"] >> compose_min_hand_z;
  ui->compose_hand_z_spinbox->setMaximum(compose_max_hand_z);
  ui->compose_hand_z_spinbox->setMinimum(compose_min_hand_z);

  config_fs["compose_machine"]["hand_limit_position"]["c_max"] >> compose_max_hand_c;
  config_fs["compose_machine"]["hand_limit_position"]["c_min"] >> compose_min_hand_c;
  ui->compose_hand_c_spinbox->setMaximum(compose_max_hand_c);
  ui->compose_hand_c_spinbox->setMinimum(compose_min_hand_c);

  // 缝纫机
  float sewing_max_hand_x = 1225;
  float sewing_min_hand_x = -880;
  float sewing_max_hand_y = 310;
  float sewing_min_hand_y = -348;
  float sewing_max_hand_c = 1.8;
  float sewing_min_hand_c = -1.8;

  config_fs["sewing_machine"]["hand_limit_position"]["x_max"] >> sewing_max_hand_x;
  config_fs["sewing_machine"]["hand_limit_position"]["x_min"] >> sewing_min_hand_x;
  ui->sewing_hand_x_spinbox->setMaximum(sewing_max_hand_x);
  ui->sewing_hand_x_spinbox->setMinimum(sewing_min_hand_x);

  config_fs["sewing_machine"]["hand_limit_position"]["y_max"] >> sewing_max_hand_y;
  config_fs["sewing_machine"]["hand_limit_position"]["y_min"] >> sewing_min_hand_y;
  ui->sewing_hand_y_spinbox->setMaximum(sewing_max_hand_y);
  ui->sewing_hand_y_spinbox->setMinimum(sewing_min_hand_y);

  config_fs["sewing_machine"]["hand_limit_position"]["c_max"] >> sewing_max_hand_c;
  config_fs["sewing_machine"]["hand_limit_position"]["c_min"] >> sewing_min_hand_c;
  ui->sewing_hand_c_spinbox->setMaximum(sewing_max_hand_c);
  ui->sewing_hand_c_spinbox->setMinimum(sewing_min_hand_c);

  config_fs.release();
}

void DeveloperWidget::setChooseMode() {
  // 模式选择界面
  void (QComboBox::*index_change_signal)(int index) = &QComboBox::currentIndexChanged;
  connect(ui->choose_mode_combo_box, index_change_signal, [=]() {
    switch (ui->choose_mode_combo_box->currentIndex()) {
    case 0:
      emit signChooseMode(syt_msgs::msg::FSMRunMode::LOOP);
      break;
    case 1:
      emit signChooseMode(syt_msgs::msg::FSMRunMode::LOOP_ONCE);
      break;
    case 2:
      emit signChooseMode(syt_msgs::msg::FSMRunMode::COMPOSE_CLOTH);
      break;
    case 3:
      emit signChooseMode(syt_msgs::msg::FSMRunMode::SEW_CLOTH);
      break;
    default:
      break;
    }
  });
}

void DeveloperWidget::setUpdateBin() {
  // 保存bin文件路径
  connect(ui->choose_bin_btn, &QPushButton::clicked, this, [=]() {
    update_bin_path_ = QFileDialog::getOpenFileName(this, tr("请选择模板路径"), QDir::homePath(), "*.bin");
    if (!update_bin_path_.isEmpty()) {
      ui->bin_file_line_edit->setText(update_bin_path_);
    }
  });

  connect(ui->flash_btn, &QPushButton::clicked, [=]() {
    if (QFile::exists(update_bin_path_)) {
      QMap<int, QString> port_map;
      port_map.insert(0, "/dev/load_machine");
      port_map.insert(1, "/dev/compose_machine");
      port_map.insert(2, "/dev/sewing_machine");

      switch (ui->choose_port_combo_box->currentIndex()) {
      case 0:
        emit signUpdateLoadMachine();
        break;
      case 1:
        emit signUpdateComposeMachine();
        break;
      case 2:
        emit signUpdateSewingMachine();
        break;
      }

      QThread::msleep(200);
      for (int i = 0; i < 3; ++i) {
        killProcesses("thanos.launch.py");
        killProcesses("ros-args");
      }

      QString command = QString("download %1 %2").arg(port_map.value(ui->choose_port_combo_box->currentIndex())).arg(update_bin_path_);
      int result = system(command.toStdString().c_str());

      if (0 == result) {
        showMessageBox(this, SUCCESS, ui->choose_port_combo_box->currentText() + "更新成功", 1, {tr("确认")});
      } else {
        showMessageBox(this, ERROR, ui->choose_port_combo_box->currentText() + "更新失败", 1, {tr("确认")});
      }
    } else {
      showMessageBox(this, WARN, tr("所选文件不存在"), 1, {tr("确认")});
    }
  });
}

void DeveloperWidget::setUseSewing() {
  connect(ui->use_sewing_check_box, &QCheckBox::toggled, [=](bool toggled) {
    if (toggled) {
      system("ros2 param set /syt_sewing_machine_node use_sewing True");
    } else {
      system("ros2 param set /syt_sewing_machine_node use_sewing False");
    }
  });
}

void DeveloperWidget::setPressureTest() {
  connect(ui->pressure_test_check_box, &QCheckBox::toggled, [=](bool toggled) {
    if (toggled) {
      system("ros2 param set /syt_compose_machine_node pressure_test True");
      signLoadMachineAgingSwitch(0, 1);
      signLoadMachineAgingSwitch(1, 1);
    } else {
      system("ros2 param set /syt_compose_machine_node pressure_test False");
      signLoadMachineAgingSwitch(0, 0);
      signLoadMachineAgingSwitch(1, 0);
    }
  });
}

void DeveloperWidget::setWeightSwitch() {
  connect(ui->weight_switch_check_box, &QCheckBox::toggled, [=](bool toggled) {
    if (toggled) {
      signLoadMachineWeightSwitch(0, 1);
      signLoadMachineWeightSwitch(1, 1);
    } else {
      signLoadMachineWeightSwitch(0, 0);
      signLoadMachineWeightSwitch(1, 0);
    }
  });
}

void DeveloperWidget::setUnpleatSwitch() {
  connect(ui->unpleat_switch_check_box, &QCheckBox::toggled, [=](bool toggled) {
    if (toggled) {
      signLoadMachineUnpleatSwitch(0, 1);
      signLoadMachineUnpleatSwitch(1, 1);
    } else {
      signLoadMachineUnpleatSwitch(0, 0);
      signLoadMachineUnpleatSwitch(1, 0);
    }
  });
}

void DeveloperWidget::setCurveSewing() {
  connect(ui->curve_sewing_check_box, &QCheckBox::toggled, [=](bool toggled) {
    if (toggled) {
      system("ros2 param set /syt_sewing_machine_node curve_sewing True");
    } else {
      system("ros2 param set /syt_sewing_machine_node curve_sewing False");
    }
  });
}

void DeveloperWidget::setCheckCalibration() {
  connect(ui->check_calibration_btn, &QPushButton::clicked, [=]() {
    emit signCheckCalib();
  });
}

void DeveloperWidget::setCheckCalibrationResult(bool result, float bottom_length, float side_length) {
  if (result) {
    ui->calib_bottom_length_line_edit->setText(QString::number(bottom_length));
    ui->calib_side_length_line_edit->setText(QString::number(side_length));
  } else {
    ui->calib_bottom_length_line_edit->setText(QString::number(0));
    ui->calib_side_length_line_edit->setText(QString::number(0));
  }
}
