#pragma once

#include "syt_hmi/wizard_pages.h"
#include "syt_msgs/msg/cloth_info.hpp"
#include "utils/waitingspinnerwidget.h"

#include <QDir>
#include <QWizard>

class AutoCreateStyleWizard : public QWizard {
  Q_OBJECT

private:
  syt_msgs::msg::ClothInfo cloth_info_front_;
  syt_msgs::msg::ClothInfo cloth_info_back_;
  syt_msgs::msg::ClothStyle cloth_style_front_;
  syt_msgs::msg::ClothStyle cloth_style_back_;
  QString file_prefix_;
  QString file_name_;

  WaitingSpinnerWidget *waiting_spinner_widget_;

public:
  AutoCreateStyleWizard(QWidget *parent = nullptr);
  ~AutoCreateStyleWizard(){};

  enum AUTO_CREATE_STYLE_PAGE {
    MOVE_HAND_PAGE,
    DETECT_FRONT_PAGE,
    EXTRA_FRONT_PAGE,
    DETECT_BACK_PAGE,
    EXTRA_BACK_PAGE,
    TOLERANCE_PAGE,
    CREATE_STYLE_PAGE,
    RENAME_STYLE_PAGE,
  };
  Q_ENUM(AUTO_CREATE_STYLE_PAGE);

signals:
  void signMoveHand();
  void signMoveHandResult(bool result);
  void signDetectCloth(int cloth_type);
  void signDetectClothResult(bool result, int cloth_type);
  void signCreateStyle(int mode, QString prefix, syt_msgs::msg::ClothStyle cloth_style_front, syt_msgs::msg::ClothStyle cloth_style_back);
  void signCreateStyleResult(bool result);
  void signSetRenameEdit(QString file_name);
  void signRenameClothStyle(QString old_name, QString new_name);
  void signRenameClothStyleResult(bool result);

public slots:
  void slotMoveHand();
  void slotMoveHandResult(bool result);
  void slotDetectCloth(int cloth_type);
  void slotDetectClothResult(bool result, int cloth_type, syt_msgs::msg::ClothInfo cloth_info);
  void slotSetExtraParam(syt_msgs::msg::ClothStyle cloth_style);
  void slotSetToleranceParam(syt_msgs::msg::ClothStyle cloth_style);
  void slotCreateStyle(QString prefix);
  void slotCreateStyleResult(bool result, QString file_name);
  void slotSetRenameEdit();
  void slotRenameClothStyle();
  void slotRenameClothStyleResult(bool result);
};
