#pragma once

#include "syt_hmi/wizard_pages.h"
#include "syt_msgs/msg/cloth_style.hpp"
#include "utils/waitingspinnerwidget.h"

#include <QDir>
#include <QWizard>

class ManualInputParamWizard : public QWizard {
  Q_OBJECT

private:
  syt_msgs::msg::ClothStyle cloth_style_front_;
  syt_msgs::msg::ClothStyle cloth_style_back_;
  QString file_name_;

  WaitingSpinnerWidget *waiting_spinner_widget_;

public:
  ManualInputParamWizard(QWidget *parent = nullptr);
  ~ManualInputParamWizard(){};

  enum MANUAL_INPUT_PARAM_PAGE {
    LENGTH_FRONT_PAGE,
    EXTRA_FRONT_PAGE,
    LENGTH_BACK_PAGE,
    EXTRA_BACK_PAGE,
    TOLERANCE_PAGE,
    CREATE_STYLE_PAGE,
    RENAME_STYLE_PAGE,
  };
  Q_ENUM(MANUAL_INPUT_PARAM_PAGE);

signals:
  void signCreateStyle(int mode, QString prefix, syt_msgs::msg::ClothStyle cloth_style_front, syt_msgs::msg::ClothStyle cloth_style_back);
  void signCreateStyleResult(bool result);
  void signSetRenameEdit(QString file_name);
  void signRenameClothStyle(QString old_name, QString new_name);
  void signRenameClothStyleResult(bool result);

public slots:
  void slotSetLengthParam(syt_msgs::msg::ClothStyle cloth_style);
  void slotSetExtraParam(syt_msgs::msg::ClothStyle cloth_style);
  void slotSetToleranceParam(syt_msgs::msg::ClothStyle cloth_style);
  void slotCreateStyle(QString prefix);
  void slotCreateStyleResult(bool result, QString file_name);
  void slotSetRenameEdit();
  void slotRenameClothStyle();
  void slotRenameClothStyleResult(bool result);
};
