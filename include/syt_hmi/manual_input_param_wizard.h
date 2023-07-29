#pragma once

#include "syt_hmi/wizard_pages.h"
#include "syt_msgs/msg/cloth_style.hpp"

#include <QWizard>

class ManualInputParamWizard : public QWizard {
  Q_OBJECT
private:
  syt_msgs::msg::ClothStyle cloth_style_front_;
  syt_msgs::msg::ClothStyle cloth_style_back_;
  QString file_name_;

public:
  ManualInputParamWizard(QWidget *parent = nullptr);
  ~ManualInputParamWizard(){};

signals:
  void signCreateStyle(int mode, syt_msgs::msg::ClothStyle cloth_style_front, syt_msgs::msg::ClothStyle cloth_style_back);
  void signCreateStyleResult(bool result);
  void signSetRenameEdit(QString file_name);
  void signRenameClothStyle(QString old_name, QString new_name);
  void signRenameClothStyleResult(bool result);

public slots:
  void slotSetLengthParam(syt_msgs::msg::ClothStyle cloth_style);
  void slotSetExtraParam(syt_msgs::msg::ClothStyle cloth_style);
  void slotCreateStyle();
  void slotCreateStyleResult(bool result, QString file_name);
  void slotSetRenameEdit();
  void slotRenameClothStyle();
  void slotRenameClothStyleResult(bool result);
};
