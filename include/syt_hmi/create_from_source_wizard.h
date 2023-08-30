#pragma once

#include "syt_hmi/wizard_pages.h"
#include "syt_msgs/msg/cloth_style.hpp"
#include "utils/waitingspinnerwidget.h"

#include <QDir>
#include <QWizard>

class CreateFromSourceWizard : public QWizard {
  Q_OBJECT
private:
  syt_msgs::msg::ClothStyle cloth_style_front_;
  syt_msgs::msg::ClothStyle cloth_style_back_;
  QString file_name_;

  WaitingSpinnerWidget *waiting_spinner_widget_;

public:
  CreateFromSourceWizard(QWidget *parent = nullptr);
  ~CreateFromSourceWizard(){};

  enum CREATE_FROM_SOURCE_PAGE {
    CHOOSE_STYLE_PAGE,
    DISPLAY_FRONT_PAGE,
    DISPLAY_BACK_PAGE,
    CREATE_STYLE_PAGE,
    RENAME_STYLE_PAGE,
  };
  Q_ENUM(CREATE_FROM_SOURCE_PAGE);

signals:
  void signGetClothStyle(QString prefix, QString style_name);
  void signGetClothStyleResult(bool result);
  void signSetFullParam(syt_msgs::msg::ClothStyle cloth_style);
  void signCreateStyle(int mode, QString prefix, syt_msgs::msg::ClothStyle cloth_style_front, syt_msgs::msg::ClothStyle cloth_style_back);
  void signCreateStyleResult(bool result);
  void signSetRenameEdit(QString file_name);
  void signRenameClothStyle(QString old_name, QString new_name);
  void signRenameClothStyleResult(bool result);

public slots:
  void slotGetClothStyle(QString prefix, QString style_name);
  void slotGetClothStyleResult(bool result, syt_msgs::msg::ClothStyle style_front, syt_msgs::msg::ClothStyle style_back);
  void slotSetFullParam(syt_msgs::msg::ClothStyle cloth_style);
  void slotCreateStyle(QString prefix);
  void slotCreateStyleResult(bool result, QString file_name);
  void slotSetRenameEdit();
  void slotRenameClothStyle();
  void slotRenameClothStyleResult(bool result);
};
