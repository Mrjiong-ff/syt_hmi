#pragma once

#include "syt_hmi/wizard_pages.h"

#include <QWizard>

class ManualInputParamWizard : public QWizard {
  Q_OBJECT
public:
  ManualInputParamWizard(QWidget *parent = nullptr);
  ~ManualInputParamWizard(){};
};
