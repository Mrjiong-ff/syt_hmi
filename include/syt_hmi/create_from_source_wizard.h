#pragma once

#include "syt_hmi/wizard_pages.h"

#include <QWizard>

class CreateFromSourceWizard : public QWizard {
  Q_OBJECT
public:
  CreateFromSourceWizard(QWidget *parent = nullptr);
  ~CreateFromSourceWizard(){};
};
