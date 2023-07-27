#pragma once

#include "syt_hmi/wizard_pages.h"

#include <QWizard>

class CreateFromCADWizard : public QWizard {
  Q_OBJECT
public:
  CreateFromCADWizard(QWidget *parent = nullptr);
  ~CreateFromCADWizard(){};

private slots:
};
