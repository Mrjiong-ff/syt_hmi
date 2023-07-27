#include "syt_hmi/manual_input_param_wizard.h"

ManualInputParamWizard::ManualInputParamWizard(QWidget *parent) : QWizard(parent) {
  setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
  setModal(true);
}
