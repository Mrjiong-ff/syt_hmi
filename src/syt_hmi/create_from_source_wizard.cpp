#include "syt_hmi/create_from_source_wizard.h"

CreateFromSourceWizard::CreateFromSourceWizard(QWidget *parent) : QWizard(parent) {
  setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
  setModal(true);
}
