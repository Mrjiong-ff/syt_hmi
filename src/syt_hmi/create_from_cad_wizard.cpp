#include "syt_hmi/create_from_cad_wizard.h"

CreateFromCADWizard::CreateFromCADWizard(QWidget *parent) : QWizard(parent) {
  setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
  setModal(true);
  ChooseCADPage *choose_cad_page = new ChooseCADPage(parent);
  addPage(choose_cad_page);
}
