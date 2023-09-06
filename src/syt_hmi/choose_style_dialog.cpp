#include "syt_hmi/choose_style_dialog.h"
#include "ui_choose_style_dialog.h"

ChooseStyleDialog::ChooseStyleDialog(QWidget *parent) : QDialog(parent), ui(new Ui::ChooseStyleDialog) {
  ui->setupUi(this);
  this->setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
  this->setModal(true); // 设置为模态 父对象控件不可选

  pop_menu_   = new QMenu(this);
  delete_act_ = new QAction("删除", this);
  pop_menu_->addAction(delete_act_);

  ui->style_path_btn->setParentEnabled(true);
  ui->style_path_btn->setForeEnabled(false);
  ui->style_path_btn->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");

  ui->return_btn->setParentEnabled(true);
  ui->return_btn->setForeEnabled(false);
  ui->return_btn->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");

  ui->confirm_btn->setParentEnabled(true);
  ui->confirm_btn->setForeEnabled(false);
  ui->confirm_btn->setStyleSheet("qproperty-press_color: rgba(0,0,100,0.5);");

  ui->icon_label->setPixmap(QPixmap(":m_icon/icon/style_layer.png").scaled(80, 80, Qt::AspectRatioMode::KeepAspectRatio, Qt::TransformationMode::SmoothTransformation));

  style_directory_ = QString(getenv("HOME")) + QDir::separator() + "style";
  ui->style_path_line_edit->setText(style_directory_);

  // 设置根目录为当前目录
  style_file_model_.setRootPath(style_directory_);

  // 设置过滤器，仅显示 .txt 文件，而不显示文件夹
  style_file_model_.setNameFilterDisables(false);
  style_file_model_.setNameFilters(QStringList() << "*.sty");
  style_file_model_.setFilter(QDir::Files);

  // 设置数据模型
  ui->style_file_list_view->setModel(&style_file_model_);

  // 设置ListView的显示模式
  ui->style_file_list_view->setRootIndex(style_file_model_.index(style_directory_));
  ui->style_file_list_view->show();

  waiting_spinner_widget_ = new WaitingSpinnerWidget(this);

  connect(ui->style_path_btn, SIGNAL(clicked()), this, SLOT(slotSetStylePath()));
  connect(ui->confirm_btn, &QPushButton::clicked, [=]() {
    waiting_spinner_widget_->start();
    emit signSetCurrentStyle(ui->style_path_line_edit->text(), ui->style_file_list_view->currentIndex().data().toString());
  });
  connect(ui->return_btn, &QPushButton::clicked, [=]() {
    this->reject();
  });

  connect(delete_act_, &QAction::triggered, this, &ChooseStyleDialog::deleteStyleFile);
  connect(ui->style_file_list_view, &QListView::customContextMenuRequested, this, &ChooseStyleDialog::showContextMenu);
}

ChooseStyleDialog::~ChooseStyleDialog() {
  delete ui;
}

void ChooseStyleDialog::slotSetStylePath() {
  style_directory_ = QFileDialog::getExistingDirectory(this, "请选择模板路径", style_directory_);
  if (!style_directory_.isEmpty()) {
    ui->style_path_line_edit->setText(style_directory_);
    // 设置根目录为当前目录
    style_file_model_.setRootPath(style_directory_);

    // 设置过滤器，仅显示 .sty 文件，而不显示文件夹
    style_file_model_.setNameFilterDisables(false);
    style_file_model_.setNameFilters(QStringList() << "*.sty");
    style_file_model_.setFilter(QDir::Files);

    // 设置数据模型
    ui->style_file_list_view->setModel(&style_file_model_);

    // 设置ListView的显示模式
    ui->style_file_list_view->setRootIndex(style_file_model_.index(style_directory_));
    ui->style_file_list_view->show();
  }
}

void ChooseStyleDialog::slotSetCurrentStyleFinish(bool result) {
  waiting_spinner_widget_->stop();
  if (result) {
    emit signSetCurrentStyleName(ui->style_file_list_view->model()->data(ui->style_file_list_view->currentIndex()).toString());
    this->accept();
  } else {
    showMessageBox(this, WARN, "设置样式失败！", 1, {"确认"});
  }
}

void ChooseStyleDialog::deleteStyleFile() {
  QString file_name = ui->style_file_list_view->model()->data(ui->style_file_list_view->currentIndex()).toString();
  int ret           = showMessageBox(this, WARN, "确认删除？", 2, {"确认", "取消"});

  switch (ret) {
  case 0:
    QFile::remove(style_directory_ + QDir::separator() + file_name);
    break;
  case 1:
    break;
  }
}

void ChooseStyleDialog::showContextMenu(const QPoint &pos) {
  Q_UNUSED(pos);
  pop_menu_->exec(QCursor::pos());
}
