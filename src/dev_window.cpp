//
// Created by jerry on 23-5-4.
//

// You may need to build the project (run Qt uic code generator) to get "ui_dev_window.h" resolved

#include "syt_hmi/dev_window.h"


DevWindow::DevWindow(QWidget *parent) :
        QWidget(parent), ui(new Ui::DevWindow) {
    ui->setupUi(this);

    setWindowFlags(Qt::FramelessWindowHint | Qt::Dialog);

    connect(ui->closePushButton, &QPushButton::clicked, [=] { emit closeDevWindow(); close();});


}

DevWindow::~DevWindow() {
    delete ui;
}
