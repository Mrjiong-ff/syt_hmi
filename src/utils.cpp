//
// Created by jerry on 23-4-28.
//

#include "utils/utils.h"


int show_message_box(QWidget *p, QString text, int btn_num, QVector<QString> btn_text) {
    auto box = new QMessageBox(p);
    box->setWindowFlags(Qt::FramelessWindowHint | Qt::Dialog);
    box->setText(text);
    for (int i = 0; i < btn_num; ++i) {
        box->setButtonText(i + 1, btn_text[i]);
    }
    box->show();
    return box->exec();
}
