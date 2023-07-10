//
// Created by jerry on 23-7-10.
//

#ifndef SYT_HMI_FILTERTEXTEDIT_H
#define SYT_HMI_FILTERTEXTEDIT_H

#include <iostream>
#include <QPlainTextEdit>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLabel>

class FilterTextEdit : public QPlainTextEdit {
Q_OBJECT

public:
    explicit FilterTextEdit(QWidget *parent = nullptr);

protected:
    void keyPressEvent(QKeyEvent *event) override;

    void mousePressEvent(QMouseEvent *e) override;

private slots:

    void filterTextChanged(const QString &text);

    void selectNextMatch();

    void selectPreviousMatch();

private:
    QFrame *filterFrame;
    QHBoxLayout *layout_;
    QPushButton *upBtn_;
    QPushButton *downBtn_;
    QLabel *label1_;
    QLabel *numLabel_;

    QLineEdit *filterLineEdit_;
    QList<QTextCursor> matches_;
    int currentMatchIndex_;

    void findMatches(const QString &text);

    void highlightMatch(const QTextCursor &cursor);
};


#endif //SYT_HMI_FILTERTEXTEDIT_H
