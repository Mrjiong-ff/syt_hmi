// filtertextedit.cpp

#include "syt_plaintextedit/filtertextedit.h"
#include <QKeyEvent>
#include <QRegularExpression>

FilterTextEdit::FilterTextEdit(QWidget *parent) : QPlainTextEdit(parent) {
  layout_         = new QHBoxLayout(this);
  filterFrame     = new QFrame(this);
  upBtn_          = new QPushButton("↑", this);
  downBtn_        = new QPushButton("↓", this);
  filterLineEdit_ = new QLineEdit(this);
  label1_         = new QLabel("当前检索条目: ", this);
  numLabel_       = new QLabel(this);
  layout_->addWidget(filterLineEdit_);
  layout_->addWidget(label1_);
  layout_->addWidget(numLabel_);
  layout_->addWidget(upBtn_);
  layout_->addWidget(downBtn_);
  filterFrame->setLayout(layout_);
  filterFrame->move(parent->width(), parent->height());
  filterFrame->hide();

  connect(filterLineEdit_, &QLineEdit::textChanged, this, &FilterTextEdit::filterTextChanged);
  connect(filterLineEdit_, &QLineEdit::returnPressed, this, &FilterTextEdit::selectNextMatch);
  connect(filterLineEdit_, &QLineEdit::editingFinished, filterLineEdit_, &QLineEdit::selectAll);
  currentMatchIndex_ = -1;
}

void FilterTextEdit::keyPressEvent(QKeyEvent *event) {
  // ctrl + f 唤醒搜索窗口 esc 退出窗口
  if (event->modifiers() == Qt::ControlModifier && event->key() == Qt::Key_F) {
    if (!filterFrame->isVisible()) {
      filterFrame->show();
      filterFrame->setFocus();
      return;
    }

  } else if (event->key() == Qt::Key_Escape) {
    if (filterFrame->isVisible()) {
      filterFrame->hide();
      QTextCursor cursor = textCursor();
      cursor.clearSelection();
      setTextCursor(cursor);
    } else {
      return;
    }
  }
  QPlainTextEdit::keyPressEvent(event);
}

void FilterTextEdit::filterTextChanged(const QString &text) {
  findMatches(text);
  currentMatchIndex_ = -1;
  selectNextMatch();
}

void FilterTextEdit::selectNextMatch() {
  if (matches_.isEmpty()) {
    return;
  }

  currentMatchIndex_ = (currentMatchIndex_ + 1) % matches_.count();
  QTextCursor cursor = textCursor();
  cursor.setPosition(matches_.at(currentMatchIndex_).position());
  setTextCursor(cursor);
  ensureCursorVisible();
  highlightMatch(cursor);
}

void FilterTextEdit::selectPreviousMatch() {
  if (matches_.isEmpty()) {
    return;
  }
  currentMatchIndex_ = (currentMatchIndex_ - 1 + matches_.count()) % matches_.count();
  QTextCursor cursor = textCursor();
  cursor.setPosition(matches_.at(currentMatchIndex_).position());
  setTextCursor(cursor);
  ensureCursorVisible();
  highlightMatch(cursor);
}

void FilterTextEdit::findMatches(const QString &text) {
  QTextCursor cursor = document()->find(text);
  matches_.clear();
  while (!cursor.isNull()) {
    matches_.append(cursor);
    cursor = document()->find(text, cursor);
  }
}

void FilterTextEdit::highlightMatch(const QTextCursor &cursor) {
  QTextEdit::ExtraSelection selection;
  selection.format.setBackground(Qt::yellow);
  selection.format.setProperty(QTextFormat::FullWidthSelection, true);
  selection.cursor = cursor;
  setExtraSelections({selection});
}

void FilterTextEdit::mousePressEvent(QMouseEvent *e) {
  Q_UNUSED(e);
}
