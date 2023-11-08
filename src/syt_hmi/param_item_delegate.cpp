#include "syt_hmi/param_item_delegate.h"
#include <QComboBox>
#include <QKeyEvent>
#include <QLineEdit>
#include <QSpinBox>

QMap<QString, DATA_TYPE> str_data_type_map = {
    std::map<QString, DATA_TYPE>::value_type("int32", INT32),
    std::map<QString, DATA_TYPE>::value_type("uint32", UINT32),
    std::map<QString, DATA_TYPE>::value_type("int64", INT64),
    std::map<QString, DATA_TYPE>::value_type("uint64", UINT64),
    std::map<QString, DATA_TYPE>::value_type("float32", FLOAT32),
    std::map<QString, DATA_TYPE>::value_type("float64", FLOAT64),
    std::map<QString, DATA_TYPE>::value_type("char", CHAR),
    std::map<QString, DATA_TYPE>::value_type("uchar", UCHAR),
    std::map<QString, DATA_TYPE>::value_type("string", STRING),
};

//////////////////////////////////// 名称委托 ////////////////////////////////////
NameItemDelegate::NameItemDelegate(QObject *parent) : QStyledItemDelegate(parent) {}

QWidget *NameItemDelegate::createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const {
  Q_UNUSED(option);
  QLineEdit *name_line_edit = new QLineEdit(parent);
  QRegExp reg_exp("^(?![_0-9])[a-z_][a-z0-9_]*$");
  name_line_edit->setValidator(new QRegExpValidator(reg_exp, parent));
  name_line_edit->installEventFilter(const_cast<NameItemDelegate *>(this));
  return name_line_edit;
}

void NameItemDelegate::setEditorData(QWidget *editor, const QModelIndex &index) const {
  QLineEdit *name_line_edit = static_cast<QLineEdit *>(editor);
  name_line_edit->setText(index.data(Qt::EditRole).toString());
}

void NameItemDelegate::setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const {
  QLineEdit *name_line_edit = static_cast<QLineEdit *>(editor);
  QString new_text = name_line_edit->text();
  QString old_text = index.data(Qt::EditRole).toString();

  if (new_text.isEmpty()) {
    new_text = old_text;
  }
  model->setData(index, new_text, Qt::EditRole);
}

void NameItemDelegate::updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option, const QModelIndex &index) const {
  Q_UNUSED(index);
  editor->setGeometry(option.rect);
}

bool NameItemDelegate::eventFilter(QObject *obj, QEvent *event) {
  if (event->type() == QEvent::KeyPress) {
    QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
    if (keyEvent->key() == Qt::Key_Enter || keyEvent->key() == Qt::Key_Return) {
      QWidget *editor = static_cast<QWidget *>(obj);
      emit commitData(editor);
      emit closeEditor(editor);
      return true;
    }
  }
  return false;
}

//////////////////////////////////// 类型委托 ////////////////////////////////////
TypeItemDelegate::TypeItemDelegate(QObject *parent) : QStyledItemDelegate(parent) {}

QWidget *TypeItemDelegate::createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const {
  Q_UNUSED(option);
  if (index.model()->data(index, Qt::EditRole).toString().isEmpty()) {
    return NULL;
  }
  QComboBox *type_combo_box = new QComboBox(parent);
  type_combo_box->addItems(QStringList() << "int32"
                                         << "uint32"
                                         << "int64"
                                         << "uint64"
                                         << "float32"
                                         << "float64"
                                         << "char"
                                         << "uchar"
                                         << "string");
  return type_combo_box;
}

void TypeItemDelegate::setEditorData(QWidget *editor, const QModelIndex &index) const {
  QString value = index.model()->data(index, Qt::EditRole).toString();
  QComboBox *type_combo_box = static_cast<QComboBox *>(editor);
  type_combo_box->setCurrentText(value);
}

void TypeItemDelegate::setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const {
  QComboBox *type_combo_box = static_cast<QComboBox *>(editor);
  model->setData(index, type_combo_box->currentText(), Qt::EditRole);
}

void TypeItemDelegate::updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option, const QModelIndex &index) const {
  Q_UNUSED(index);
  editor->setGeometry(option.rect);
}

//////////////////////////////////// 长度委托 ////////////////////////////////////
LengthItemDelegate::LengthItemDelegate(QObject *parent) : QStyledItemDelegate(parent) {}

QWidget *LengthItemDelegate::createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const {
  Q_UNUSED(option);
  if (index.model()->data(index, Qt::EditRole).toString().isEmpty()) {
    return NULL;
  }
  QSpinBox *length_spin_box = new QSpinBox(parent);
  length_spin_box->setRange(1, 65535);
  length_spin_box->setSingleStep(1);
  return length_spin_box;
}

void LengthItemDelegate::setEditorData(QWidget *editor, const QModelIndex &index) const {
  QString value = index.model()->data(index, Qt::EditRole).toString();
  QSpinBox *length_spin_box = static_cast<QSpinBox *>(editor);
  length_spin_box->setValue(value.toInt());
}

void LengthItemDelegate::setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const {
  QSpinBox *length_spin_box = static_cast<QSpinBox *>(editor);
  model->setData(index, length_spin_box->value(), Qt::EditRole);
}

void LengthItemDelegate::updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option, const QModelIndex &index) const {
  Q_UNUSED(index);
  editor->setGeometry(option.rect);
}

//////////////////////////////////// 范围委托 ////////////////////////////////////
RangeItemDelegate::RangeItemDelegate(QObject *parent) : QStyledItemDelegate(parent) {}

QWidget *RangeItemDelegate::createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const {
  Q_UNUSED(option);
  if (index.model()->data(index, Qt::EditRole).toString().isEmpty()) {
    return NULL;
  }
  QComboBox *range_combo_box = new QComboBox(parent);
  range_combo_box->setEditable(true);
  range_combo_box->addItems(QStringList() << "pi"
                                          << "-pi"
                                          << "inf"
                                          << "-inf");
  QLineEdit *line_edit = range_combo_box->lineEdit();
  QRegExp reg_exp("^-?(?:\\d+\\.\\d+|\\d+|pi|-pi|inf|-inf)$");
  line_edit->setValidator(new QRegExpValidator(reg_exp, parent));
  return range_combo_box;
}

void RangeItemDelegate::setEditorData(QWidget *editor, const QModelIndex &index) const {
  QString value = index.model()->data(index, Qt::EditRole).toString();
  if (value.isEmpty()) {
    value = QString("0");
  }
  QComboBox *range_combo_box = static_cast<QComboBox *>(editor);
  range_combo_box->setCurrentText(value);
}

void RangeItemDelegate::setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const {
  QComboBox *range_combo_box = static_cast<QComboBox *>(editor);
  if (range_combo_box->currentText().isEmpty()) {
    range_combo_box->setCurrentText("0");
  }
  model->setData(index, range_combo_box->currentText(), Qt::EditRole);
}

void RangeItemDelegate::updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option, const QModelIndex &index) const {
  Q_UNUSED(index);
  editor->setGeometry(option.rect);
}

//////////////////////////////////// 数值委托 ////////////////////////////////////
ValueItemDelegate::ValueItemDelegate(QObject *parent) : QStyledItemDelegate(parent) {}

QWidget *ValueItemDelegate::createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const {
  Q_UNUSED(option);
  if (index.model()->data(index, Qt::EditRole).toString().isEmpty()) {
    return NULL;
  }
  QLineEdit *value_line_edit = new QLineEdit(parent);
  QRegExp reg_exp("^\\s*(-?inf|-?pi|-?\\d+(?:\\.\\d+)?)(?:\\s*,\\s*(-?inf|-?pi|-?\\d+(?:\\.\\d+)?))*\\s*$");
  value_line_edit->setValidator(new QRegExpValidator(reg_exp, parent));
  return value_line_edit;
}

void ValueItemDelegate::setEditorData(QWidget *editor, const QModelIndex &index) const {
  QString text = index.model()->data(index, Qt::EditRole).toString();
  if (text.isEmpty()) {
    text = QString("0");
  }
  QLineEdit *value_line_edit = static_cast<QLineEdit *>(editor);
  value_line_edit->setText(text);
}

void ValueItemDelegate::setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const {
  QLineEdit *value_line_edit = static_cast<QLineEdit *>(editor);
  if (value_line_edit->text().isEmpty()) {
    value_line_edit->setText("0");
  }
  model->setData(index, value_line_edit->text(), Qt::EditRole);
}

void ValueItemDelegate::updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option, const QModelIndex &index) const {
  Q_UNUSED(index);
  editor->setGeometry(option.rect);
}

//////////////////////////////////// 存储委托 ////////////////////////////////////
StoreItemDelegate::StoreItemDelegate(QObject *parent) : QStyledItemDelegate(parent) {}

QWidget *StoreItemDelegate::createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const {
  Q_UNUSED(option);
  if (index.model()->data(index, Qt::EditRole).toString().isEmpty()) {
    return NULL;
  }
  QSpinBox *store_spin_box = new QSpinBox(parent);
  store_spin_box->setRange(0, 1);
  store_spin_box->setSingleStep(1);
  return store_spin_box;
}

void StoreItemDelegate::setEditorData(QWidget *editor, const QModelIndex &index) const {
  QString value = index.model()->data(index, Qt::EditRole).toString();
  QSpinBox *store_spin_box = static_cast<QSpinBox *>(editor);
  store_spin_box->setValue(value.toInt());
}

void StoreItemDelegate::setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const {
  QSpinBox *store_spin_box = static_cast<QSpinBox *>(editor);
  model->setData(index, store_spin_box->value(), Qt::EditRole);
}

void StoreItemDelegate::updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option, const QModelIndex &index) const {
  Q_UNUSED(index);
  editor->setGeometry(option.rect);
}
