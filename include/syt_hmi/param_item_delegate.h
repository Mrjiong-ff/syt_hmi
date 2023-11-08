#pragma once
#include <QMap>
#include <QStyledItemDelegate>

//////////////////////////////////// 名称委托 ////////////////////////////////////
class NameItemDelegate : public QStyledItemDelegate {
  Q_OBJECT
public:
  NameItemDelegate(QObject *parent = Q_NULLPTR);

  QWidget *createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const override;
  void setEditorData(QWidget *editor, const QModelIndex &index) const override;
  void setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const override;
  void updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option, const QModelIndex &index) const override;
  bool eventFilter(QObject *obj, QEvent *event) override;
};

//////////////////////////////////// 类型委托 ////////////////////////////////////
enum DATA_TYPE {
  INT32 = 0,
  UINT32 = 1,
  INT64 = 2,
  UINT64 = 3,
  FLOAT32 = 4,
  FLOAT64 = 5,
  CHAR = 6,
  UCHAR = 7,
  STRING = 8
};

extern QMap<QString, DATA_TYPE> str_data_type_map; 

class TypeItemDelegate : public QStyledItemDelegate {
  Q_OBJECT
public:
  TypeItemDelegate(QObject *parent = Q_NULLPTR);

  QWidget *createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const override;
  void setEditorData(QWidget *editor, const QModelIndex &index) const override;
  void setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const override;
  void updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option, const QModelIndex &index) const override;
};

//////////////////////////////////// 长度委托 ////////////////////////////////////
class LengthItemDelegate : public QStyledItemDelegate {
  Q_OBJECT
public:
  LengthItemDelegate(QObject *parent = Q_NULLPTR);

  QWidget *createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const override;
  void setEditorData(QWidget *editor, const QModelIndex &index) const override;
  void setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const override;
  void updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option, const QModelIndex &index) const override;
};

//////////////////////////////////// 范围委托 ////////////////////////////////////
class RangeItemDelegate : public QStyledItemDelegate {
  Q_OBJECT
public:
  RangeItemDelegate(QObject *parent = Q_NULLPTR);

  QWidget *createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const override;
  void setEditorData(QWidget *editor, const QModelIndex &index) const override;
  void setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const override;
  void updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option, const QModelIndex &index) const override;
};

//////////////////////////////////// 数值委托 ////////////////////////////////////
class ValueItemDelegate : public QStyledItemDelegate {
  Q_OBJECT
public:
  ValueItemDelegate(QObject *parent = Q_NULLPTR);

  QWidget *createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const override;
  void setEditorData(QWidget *editor, const QModelIndex &index) const override;
  void setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const override;
  void updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option, const QModelIndex &index) const override;
};

//////////////////////////////////// 存储委托 ////////////////////////////////////
class StoreItemDelegate : public QStyledItemDelegate {
  Q_OBJECT
public:
  StoreItemDelegate(QObject *parent = Q_NULLPTR);

  QWidget *createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const override;
  void setEditorData(QWidget *editor, const QModelIndex &index) const override;
  void setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const override;
  void updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option, const QModelIndex &index) const override;
};
