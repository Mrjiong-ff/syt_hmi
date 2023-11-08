#include "syt_hmi/param_tree_view.h"
#include "utils/utils.h"
#include <QDebug>
#include <QStandardItemModel>

ParamTreeView::ParamTreeView(QWidget *parent) : QTreeView(parent) {}

void ParamTreeView::mousePressEvent(QMouseEvent *event) {
  QPoint p = event->pos();
  QModelIndex index = indexAt(p);
  if (!index.isValid()) {
    setCurrentIndex(QModelIndex());
    return;
  }

  QTreeView::mousePressEvent(event);
}

void ParamTreeView::keyPressEvent(QKeyEvent *event) {
  if (event->key() == Qt::Key_Delete) {
    QModelIndex index = currentIndex();

    auto getLevel = [=](QStandardItem *item) {
      int level = 0;
      while (item && item->parent()) {
        item = item->parent();
        level++;
      }
      return level;
    };

    // 在这里处理删除选中项的逻辑
    QStandardItemModel *model = dynamic_cast<QStandardItemModel *>(this->model());
    if (model) {
      QStandardItem *item = model->itemFromIndex(index);
      int item_level = getLevel(item);
      if (item_level == 0) {
        bool ret = showMessageBox(this, WARN, tr("确认删除当前组？"), 2, {tr("确认"), tr("取消")});
        if (ret == 0) {
          model->removeRow(index.row());
        }
      } else {
        item->parent()->removeRow(index.row());
      }
    }
  } else {
    QTreeView::keyPressEvent(event);
  }
}
