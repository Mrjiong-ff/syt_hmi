#pragma once

#include "syt_common/common.h"
#include <QDebug>
#include <QDir>
#include <QDragEnterEvent>
#include <QGraphicsItem>
#include <QGraphicsSceneWheelEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QPixmap>
#include <QPointF>
#include <QWidget>
#include <cmath>

class ImageItem : public QObject, public QGraphicsItem {
  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)
public:
  ImageItem() {}
  ImageItem(const QImage &image);
  ~ImageItem(){};

  bool is_clicked_ = false;

  QRectF boundingRect() const override;           // 外边框重载
  qreal getScaleValue() const;                    // 获取缩放系数
  void resetItemPos();                            // 重置位置大小
  void setQGraphicsViewWH(int width, int height); // 设置缩放系数

protected:
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
  void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
  void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;
  void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;
  void wheelEvent(QGraphicsSceneWheelEvent *event) override;

private:
  int min_width_  = 1500;
  int min_height_ = 750;

  // 鼠标事件变量
  qreal scale_min_;
  qreal scale_value_;
  qreal scale_default_;
  QPointF start_pos_;
  QPixmap pix_;

  void normalPressEvent(QGraphicsSceneMouseEvent *event);
  void normalMoveEvent(QGraphicsSceneMouseEvent *event);
  void normalReleaseEvent(QGraphicsSceneMouseEvent *event);
};
