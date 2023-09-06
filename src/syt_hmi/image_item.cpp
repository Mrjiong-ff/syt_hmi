#include "syt_hmi/image_item.h"

ImageItem::ImageItem(const QImage &image) : scale_value_(1), scale_default_(1), is_clicked_(false) {
  pix_ = QPixmap::fromImage(image);
}

QRectF ImageItem::boundingRect() const {
  return QRectF(-qreal(pix_.width()) / 2, -qreal(pix_.height()) / 2, pix_.width(), pix_.height());
}

/**
 *@brief: 获取缩放比例
 */
qreal ImageItem::getScaleValue() const {
  return scale_value_;
}

/**
 *@brief: 重置位置和大小
 */
void ImageItem::resetItemPos() {
  scale_value_ = scale_default_;
  setScale(scale_default_);
  setPos(0, 0);
}

/**
 *@brief: 绘图事件
 */
void ImageItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
  painter->drawPixmap(-pix_.width() / 2, -pix_.height() / 2, pix_);
}

/**
 *@brief: 设置pix默认缩放倍率和最小缩放倍率
 */
void ImageItem::setQGraphicsViewWH(int width, int height) {
  int pix_width  = pix_.width();
  int pix_height = pix_.height();

  // default scale
  qreal scale_width  = width * 1.0 / pix_width;
  qreal scale_height = height * 1.0 / pix_height;
  scale_default_     = scale_width > scale_height ? scale_height : scale_width;

  // min scale
  qreal scale_width_min  = min_width_ * 1.0 / pix_width;
  qreal scale_height_min = min_height_ * 1.0 / pix_height;
  scale_min_             = 0.3;
  // scale_width_min > scale_height_min ? scale_height_min : scale_width_min;

  setScale(scale_default_);
  scale_value_ = scale_default_;
}

/**
 *@brief: 滚轮缩放事件
 */
void ImageItem::wheelEvent(QGraphicsSceneWheelEvent *event) {
  if ((event->delta() > 0) && (scale_value_ >= 2)) { // 设置放大倍数上限
    return;
  } else if ((event->delta() < 0) && (scale_value_ <= scale_min_)) { // 设置缩小倍数上限
    return;
  } else {
    qreal original_scale = scale_value_;
    if (event->delta() > 0) { // 滚轮前滚
      scale_value_ *= 1.1;    // 每次放大倍率
    } else {
      scale_value_ *= 0.9; // 缩小倍率
    }

    setScale(scale_value_);
    if (event->delta() > 0) {
      // 使缩放处于鼠标中心
      moveBy(-event->pos().x() * original_scale * 0.1, -event->pos().y() * original_scale * 0.1);
    } else {
      moveBy(event->pos().x() * original_scale * 0.1, event->pos().y() * original_scale * 0.1);
    }
  }
}

QVariant ImageItem::itemChange(GraphicsItemChange change, const QVariant &value) {
  if (change == ItemPositionChange && scene()) {
    // value is the new position.
    QPointF newPos = value.toPointF();
    QRectF rect    = sceneBoundingRect();
    qDebug() << rect;
    if (!rect.contains(newPos)) {
      // Keep the item inside the scene rect.
      newPos.setX(qMin(rect.right(), qMax(newPos.x(), rect.left())));
      newPos.setY(qMin(rect.bottom(), qMax(newPos.y(), rect.top())));
      return newPos;
    }
  }
  return QGraphicsItem::itemChange(change, value);
}

// 常规点击事件
void ImageItem::normalPressEvent(QGraphicsSceneMouseEvent *event) {
  if (event->buttons() & Qt::LeftButton) {
    start_pos_  = event->pos();
    is_clicked_ = true; // 记录按下状态
  } else if (event->buttons() & Qt::RightButton) {
    resetItemPos(); // 右键重置大小
  }
}

// 常规移动事件
void ImageItem::normalMoveEvent(QGraphicsSceneMouseEvent *event) {
  if (event->buttons() & Qt::LeftButton) {
    if (is_clicked_) {
      QPointF point = (event->pos() - start_pos_) * scale_value_;
      moveBy(point.x(), point.y());
    }
  }
}

// 常规释放事件
void ImageItem::normalReleaseEvent(QGraphicsSceneMouseEvent *event) {
  if (event->button() == Qt::LeftButton) {
    is_clicked_ = false; // 标记鼠标释放
  }
}

/**
 *@brief: 鼠标点击事件
 */
void ImageItem::mousePressEvent(QGraphicsSceneMouseEvent *event) {
  normalPressEvent(event);
}

/**
 *@brief: 鼠标移动事件
 */
void ImageItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event) {
  normalMoveEvent(event);
}

/**
 *@brief: 鼠标释放事件
 */
void ImageItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event) {
  normalReleaseEvent(event);
}
