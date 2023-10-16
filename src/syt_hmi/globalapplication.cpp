#include "syt_hmi/globalapplication.h"
#include <QDebug>
#include <QKeyEvent>
#include <QTranslator>
GlobalApplication::GlobalApplication(int &argc, char **argv) : QApplication(argc, argv) {
  initPage();

  qm_vec_ = QVector<QString>({
      QString(":m_lang/lang/zh_CN.qm"),
      QString(":m_lang/lang/en_US.qm"),
  });
}

GlobalApplication::~GlobalApplication() {
  m_pWidget = Q_NULLPTR;
}

void GlobalApplication::initPage() {
}

bool GlobalApplication::notify(QObject *obj, QEvent *e) {
  if (e->type() == QEvent::MouseButtonPress || e->type() == QEvent::KeyPress) {

  } else if (e->type() == QEvent::MouseButtonRelease || e->type() == QEvent::KeyRelease) {
  }
  if (e->type() == QEvent::KeyPress) {
    auto *keyEvent = dynamic_cast<QKeyEvent *>(e);
    if (keyEvent == Q_NULLPTR) {
      return QApplication::notify(obj, e);
    }
    if (keyEvent->key() == Qt::Key_Apostrophe || // 全局禁用引号，防止数据库插入失败
        keyEvent->key() == Qt::Key_QuoteDbl ||
        keyEvent->key() == Qt::Key_Shift ||
        keyEvent->key() == Qt::Key_Backtab) {
      return true;
    }
  }
  return QApplication::notify(obj, e);
}

void GlobalApplication::setWindowInstance(MainWindow *wnd) {
  m_pWidget = wnd;
}

MainWindow *GlobalApplication::getMainWindow() {
  return m_pWidget;
}

void GlobalApplication::switchApplicationLanguage(int index) {
  if (Q_NULLPTR != m_pTranslator) {
    qApp->removeTranslator(m_pTranslator);
    delete m_pTranslator;
    m_pTranslator = Q_NULLPTR;
  }

  m_pTranslator = new QTranslator;
  bool flag = m_pTranslator->load(qm_vec_.at(index));

  if (!flag) {
    return;
  }

  qApp->installTranslator(m_pTranslator);
}
