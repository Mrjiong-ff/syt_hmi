#ifndef GLOBALAPPLICATION_H
#define GLOBALAPPLICATION_H

#include "syt_hmi/main_window.h"
#include <QApplication>
#include <QCoreApplication>
#include <QObject>

class GlobalApplication;
#define app static_cast<GlobalApplication *>(QCoreApplication::instance())

class GlobalApplication : public QApplication {
  Q_OBJECT
public:
  GlobalApplication(int &argc, char **argv);
  ~GlobalApplication() override;

  MainWindow *getMainWindow();

  bool notify(QObject *obj, QEvent *event) override;
  void setWindowInstance(MainWindow *wnd);

  void switchApplicationLanguage(int index);

private:
  void initPage();

private:
  QVector<QString> qm_vec_;
  MainWindow *m_pWidget{Q_NULLPTR};
  QTranslator *m_pTranslator{Q_NULLPTR};
};

#endif // GLOBALAPPLICATION_H
