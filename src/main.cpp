#include "syt_hmi/globalapplication.h"
#include "syt_hmi/main_window.h"
#include "syt_lib_crypt/syt_decryption_utils.h"
#include <QApplication>
#include <QFontDatabase>
#include <QMovie>
#include <QSplashScreen>
#include <QTextCodec>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  // syt::crypt::Decryption decryptor;
  // if (!decryptor.isAuthorized()) {
  //   return 0;
  // }

  std::string config_path = std::string(getenv("ENV_ROBOT_ETC")) + "/syt_hmi/syt_hmi.yaml";
  cv::FileStorage fs(config_path, cv::FileStorage::READ);

  int language;
  fs["language"] >> language;

  fs.release();

  // 支持高分屏
  QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts);
  QGuiApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
  // QApplication a(argc, argv);
  GlobalApplication a(argc, argv);
  a.switchApplicationLanguage(language);

  // QPixmap pixmap(":m_bg/background/loading.gif");
  // QSplashScreen splash(pixmap);
  // splash.setWindowOpacity(1); // 设置窗口透明度
  // QLabel label(&splash);
  // QMovie mv(":m_bg/background/loading.gif");
  // label.setMovie(&mv);
  // mv.start();
  // splash.show();
  // mv.setSpeed(mv.speed() * 2);

  // splash.setCursor(Qt::BlankCursor);
  // for (int i = 0; i < 32000; i += mv.speed()) {
  // a.processEvents(); // 使程序在显示启动画面的同时仍能响应鼠标等其他事件
  // QThread::msleep(10); // 延时
  //}

  rclcpp::init(argc, argv);

  // 字体
  QTextCodec *code = QTextCodec::codecForName("UTF-8");
  QTextCodec::setCodecForLocale(code);
  int font_id = QFontDatabase::addApplicationFont(":m_font/font/NotoSansSC-Regular.otf");
  if (font_id >= 0) {
    QString local_font = QFontDatabase::applicationFontFamilies(font_id).at(0);
    QFont font(local_font);
    QApplication::setFont(font);
  }

  // qss 相关
  QFile qss(":m_qss/syt_style.qss");
  qss.open(QFile::ReadOnly);
  a.setStyleSheet(qss.readAll());
  qss.close();

  MainWindow main_window;
  main_window.show();

  // splash.finish(&main_window); // 在主体对象初始化完成后结束启动动画

  a.connect(&a, SIGNAL(lastWindowClosed()), &a, SLOT(quit()));

  return a.exec();
}
