//
// Created by jerry on 23-4-26.
//

#include <QApplication>
#include <QTextCodec>
#include <QFontDatabase>
#include <QSplashScreen>
#include <QMovie>
#include <rclcpp/rclcpp.hpp>
#include "syt_hmi/main_window.h"

int main(int argc, char **argv) {
    // 支持高分屏
    QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts);
    QGuiApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QApplication app(argc, argv);

    QPixmap pixmap(":m_bg/background/loading.gif");
    QSplashScreen splash(pixmap);
    splash.setWindowOpacity(1);  // 设置窗口透明度
    QLabel label(&splash);
    QMovie mv(":m_bg/background/loading.gif");
    label.setMovie(&mv);
    mv.start();
    splash.show();
    mv.setSpeed(mv.speed() * 2);

    splash.setCursor(Qt::BlankCursor);
    for (int i = 0; i < 32000; i += mv.speed()) {
        app.processEvents();  //使程序在显示启动画面的同时仍能响应鼠标等其他事件
        QThread::msleep(10);  // 延时
    }

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
    app.setStyleSheet(qss.readAll());
    qss.close();

    // todo 是否要有login ui？

    MainWindow m;
    m.showMaximized();

    splash.finish(&m);  //在主体对象初始化完成后结束启动动画

    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

    return app.exec();

}
