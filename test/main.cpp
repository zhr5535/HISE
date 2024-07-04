#include <QApplication>
#include <QWidget>
#include <QVBoxLayout>
#include <QThread>
#include <QLocale>
#include <QTranslator>
#include "mainwindow.h"
#include <iostream>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    /* QFont font("Arial", 10); // 设置字体为Arial，字号为10
    a.setFont(font); */

    QTranslator translator;
    QLocale locale = QLocale::system();

    if (locale.language() == QLocale::Chinese)
    {
        // 中文环境使用默认界面
    }
    else
    {
        // 其他环境使用英文
        translator.load(QString(":/BasicDemo_zh_EN.qm")); // 选择翻译文件
        a.installTranslator(&translator);
    }

    MainWindow w;
    w.setWindowFlags(w.windowFlags()); // 禁止最大化

    w.show();

    return a.exec();
}