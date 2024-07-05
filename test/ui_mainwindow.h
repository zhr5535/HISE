/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.15.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "scalview2d.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionEnum;
    QAction *actionOpenClose;
    QAction *actionStartStop;
    QAction *actionStartStopTracking;
    QAction *actionCameraParam;
    QAction *actionPauseResumeTracking;
    QAction *actionForwardTracking;
    QWidget *centralwidget;
    QVBoxLayout *verticalLayout;
    ScalableGraphicsView *graphicsView;
    QToolBar *tbEnum;
    QToolBar *tbControl;
    QToolBar *tbTracking;
    QToolBar *tbCamParam;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(800, 600);
        actionEnum = new QAction(MainWindow);
        actionEnum->setObjectName(QString::fromUtf8("actionEnum"));
        actionOpenClose = new QAction(MainWindow);
        actionOpenClose->setObjectName(QString::fromUtf8("actionOpenClose"));
        actionStartStop = new QAction(MainWindow);
        actionStartStop->setObjectName(QString::fromUtf8("actionStartStop"));
        actionStartStopTracking = new QAction(MainWindow);
        actionStartStopTracking->setObjectName(QString::fromUtf8("actionStartStopTracking"));
        actionCameraParam = new QAction(MainWindow);
        actionCameraParam->setObjectName(QString::fromUtf8("actionCameraParam"));
        actionPauseResumeTracking = new QAction(MainWindow);
        actionPauseResumeTracking->setObjectName(QString::fromUtf8("actionPauseResumeTracking"));
        actionForwardTracking = new QAction(MainWindow);
        actionForwardTracking->setObjectName(QString::fromUtf8("actionForwardTracking"));
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        verticalLayout = new QVBoxLayout(centralwidget);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        graphicsView = new ScalableGraphicsView(centralwidget);
        graphicsView->setObjectName(QString::fromUtf8("graphicsView"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(graphicsView->sizePolicy().hasHeightForWidth());
        graphicsView->setSizePolicy(sizePolicy);

        verticalLayout->addWidget(graphicsView);

        MainWindow->setCentralWidget(centralwidget);
        tbEnum = new QToolBar(MainWindow);
        tbEnum->setObjectName(QString::fromUtf8("tbEnum"));
        QFont font;
        font.setPointSize(10);
        tbEnum->setFont(font);
        MainWindow->addToolBar(Qt::TopToolBarArea, tbEnum);
        tbControl = new QToolBar(MainWindow);
        tbControl->setObjectName(QString::fromUtf8("tbControl"));
        tbControl->setFont(font);
        MainWindow->addToolBar(Qt::TopToolBarArea, tbControl);
        tbTracking = new QToolBar(MainWindow);
        tbTracking->setObjectName(QString::fromUtf8("tbTracking"));
        MainWindow->addToolBar(Qt::TopToolBarArea, tbTracking);
        tbCamParam = new QToolBar(MainWindow);
        tbCamParam->setObjectName(QString::fromUtf8("tbCamParam"));
        MainWindow->addToolBar(Qt::TopToolBarArea, tbCamParam);

        tbEnum->addAction(actionEnum);
        tbEnum->addSeparator();
        tbControl->addAction(actionOpenClose);
        tbControl->addSeparator();
        tbControl->addAction(actionStartStop);
        tbControl->addSeparator();
        tbControl->addAction(actionStartStopTracking);
        tbControl->addSeparator();
        tbControl->addAction(actionCameraParam);
        tbControl->addSeparator();
        tbTracking->addAction(actionPauseResumeTracking);
        tbTracking->addSeparator();
        tbTracking->addAction(actionForwardTracking);
        tbTracking->addSeparator();

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        actionEnum->setText(QCoreApplication::translate("MainWindow", "Search", nullptr));
#if QT_CONFIG(tooltip)
        actionEnum->setToolTip(QCoreApplication::translate("MainWindow", "Search for available devices", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(shortcut)
        actionEnum->setShortcut(QCoreApplication::translate("MainWindow", "Ctrl+E", nullptr));
#endif // QT_CONFIG(shortcut)
        actionOpenClose->setText(QCoreApplication::translate("MainWindow", "Open", nullptr));
#if QT_CONFIG(tooltip)
        actionOpenClose->setToolTip(QCoreApplication::translate("MainWindow", "Open/Close selected camera", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(shortcut)
        actionOpenClose->setShortcut(QCoreApplication::translate("MainWindow", "Ctrl+P", nullptr));
#endif // QT_CONFIG(shortcut)
        actionStartStop->setText(QCoreApplication::translate("MainWindow", "Start", nullptr));
#if QT_CONFIG(tooltip)
        actionStartStop->setToolTip(QCoreApplication::translate("MainWindow", "Start/Stop acquisition", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(shortcut)
        actionStartStop->setShortcut(QCoreApplication::translate("MainWindow", "Ctrl+A", nullptr));
#endif // QT_CONFIG(shortcut)
        actionStartStopTracking->setText(QCoreApplication::translate("MainWindow", "Track", nullptr));
#if QT_CONFIG(tooltip)
        actionStartStopTracking->setToolTip(QCoreApplication::translate("MainWindow", "Start/Stop Tracking", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(shortcut)
        actionStartStopTracking->setShortcut(QCoreApplication::translate("MainWindow", "Ctrl+T", nullptr));
#endif // QT_CONFIG(shortcut)
        actionCameraParam->setText(QCoreApplication::translate("MainWindow", "CameraParam", nullptr));
#if QT_CONFIG(tooltip)
        actionCameraParam->setToolTip(QCoreApplication::translate("MainWindow", "set camera params", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(shortcut)
        actionCameraParam->setShortcut(QCoreApplication::translate("MainWindow", "Ctrl+O", nullptr));
#endif // QT_CONFIG(shortcut)
        actionPauseResumeTracking->setText(QCoreApplication::translate("MainWindow", "Pause", nullptr));
        actionForwardTracking->setText(QCoreApplication::translate("MainWindow", "Forward", nullptr));
        tbEnum->setWindowTitle(QCoreApplication::translate("MainWindow", "toolBar", nullptr));
        tbControl->setWindowTitle(QCoreApplication::translate("MainWindow", "toolBar_2", nullptr));
        tbTracking->setWindowTitle(QCoreApplication::translate("MainWindow", "toolBar", nullptr));
        tbCamParam->setWindowTitle(QCoreApplication::translate("MainWindow", "toolBar_2", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
