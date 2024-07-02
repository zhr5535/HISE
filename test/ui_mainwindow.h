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
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>
#include "scalview2d.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QFrame *top_frame;
    QComboBox *ComboDevices;
    QPushButton *bnEnum;
    QPushButton *bnOpenClose;
    QPushButton *bnStartStop;
    QPushButton *bnStartStopTracking;
    QPushButton *bnForwardTracking;
    QPushButton *bnPauseResumeTracking;
    QLineEdit *tbExposure;
    QLabel *lExposure;
    QLabel *lGain;
    QLineEdit *tbGain;
    QLabel *lFrameRate;
    QLineEdit *tbFrameRate;
    ScalableGraphicsView *graphicsView;
    QFrame *bottom_frame;
    QLabel *lXCoord;
    QLabel *lX;
    QLabel *lYCoord;
    QLabel *lY;
    QLineEdit *tbX;
    QLabel *lTrackPoint;
    QLineEdit *tbY;
    QPushButton *bnSetParam;
    QPushButton *bnGetParam;
    QPushButton *bnZoomIn;
    QPushButton *bnZoomOut;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(700, 600);
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MainWindow->sizePolicy().hasHeightForWidth());
        MainWindow->setSizePolicy(sizePolicy);
        MainWindow->setTabShape(QTabWidget::Rounded);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        top_frame = new QFrame(centralwidget);
        top_frame->setObjectName(QString::fromUtf8("top_frame"));
        top_frame->setGeometry(QRect(-2, 0, 704, 40));
        sizePolicy.setHeightForWidth(top_frame->sizePolicy().hasHeightForWidth());
        top_frame->setSizePolicy(sizePolicy);
        top_frame->setStyleSheet(QString::fromUtf8("QFrame {  \n"
"    border: 2px solid gray; /* \350\256\276\347\275\256\350\276\271\346\241\206\345\216\232\345\272\246\344\270\2722\345\203\217\347\264\240\357\274\214\351\242\234\350\211\262\344\270\272\351\273\221\350\211\262 */  \n"
"    background-color: rgb(190,190,190); /* \350\256\276\347\275\256\350\203\214\346\231\257\351\242\234\350\211\262\344\270\272\347\201\260\350\211\262 */  \n"
"}"));
        top_frame->setFrameShape(QFrame::StyledPanel);
        top_frame->setFrameShadow(QFrame::Raised);
        ComboDevices = new QComboBox(top_frame);
        ComboDevices->setObjectName(QString::fromUtf8("ComboDevices"));
        ComboDevices->setGeometry(QRect(40, 7, 450, 25));
        bnEnum = new QPushButton(top_frame);
        bnEnum->setObjectName(QString::fromUtf8("bnEnum"));
        bnEnum->setGeometry(QRect(510, 10, 60, 20));
        bnEnum->setCheckable(false);
        bnEnum->setChecked(false);
        bnOpenClose = new QPushButton(top_frame);
        bnOpenClose->setObjectName(QString::fromUtf8("bnOpenClose"));
        bnOpenClose->setGeometry(QRect(630, 5, 60, 30));
        bnStartStop = new QPushButton(top_frame);
        bnStartStop->setObjectName(QString::fromUtf8("bnStartStop"));
        bnStartStop->setEnabled(true);
        bnStartStop->setGeometry(QRect(40, 7, 60, 25));
        bnStartStopTracking = new QPushButton(top_frame);
        bnStartStopTracking->setObjectName(QString::fromUtf8("bnStartStopTracking"));
        bnStartStopTracking->setEnabled(false);
        bnStartStopTracking->setGeometry(QRect(120, 7, 60, 25));
        bnForwardTracking = new QPushButton(top_frame);
        bnForwardTracking->setObjectName(QString::fromUtf8("bnForwardTracking"));
        bnForwardTracking->setEnabled(false);
        bnForwardTracking->setGeometry(QRect(250, 10, 60, 20));
        bnPauseResumeTracking = new QPushButton(top_frame);
        bnPauseResumeTracking->setObjectName(QString::fromUtf8("bnPauseResumeTracking"));
        bnPauseResumeTracking->setEnabled(false);
        bnPauseResumeTracking->setGeometry(QRect(190, 10, 60, 20));
        tbExposure = new QLineEdit(top_frame);
        tbExposure->setObjectName(QString::fromUtf8("tbExposure"));
        tbExposure->setEnabled(false);
        tbExposure->setGeometry(QRect(360, 10, 60, 20));
        lExposure = new QLabel(top_frame);
        lExposure->setObjectName(QString::fromUtf8("lExposure"));
        lExposure->setGeometry(QRect(330, 10, 30, 20));
        lExposure->setAlignment(Qt::AlignCenter);
        lGain = new QLabel(top_frame);
        lGain->setObjectName(QString::fromUtf8("lGain"));
        lGain->setGeometry(QRect(430, 10, 30, 20));
        lGain->setAlignment(Qt::AlignCenter);
        tbGain = new QLineEdit(top_frame);
        tbGain->setObjectName(QString::fromUtf8("tbGain"));
        tbGain->setEnabled(false);
        tbGain->setGeometry(QRect(460, 10, 60, 20));
        lFrameRate = new QLabel(top_frame);
        lFrameRate->setObjectName(QString::fromUtf8("lFrameRate"));
        lFrameRate->setGeometry(QRect(530, 10, 30, 20));
        lFrameRate->setAlignment(Qt::AlignCenter);
        tbFrameRate = new QLineEdit(top_frame);
        tbFrameRate->setObjectName(QString::fromUtf8("tbFrameRate"));
        tbFrameRate->setEnabled(false);
        tbFrameRate->setGeometry(QRect(560, 10, 60, 20));
        graphicsView = new ScalableGraphicsView(centralwidget);
        graphicsView->setObjectName(QString::fromUtf8("graphicsView"));
        graphicsView->setGeometry(QRect(40, 50, 612, 512));
        sizePolicy.setHeightForWidth(graphicsView->sizePolicy().hasHeightForWidth());
        graphicsView->setSizePolicy(sizePolicy);
        bottom_frame = new QFrame(centralwidget);
        bottom_frame->setObjectName(QString::fromUtf8("bottom_frame"));
        bottom_frame->setGeometry(QRect(0, 575, 700, 25));
        bottom_frame->setStyleSheet(QString::fromUtf8("QFrame {   \n"
"    background-color: white; /* \350\256\276\347\275\256\350\203\214\346\231\257\351\242\234\350\211\262\344\270\272\347\231\275\350\211\262 */  \n"
"}"));
        bottom_frame->setFrameShape(QFrame::StyledPanel);
        bottom_frame->setFrameShadow(QFrame::Raised);
        lXCoord = new QLabel(bottom_frame);
        lXCoord->setObjectName(QString::fromUtf8("lXCoord"));
        lXCoord->setGeometry(QRect(40, 2, 60, 20));
        lXCoord->setAutoFillBackground(false);
        lXCoord->setTextFormat(Qt::AutoText);
        lXCoord->setScaledContents(false);
        lXCoord->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        lX = new QLabel(bottom_frame);
        lX->setObjectName(QString::fromUtf8("lX"));
        lX->setGeometry(QRect(20, 2, 21, 20));
        lYCoord = new QLabel(bottom_frame);
        lYCoord->setObjectName(QString::fromUtf8("lYCoord"));
        lYCoord->setGeometry(QRect(130, 2, 60, 20));
        lY = new QLabel(bottom_frame);
        lY->setObjectName(QString::fromUtf8("lY"));
        lY->setGeometry(QRect(110, 2, 21, 20));
        tbX = new QLineEdit(bottom_frame);
        tbX->setObjectName(QString::fromUtf8("tbX"));
        tbX->setGeometry(QRect(310, 2, 41, 20));
        lTrackPoint = new QLabel(bottom_frame);
        lTrackPoint->setObjectName(QString::fromUtf8("lTrackPoint"));
        lTrackPoint->setGeometry(QRect(250, 2, 54, 21));
        tbY = new QLineEdit(bottom_frame);
        tbY->setObjectName(QString::fromUtf8("tbY"));
        tbY->setGeometry(QRect(370, 2, 41, 20));
        bnSetParam = new QPushButton(bottom_frame);
        bnSetParam->setObjectName(QString::fromUtf8("bnSetParam"));
        bnSetParam->setEnabled(false);
        bnSetParam->setGeometry(QRect(530, 2, 60, 20));
        bnGetParam = new QPushButton(bottom_frame);
        bnGetParam->setObjectName(QString::fromUtf8("bnGetParam"));
        bnGetParam->setEnabled(false);
        bnGetParam->setGeometry(QRect(460, 2, 60, 20));
        bnZoomIn = new QPushButton(centralwidget);
        bnZoomIn->setObjectName(QString::fromUtf8("bnZoomIn"));
        bnZoomIn->setGeometry(QRect(617, 492, 35, 35));
        QFont font;
        font.setPointSize(18);
        bnZoomIn->setFont(font);
        bnZoomOut = new QPushButton(centralwidget);
        bnZoomOut->setObjectName(QString::fromUtf8("bnZoomOut"));
        bnZoomOut->setGeometry(QRect(617, 527, 35, 35));
        bnZoomOut->setFont(font);
        MainWindow->setCentralWidget(centralwidget);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        bnEnum->setText(QCoreApplication::translate("MainWindow", "Search", nullptr));
        bnOpenClose->setText(QCoreApplication::translate("MainWindow", "Open", nullptr));
        bnStartStop->setText(QCoreApplication::translate("MainWindow", "Start", nullptr));
        bnStartStopTracking->setText(QCoreApplication::translate("MainWindow", "Track", nullptr));
        bnForwardTracking->setText(QCoreApplication::translate("MainWindow", "Forward", nullptr));
        bnPauseResumeTracking->setText(QCoreApplication::translate("MainWindow", "Pause", nullptr));
        lExposure->setText(QCoreApplication::translate("MainWindow", "Expo", nullptr));
        lGain->setText(QCoreApplication::translate("MainWindow", "Gain", nullptr));
        lFrameRate->setText(QCoreApplication::translate("MainWindow", "FPS", nullptr));
        lXCoord->setText(QCoreApplication::translate("MainWindow", "0", nullptr));
        lX->setText(QCoreApplication::translate("MainWindow", "x:", nullptr));
        lYCoord->setText(QCoreApplication::translate("MainWindow", "0", nullptr));
        lY->setText(QCoreApplication::translate("MainWindow", "y:", nullptr));
        tbX->setText(QCoreApplication::translate("MainWindow", "1224", nullptr));
        lTrackPoint->setText(QCoreApplication::translate("MainWindow", "\350\277\275\350\270\252\347\202\271: ", nullptr));
        tbY->setText(QCoreApplication::translate("MainWindow", "1024", nullptr));
        bnSetParam->setText(QCoreApplication::translate("MainWindow", "\350\256\276\347\275\256\345\217\202\346\225\260", nullptr));
        bnGetParam->setText(QCoreApplication::translate("MainWindow", "\350\216\267\345\217\226\345\217\202\346\225\260", nullptr));
        bnZoomIn->setText(QCoreApplication::translate("MainWindow", "+", nullptr));
        bnZoomOut->setText(QCoreApplication::translate("MainWindow", "-", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
