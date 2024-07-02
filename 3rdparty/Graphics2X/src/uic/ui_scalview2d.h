/********************************************************************************
** Form generated from reading UI file 'scalview2d.ui'
**
** Created by: Qt User Interface Compiler version 6.3.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SCALVIEW2D_H
#define UI_SCALVIEW2D_H

#include <QtCore/QVariant>
#include <QtGui/QIcon>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGraphicsView>

QT_BEGIN_NAMESPACE

class Ui_ScalableGraphicsView
{
public:

    void setupUi(QGraphicsView *ScalableGraphicsView)
    {
        if (ScalableGraphicsView->objectName().isEmpty())
            ScalableGraphicsView->setObjectName(QString::fromUtf8("ScalableGraphicsView"));
        ScalableGraphicsView->resize(400, 300);
        QFont font;
        font.setFamilies({QString::fromUtf8("Arial")});
        ScalableGraphicsView->setFont(font);
        ScalableGraphicsView->setMouseTracking(true);
        QIcon icon;
        icon.addFile(QString::fromUtf8("Resources/logos/mainwndlogo.png"), QSize(), QIcon::Normal, QIcon::Off);
        ScalableGraphicsView->setWindowIcon(icon);
        ScalableGraphicsView->setStyleSheet(QString::fromUtf8("background-color: rgb(4, 47, 63);"));
        ScalableGraphicsView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        ScalableGraphicsView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        QBrush brush(QColor(0, 48, 71, 255));
        brush.setStyle(Qt::NoBrush);
        ScalableGraphicsView->setBackgroundBrush(brush);
        ScalableGraphicsView->setDragMode(QGraphicsView::ScrollHandDrag);

        retranslateUi(ScalableGraphicsView);

        QMetaObject::connectSlotsByName(ScalableGraphicsView);
    } // setupUi

    void retranslateUi(QGraphicsView *ScalableGraphicsView)
    {
        ScalableGraphicsView->setWindowTitle(QCoreApplication::translate("ScalableGraphicsView", "Scalable Graphics View", nullptr));
    } // retranslateUi

};

namespace Ui {
    class ScalableGraphicsView: public Ui_ScalableGraphicsView {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SCALVIEW2D_H
