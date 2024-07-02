/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../include/mainwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[14];
    char stringdata0[278];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow_t qt_meta_stringdata_MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MainWindow"
QT_MOC_LITERAL(1, 11, 17), // "on_bnEnum_clicked"
QT_MOC_LITERAL(2, 29, 0), // ""
QT_MOC_LITERAL(3, 30, 22), // "on_bnOpenClose_clicked"
QT_MOC_LITERAL(4, 53, 22), // "on_bnStartStop_clicked"
QT_MOC_LITERAL(5, 76, 21), // "on_bnGetParam_clicked"
QT_MOC_LITERAL(6, 98, 21), // "on_bnSetParam_clicked"
QT_MOC_LITERAL(7, 120, 30), // "on_bnStartStopTracking_clicked"
QT_MOC_LITERAL(8, 151, 32), // "on_bnPauseResumeTracking_clicked"
QT_MOC_LITERAL(9, 184, 28), // "on_bnForwardTracking_clicked"
QT_MOC_LITERAL(10, 213, 19), // "on_bnZoomIn_clicked"
QT_MOC_LITERAL(11, 233, 20), // "on_bnZoomOut_clicked"
QT_MOC_LITERAL(12, 254, 17), // "update_live_scene"
QT_MOC_LITERAL(13, 272, 5) // "image"

    },
    "MainWindow\0on_bnEnum_clicked\0\0"
    "on_bnOpenClose_clicked\0on_bnStartStop_clicked\0"
    "on_bnGetParam_clicked\0on_bnSetParam_clicked\0"
    "on_bnStartStopTracking_clicked\0"
    "on_bnPauseResumeTracking_clicked\0"
    "on_bnForwardTracking_clicked\0"
    "on_bnZoomIn_clicked\0on_bnZoomOut_clicked\0"
    "update_live_scene\0image"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      11,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   69,    2, 0x0a /* Public */,
       3,    0,   70,    2, 0x0a /* Public */,
       4,    0,   71,    2, 0x0a /* Public */,
       5,    0,   72,    2, 0x0a /* Public */,
       6,    0,   73,    2, 0x0a /* Public */,
       7,    0,   74,    2, 0x0a /* Public */,
       8,    0,   75,    2, 0x0a /* Public */,
       9,    0,   76,    2, 0x0a /* Public */,
      10,    0,   77,    2, 0x0a /* Public */,
      11,    0,   78,    2, 0x0a /* Public */,
      12,    1,   79,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QImage,   13,

       0        // eod
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<MainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->on_bnEnum_clicked(); break;
        case 1: _t->on_bnOpenClose_clicked(); break;
        case 2: _t->on_bnStartStop_clicked(); break;
        case 3: _t->on_bnGetParam_clicked(); break;
        case 4: _t->on_bnSetParam_clicked(); break;
        case 5: _t->on_bnStartStopTracking_clicked(); break;
        case 6: _t->on_bnPauseResumeTracking_clicked(); break;
        case 7: _t->on_bnForwardTracking_clicked(); break;
        case 8: _t->on_bnZoomIn_clicked(); break;
        case 9: _t->on_bnZoomOut_clicked(); break;
        case 10: _t->update_live_scene((*reinterpret_cast< const QImage(*)>(_a[1]))); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject MainWindow::staticMetaObject = { {
    QMetaObject::SuperData::link<QMainWindow::staticMetaObject>(),
    qt_meta_stringdata_MainWindow.data,
    qt_meta_data_MainWindow,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 11)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 11;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 11)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 11;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
