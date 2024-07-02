/****************************************************************************
** Meta object code from reading C++ file 'acquisition_worker.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../include/acquisition_worker.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'acquisition_worker.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_acquisition_worker_t {
    QByteArrayData data[6];
    char stringdata0[69];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_acquisition_worker_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_acquisition_worker_t qt_meta_stringdata_acquisition_worker = {
    {
QT_MOC_LITERAL(0, 0, 18), // "acquisition_worker"
QT_MOC_LITERAL(1, 19, 14), // "image_captured"
QT_MOC_LITERAL(2, 34, 0), // ""
QT_MOC_LITERAL(3, 35, 5), // "image"
QT_MOC_LITERAL(4, 41, 14), // "error_occurred"
QT_MOC_LITERAL(5, 56, 12) // "errorMessage"

    },
    "acquisition_worker\0image_captured\0\0"
    "image\0error_occurred\0errorMessage"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_acquisition_worker[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   24,    2, 0x06 /* Public */,
       4,    1,   27,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::QImage,    3,
    QMetaType::Void, QMetaType::QString,    5,

       0        // eod
};

void acquisition_worker::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<acquisition_worker *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->image_captured((*reinterpret_cast< QImage(*)>(_a[1]))); break;
        case 1: _t->error_occurred((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (acquisition_worker::*)(QImage );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&acquisition_worker::image_captured)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (acquisition_worker::*)(const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&acquisition_worker::error_occurred)) {
                *result = 1;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject acquisition_worker::staticMetaObject = { {
    QMetaObject::SuperData::link<QThread::staticMetaObject>(),
    qt_meta_stringdata_acquisition_worker.data,
    qt_meta_data_acquisition_worker,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *acquisition_worker::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *acquisition_worker::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_acquisition_worker.stringdata0))
        return static_cast<void*>(this);
    return QThread::qt_metacast(_clname);
}

int acquisition_worker::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 2)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 2;
    }
    return _id;
}

// SIGNAL 0
void acquisition_worker::image_captured(QImage _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void acquisition_worker::error_occurred(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
