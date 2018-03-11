/****************************************************************************
** Meta object code from reading C++ file 'main.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../main.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'main.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_Motion_t {
    QByteArrayData data[21];
    char stringdata0[198];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Motion_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Motion_t qt_meta_stringdata_Motion = {
    {
QT_MOC_LITERAL(0, 0, 6), // "Motion"
QT_MOC_LITERAL(1, 7, 11), // "updatedPWM1"
QT_MOC_LITERAL(2, 19, 0), // ""
QT_MOC_LITERAL(3, 20, 11), // "updatedPWM2"
QT_MOC_LITERAL(4, 32, 11), // "updatedPWM3"
QT_MOC_LITERAL(5, 44, 11), // "updatedPWM4"
QT_MOC_LITERAL(6, 56, 11), // "updatedPWM5"
QT_MOC_LITERAL(7, 68, 11), // "updatedPWM6"
QT_MOC_LITERAL(8, 80, 10), // "updatePWM1"
QT_MOC_LITERAL(9, 91, 3), // "tmp"
QT_MOC_LITERAL(10, 95, 10), // "updatePWM2"
QT_MOC_LITERAL(11, 106, 10), // "updatePWM3"
QT_MOC_LITERAL(12, 117, 10), // "updatePWM4"
QT_MOC_LITERAL(13, 128, 10), // "updatePWM5"
QT_MOC_LITERAL(14, 139, 10), // "updatePWM6"
QT_MOC_LITERAL(15, 150, 7), // "getPWM1"
QT_MOC_LITERAL(16, 158, 7), // "getPWM2"
QT_MOC_LITERAL(17, 166, 7), // "getPWM3"
QT_MOC_LITERAL(18, 174, 7), // "getPWM4"
QT_MOC_LITERAL(19, 182, 7), // "getPWM5"
QT_MOC_LITERAL(20, 190, 7) // "getPWM6"

    },
    "Motion\0updatedPWM1\0\0updatedPWM2\0"
    "updatedPWM3\0updatedPWM4\0updatedPWM5\0"
    "updatedPWM6\0updatePWM1\0tmp\0updatePWM2\0"
    "updatePWM3\0updatePWM4\0updatePWM5\0"
    "updatePWM6\0getPWM1\0getPWM2\0getPWM3\0"
    "getPWM4\0getPWM5\0getPWM6"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Motion[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      18,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       6,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,  104,    2, 0x06 /* Public */,
       3,    0,  105,    2, 0x06 /* Public */,
       4,    0,  106,    2, 0x06 /* Public */,
       5,    0,  107,    2, 0x06 /* Public */,
       6,    0,  108,    2, 0x06 /* Public */,
       7,    0,  109,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       8,    1,  110,    2, 0x0a /* Public */,
      10,    1,  113,    2, 0x0a /* Public */,
      11,    1,  116,    2, 0x0a /* Public */,
      12,    1,  119,    2, 0x0a /* Public */,
      13,    1,  122,    2, 0x0a /* Public */,
      14,    1,  125,    2, 0x0a /* Public */,
      15,    0,  128,    2, 0x0a /* Public */,
      16,    0,  129,    2, 0x0a /* Public */,
      17,    0,  130,    2, 0x0a /* Public */,
      18,    0,  131,    2, 0x0a /* Public */,
      19,    0,  132,    2, 0x0a /* Public */,
      20,    0,  133,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,    9,
    QMetaType::Void, QMetaType::Int,    9,
    QMetaType::Void, QMetaType::Int,    9,
    QMetaType::Void, QMetaType::Int,    9,
    QMetaType::Void, QMetaType::Int,    9,
    QMetaType::Void, QMetaType::Int,    9,
    QMetaType::Int,
    QMetaType::Int,
    QMetaType::Int,
    QMetaType::Int,
    QMetaType::Int,
    QMetaType::Int,

       0        // eod
};

void Motion::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Motion *_t = static_cast<Motion *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->updatedPWM1(); break;
        case 1: _t->updatedPWM2(); break;
        case 2: _t->updatedPWM3(); break;
        case 3: _t->updatedPWM4(); break;
        case 4: _t->updatedPWM5(); break;
        case 5: _t->updatedPWM6(); break;
        case 6: _t->updatePWM1((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 7: _t->updatePWM2((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 8: _t->updatePWM3((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 9: _t->updatePWM4((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 10: _t->updatePWM5((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 11: _t->updatePWM6((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 12: { int _r = _t->getPWM1();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        case 13: { int _r = _t->getPWM2();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        case 14: { int _r = _t->getPWM3();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        case 15: { int _r = _t->getPWM4();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        case 16: { int _r = _t->getPWM5();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        case 17: { int _r = _t->getPWM6();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (Motion::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Motion::updatedPWM1)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (Motion::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Motion::updatedPWM2)) {
                *result = 1;
                return;
            }
        }
        {
            typedef void (Motion::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Motion::updatedPWM3)) {
                *result = 2;
                return;
            }
        }
        {
            typedef void (Motion::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Motion::updatedPWM4)) {
                *result = 3;
                return;
            }
        }
        {
            typedef void (Motion::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Motion::updatedPWM5)) {
                *result = 4;
                return;
            }
        }
        {
            typedef void (Motion::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Motion::updatedPWM6)) {
                *result = 5;
                return;
            }
        }
    }
}

const QMetaObject Motion::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_Motion.data,
      qt_meta_data_Motion,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *Motion::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Motion::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_Motion.stringdata0))
        return static_cast<void*>(const_cast< Motion*>(this));
    return QObject::qt_metacast(_clname);
}

int Motion::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 18)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 18;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 18)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 18;
    }
    return _id;
}

// SIGNAL 0
void Motion::updatedPWM1()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}

// SIGNAL 1
void Motion::updatedPWM2()
{
    QMetaObject::activate(this, &staticMetaObject, 1, Q_NULLPTR);
}

// SIGNAL 2
void Motion::updatedPWM3()
{
    QMetaObject::activate(this, &staticMetaObject, 2, Q_NULLPTR);
}

// SIGNAL 3
void Motion::updatedPWM4()
{
    QMetaObject::activate(this, &staticMetaObject, 3, Q_NULLPTR);
}

// SIGNAL 4
void Motion::updatedPWM5()
{
    QMetaObject::activate(this, &staticMetaObject, 4, Q_NULLPTR);
}

// SIGNAL 5
void Motion::updatedPWM6()
{
    QMetaObject::activate(this, &staticMetaObject, 5, Q_NULLPTR);
}
QT_END_MOC_NAMESPACE
