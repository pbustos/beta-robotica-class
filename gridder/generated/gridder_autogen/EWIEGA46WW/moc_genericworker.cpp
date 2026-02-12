/****************************************************************************
** Meta object code from reading C++ file 'genericworker.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.2.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../genericworker.h"
#include <QtGui/qtextcursor.h>
#include <QScreen>
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'genericworker.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.2.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_GenericWorker_t {
    const uint offsetsAndSize[22];
    char stringdata0[116];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(offsetof(qt_meta_stringdata_GenericWorker_t, stringdata0) + ofs), len 
static const qt_meta_stringdata_GenericWorker_t qt_meta_stringdata_GenericWorker = {
    {
QT_MOC_LITERAL(0, 13), // "GenericWorker"
QT_MOC_LITERAL(14, 4), // "kill"
QT_MOC_LITERAL(19, 0), // ""
QT_MOC_LITERAL(20, 13), // "goToEmergency"
QT_MOC_LITERAL(34, 11), // "goToRestore"
QT_MOC_LITERAL(46, 10), // "initialize"
QT_MOC_LITERAL(57, 7), // "compute"
QT_MOC_LITERAL(65, 9), // "emergency"
QT_MOC_LITERAL(75, 7), // "restore"
QT_MOC_LITERAL(83, 16), // "hibernationCheck"
QT_MOC_LITERAL(100, 15) // "hibernationTick"

    },
    "GenericWorker\0kill\0\0goToEmergency\0"
    "goToRestore\0initialize\0compute\0emergency\0"
    "restore\0hibernationCheck\0hibernationTick"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_GenericWorker[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    0,   68,    2, 0x06,    1 /* Public */,
       3,    0,   69,    2, 0x06,    2 /* Public */,
       4,    0,   70,    2, 0x06,    3 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       5,    0,   71,    2, 0x0a,    4 /* Public */,
       6,    0,   72,    2, 0x0a,    5 /* Public */,
       7,    0,   73,    2, 0x0a,    6 /* Public */,
       8,    0,   74,    2, 0x0a,    7 /* Public */,
       9,    0,   75,    2, 0x0a,    8 /* Public */,
      10,    0,   76,    2, 0x0a,    9 /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void GenericWorker::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<GenericWorker *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->kill(); break;
        case 1: _t->goToEmergency(); break;
        case 2: _t->goToRestore(); break;
        case 3: _t->initialize(); break;
        case 4: _t->compute(); break;
        case 5: _t->emergency(); break;
        case 6: _t->restore(); break;
        case 7: _t->hibernationCheck(); break;
        case 8: _t->hibernationTick(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (GenericWorker::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&GenericWorker::kill)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (GenericWorker::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&GenericWorker::goToEmergency)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (GenericWorker::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&GenericWorker::goToRestore)) {
                *result = 2;
                return;
            }
        }
    }
    (void)_a;
}

const QMetaObject GenericWorker::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_GenericWorker.offsetsAndSize,
    qt_meta_data_GenericWorker,
    qt_static_metacall,
    nullptr,
qt_incomplete_metaTypeArray<qt_meta_stringdata_GenericWorker_t
, QtPrivate::TypeAndForceComplete<GenericWorker, std::true_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>
, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>


>,
    nullptr
} };


const QMetaObject *GenericWorker::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *GenericWorker::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_GenericWorker.stringdata0))
        return static_cast<void*>(this);
    if (!strcmp(_clname, "Ui_guiDlg"))
        return static_cast< Ui_guiDlg*>(this);
    return QWidget::qt_metacast(_clname);
}

int GenericWorker::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 9)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 9;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 9)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 9;
    }
    return _id;
}

// SIGNAL 0
void GenericWorker::kill()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void GenericWorker::goToEmergency()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void GenericWorker::goToRestore()
{
    QMetaObject::activate(this, &staticMetaObject, 2, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
