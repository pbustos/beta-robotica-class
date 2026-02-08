/****************************************************************************
** Meta object code from reading C++ file 'abstract_graphic_viewer.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.4.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../../../../../robocomp/robocomp/classes/abstract_graphic_viewer/abstract_graphic_viewer.h"
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'abstract_graphic_viewer.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.4.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

#ifndef Q_CONSTINIT
#define Q_CONSTINIT
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
namespace {
struct qt_meta_stringdata_AbstractGraphicViewer_t {
    uint offsetsAndSizes[8];
    char stringdata0[22];
    char stringdata1[22];
    char stringdata2[1];
    char stringdata3[12];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(sizeof(qt_meta_stringdata_AbstractGraphicViewer_t::offsetsAndSizes) + ofs), len 
Q_CONSTINIT static const qt_meta_stringdata_AbstractGraphicViewer_t qt_meta_stringdata_AbstractGraphicViewer = {
    {
        QT_MOC_LITERAL(0, 21),  // "AbstractGraphicViewer"
        QT_MOC_LITERAL(22, 21),  // "new_mouse_coordinates"
        QT_MOC_LITERAL(44, 0),  // ""
        QT_MOC_LITERAL(45, 11)   // "right_click"
    },
    "AbstractGraphicViewer",
    "new_mouse_coordinates",
    "",
    "right_click"
};
#undef QT_MOC_LITERAL
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_AbstractGraphicViewer[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    1,   26,    2, 0x06,    1 /* Public */,
       3,    1,   29,    2, 0x06,    3 /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::QPointF,    2,
    QMetaType::Void, QMetaType::QPointF,    2,

       0        // eod
};

Q_CONSTINIT const QMetaObject AbstractGraphicViewer::staticMetaObject = { {
    QMetaObject::SuperData::link<QGraphicsView::staticMetaObject>(),
    qt_meta_stringdata_AbstractGraphicViewer.offsetsAndSizes,
    qt_meta_data_AbstractGraphicViewer,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_AbstractGraphicViewer_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<AbstractGraphicViewer, std::true_type>,
        // method 'new_mouse_coordinates'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<QPointF, std::false_type>,
        // method 'right_click'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<QPointF, std::false_type>
    >,
    nullptr
} };

void AbstractGraphicViewer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<AbstractGraphicViewer *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->new_mouse_coordinates((*reinterpret_cast< std::add_pointer_t<QPointF>>(_a[1]))); break;
        case 1: _t->right_click((*reinterpret_cast< std::add_pointer_t<QPointF>>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (AbstractGraphicViewer::*)(QPointF );
            if (_t _q_method = &AbstractGraphicViewer::new_mouse_coordinates; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (AbstractGraphicViewer::*)(QPointF );
            if (_t _q_method = &AbstractGraphicViewer::right_click; *reinterpret_cast<_t *>(_a[1]) == _q_method) {
                *result = 1;
                return;
            }
        }
    }
}

const QMetaObject *AbstractGraphicViewer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *AbstractGraphicViewer::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_AbstractGraphicViewer.stringdata0))
        return static_cast<void*>(this);
    return QGraphicsView::qt_metacast(_clname);
}

int AbstractGraphicViewer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QGraphicsView::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 2)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 2;
    }
    return _id;
}

// SIGNAL 0
void AbstractGraphicViewer::new_mouse_coordinates(QPointF _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void AbstractGraphicViewer::right_click(QPointF _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
