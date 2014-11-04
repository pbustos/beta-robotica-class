/****************************************************************************
** Meta object code from reading C++ file 'osgview.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../../../../../../Classes/osgviewer/osgview.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'osgview.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_OsgView[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       6,       // signalCount

 // signals: signature, parameters, type, tag, flags
      13,    9,    8,    8, 0x05,
      36,   34,    8,    8, 0x05,
      57,   34,    8,    8, 0x05,
      79,   34,    8,    8, 0x05,
      98,   96,    8,    8, 0x05,
     116,   96,    8,    8, 0x05,

       0        // eod
};

static const char qt_meta_stringdata_OsgView[] = {
    "OsgView\0\0vec\0newWorld3DCoor(QVec)\0p\0"
    "newLeftCoor(QPointF)\0newRightCoor(QPointF)\0"
    "endCoor(QPointF)\0t\0keyPress(QString)\0"
    "keyRelease(QString)\0"
};

void OsgView::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        OsgView *_t = static_cast<OsgView *>(_o);
        switch (_id) {
        case 0: _t->newWorld3DCoor((*reinterpret_cast< const QVec(*)>(_a[1]))); break;
        case 1: _t->newLeftCoor((*reinterpret_cast< QPointF(*)>(_a[1]))); break;
        case 2: _t->newRightCoor((*reinterpret_cast< QPointF(*)>(_a[1]))); break;
        case 3: _t->endCoor((*reinterpret_cast< QPointF(*)>(_a[1]))); break;
        case 4: _t->keyPress((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 5: _t->keyRelease((*reinterpret_cast< QString(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData OsgView::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject OsgView::staticMetaObject = {
    { &QGLWidget::staticMetaObject, qt_meta_stringdata_OsgView,
      qt_meta_data_OsgView, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &OsgView::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *OsgView::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *OsgView::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_OsgView))
        return static_cast<void*>(const_cast< OsgView*>(this));
    if (!strcmp(_clname, "osgViewer::Viewer"))
        return static_cast< osgViewer::Viewer*>(const_cast< OsgView*>(this));
    return QGLWidget::qt_metacast(_clname);
}

int OsgView::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QGLWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    }
    return _id;
}

// SIGNAL 0
void OsgView::newWorld3DCoor(const QVec & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void OsgView::newLeftCoor(QPointF _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void OsgView::newRightCoor(QPointF _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void OsgView::endCoor(QPointF _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void OsgView::keyPress(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void OsgView::keyRelease(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}
QT_END_MOC_NAMESPACE
