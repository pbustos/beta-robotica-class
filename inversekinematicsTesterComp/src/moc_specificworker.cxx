/****************************************************************************
** Meta object code from reading C++ file 'specificworker.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "specificworker.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'specificworker.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_SpecificWorker[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      32,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      16,   15,   15,   15, 0x0a,
      26,   15,   15,   15, 0x0a,
      33,   15,   15,   15, 0x0a,
      54,   15,   15,   15, 0x0a,
      61,   15,   15,   15, 0x0a,
      74,   15,   15,   15, 0x0a,
      88,   15,   15,   15, 0x0a,
      98,   15,   15,   15, 0x0a,
     108,   15,   15,   15, 0x0a,
     118,   15,   15,   15, 0x0a,
     128,   15,   15,   15, 0x0a,
     138,   15,   15,   15, 0x0a,
     151,   15,   15,   15, 0x0a,
     174,   15,   15,   15, 0x0a,
     190,   15,   15,   15, 0x0a,
     208,   15,   15,   15, 0x0a,
     225,   15,   15,   15, 0x0a,
     240,   15,   15,   15, 0x0a,
     253,   15,   15,   15, 0x0a,
     277,  268,   15,   15, 0x0a,
     293,   15,   15,   15, 0x0a,
     306,   15,   15,   15, 0x0a,
     320,   15,   15,   15, 0x0a,
     338,   15,   15,   15, 0x0a,
     354,   15,   15,   15, 0x0a,
     371,   15,   15,   15, 0x0a,
     384,   15,   15,   15, 0x0a,
     394,   15,   15,   15, 0x0a,
     413,   15,   15,   15, 0x0a,
     440,  432,   15,   15, 0x0a,
     472,   15,   15,   15, 0x2a,
     501,   15,   15,   15, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_SpecificWorker[] = {
    "SpecificWorker\0\0compute()\0stop()\0"
    "updateBodyPartsBox()\0send()\0enviarRCIS()\0"
    "enviarROBOT()\0boton_1()\0boton_2()\0"
    "boton_3()\0boton_4()\0boton_5()\0"
    "enviarHome()\0actualizarInnerModel()\0"
    "camareroZurdo()\0camareroDiestro()\0"
    "camareroCentro()\0puntosEsfera()\0"
    "puntosCubo()\0closeFingers()\0partName\0"
    "goHome(QString)\0abrirPinza()\0cerrarPinza()\0"
    "posicionInicial()\0posicionCoger()\0"
    "posicionSoltar()\0retroceder()\0goHomeR()\0"
    "izquierdoRecoger()\0izquierdoOfrecer()\0"
    "xoffset\0ballisticPartToAprilTarget(int)\0"
    "ballisticPartToAprilTarget()\0"
    "finePartToAprilTarget()\0"
};

void SpecificWorker::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        SpecificWorker *_t = static_cast<SpecificWorker *>(_o);
        switch (_id) {
        case 0: _t->compute(); break;
        case 1: _t->stop(); break;
        case 2: _t->updateBodyPartsBox(); break;
        case 3: _t->send(); break;
        case 4: _t->enviarRCIS(); break;
        case 5: _t->enviarROBOT(); break;
        case 6: _t->boton_1(); break;
        case 7: _t->boton_2(); break;
        case 8: _t->boton_3(); break;
        case 9: _t->boton_4(); break;
        case 10: _t->boton_5(); break;
        case 11: _t->enviarHome(); break;
        case 12: _t->actualizarInnerModel(); break;
        case 13: _t->camareroZurdo(); break;
        case 14: _t->camareroDiestro(); break;
        case 15: _t->camareroCentro(); break;
        case 16: _t->puntosEsfera(); break;
        case 17: _t->puntosCubo(); break;
        case 18: _t->closeFingers(); break;
        case 19: _t->goHome((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 20: _t->abrirPinza(); break;
        case 21: _t->cerrarPinza(); break;
        case 22: _t->posicionInicial(); break;
        case 23: _t->posicionCoger(); break;
        case 24: _t->posicionSoltar(); break;
        case 25: _t->retroceder(); break;
        case 26: _t->goHomeR(); break;
        case 27: _t->izquierdoRecoger(); break;
        case 28: _t->izquierdoOfrecer(); break;
        case 29: _t->ballisticPartToAprilTarget((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 30: _t->ballisticPartToAprilTarget(); break;
        case 31: _t->finePartToAprilTarget(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData SpecificWorker::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject SpecificWorker::staticMetaObject = {
    { &GenericWorker::staticMetaObject, qt_meta_stringdata_SpecificWorker,
      qt_meta_data_SpecificWorker, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &SpecificWorker::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *SpecificWorker::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *SpecificWorker::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_SpecificWorker))
        return static_cast<void*>(const_cast< SpecificWorker*>(this));
    return GenericWorker::qt_metacast(_clname);
}

int SpecificWorker::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = GenericWorker::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 32)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 32;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
