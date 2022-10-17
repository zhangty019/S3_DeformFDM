/****************************************************************************
** Meta object code from reading C++ file 'MainWindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.11.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../MainWindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'MainWindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.11.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[19];
    char stringdata0[243];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow_t qt_meta_stringdata_MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MainWindow"
QT_MOC_LITERAL(1, 11, 20), // "doTimerShapeUpDeform"
QT_MOC_LITERAL(2, 32, 0), // ""
QT_MOC_LITERAL(3, 33, 4), // "open"
QT_MOC_LITERAL(4, 38, 4), // "save"
QT_MOC_LITERAL(5, 43, 13), // "saveSelection"
QT_MOC_LITERAL(6, 57, 13), // "readSelection"
QT_MOC_LITERAL(7, 71, 16), // "signalNavigation"
QT_MOC_LITERAL(8, 88, 4), // "flag"
QT_MOC_LITERAL(9, 93, 13), // "shiftToOrigin"
QT_MOC_LITERAL(10, 107, 10), // "updateTree"
QT_MOC_LITERAL(11, 118, 14), // "mouseMoveEvent"
QT_MOC_LITERAL(12, 133, 12), // "QMouseEvent*"
QT_MOC_LITERAL(13, 146, 5), // "event"
QT_MOC_LITERAL(14, 152, 30), // "on_pushButton_clearAll_clicked"
QT_MOC_LITERAL(15, 183, 19), // "on_treeView_clicked"
QT_MOC_LITERAL(16, 203, 11), // "QModelIndex"
QT_MOC_LITERAL(17, 215, 5), // "index"
QT_MOC_LITERAL(18, 221, 21) // "runShapeUpDeformation"

    },
    "MainWindow\0doTimerShapeUpDeform\0\0open\0"
    "save\0saveSelection\0readSelection\0"
    "signalNavigation\0flag\0shiftToOrigin\0"
    "updateTree\0mouseMoveEvent\0QMouseEvent*\0"
    "event\0on_pushButton_clearAll_clicked\0"
    "on_treeView_clicked\0QModelIndex\0index\0"
    "runShapeUpDeformation"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      12,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   74,    2, 0x0a /* Public */,
       3,    0,   75,    2, 0x08 /* Private */,
       4,    0,   76,    2, 0x08 /* Private */,
       5,    0,   77,    2, 0x08 /* Private */,
       6,    0,   78,    2, 0x08 /* Private */,
       7,    1,   79,    2, 0x08 /* Private */,
       9,    0,   82,    2, 0x08 /* Private */,
      10,    0,   83,    2, 0x08 /* Private */,
      11,    1,   84,    2, 0x08 /* Private */,
      14,    0,   87,    2, 0x08 /* Private */,
      15,    1,   88,    2, 0x08 /* Private */,
      18,    0,   91,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    8,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 12,   13,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 16,   17,
    QMetaType::Void,

       0        // eod
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MainWindow *_t = static_cast<MainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->doTimerShapeUpDeform(); break;
        case 1: _t->open(); break;
        case 2: _t->save(); break;
        case 3: _t->saveSelection(); break;
        case 4: _t->readSelection(); break;
        case 5: _t->signalNavigation((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->shiftToOrigin(); break;
        case 7: _t->updateTree(); break;
        case 8: _t->mouseMoveEvent((*reinterpret_cast< QMouseEvent*(*)>(_a[1]))); break;
        case 9: _t->on_pushButton_clearAll_clicked(); break;
        case 10: _t->on_treeView_clicked((*reinterpret_cast< const QModelIndex(*)>(_a[1]))); break;
        case 11: _t->runShapeUpDeformation(); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow.data,
      qt_meta_data_MainWindow,  qt_static_metacall, nullptr, nullptr}
};


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
        if (_id < 12)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 12;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 12)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 12;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
