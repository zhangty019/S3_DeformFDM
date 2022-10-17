# ----------------------------------------------------
# This file is generated by the Qt Visual Studio Tools.
# ------------------------------------------------------

TEMPLATE = app
TARGET = ShapeLab
QT += opengl
QT += widgets
QT += core gui opengl

OBJECTS_DIR += release
UI_DIR += ./GeneratedFiles


INCLUDEPATH += $$PWD/../ThirdPartyDependence/eigen3
INCLUDEPATH += $$PWD/../ThirdPartyDependence

HEADERS  +=MainWindow.h

#ui_MainWindow not included.
SOURCES += main.cpp\
MainWindow.cpp


FORMS += ./MainWindow.ui
RESOURCES += ShapeLab.qrc

#open gl lib
INCLUDEPATH += $$PWD/../ThirdPartyDependence/glut


win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../ThirdPartyDependence/glut -lglut32
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../ThirdPartyDependence/glut -lglut32

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../ThirdPartyDependence/glut -lOpenGL32
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../ThirdPartyDependence/glut -lOpenGL32

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../ThirdPartyDependence/glut -lGlU32
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../ThirdPartyDependence/glut -lGlU32



#QMeshLib
win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../QMeshLib/release/ -lQMeshLib
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../QMeshLib/debug/ -lQMeshLib
else:unix: LIBS += -L$$OUT_PWD/../QMeshLib/ -lQMeshLib

INCLUDEPATH += $$PWD/../QMeshLib
DEPENDPATH += $$PWD/../QMeshLib

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../QMeshLib/release/libQMeshLib.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../QMeshLib/debug/libQMeshLib.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../QMeshLib/release/QMeshLib.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../QMeshLib/debug/QMeshLib.lib
else:unix: PRE_TARGETDEPS += $$OUT_PWD/../QMeshLib/libQMeshLib.a

#GLK lib and pre-target.
win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../GLKLib/release/ -lGLKLib
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../GLKLib/debug/ -lGLKLib
else:unix: LIBS += -L$$OUT_PWD/../GLKLib/ -lGLKLib

INCLUDEPATH += $$PWD/../GLKLib
DEPENDPATH += $$PWD/../GLKLib

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../GLKLib/release/libGLKLib.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../GLKLib/debug/libGLKLib.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../GLKLib/release/GLKLib.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../GLKLib/debug/GLKLib.lib
else:unix: PRE_TARGETDEPS += $$OUT_PWD/../GLKLib/libGLKLib.a