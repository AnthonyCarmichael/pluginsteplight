COMPUTREE += ctlibplugin ctlibclouds ctlibstep ctlibstepaddon ctlibio ctlibfilters ctlibaction ctlibstdactions ctlibmath ctliblas

MUST_USE_OPENCV = 1

include(../../computreev6/config/plugin_shared.pri)

TARGET = plug_steplight

QT += concurrent

QMAKE_CXXFLAGS += -openmp


HEADERS += \
    $$CT_PREFIX_LIB/ctlibplugin/pluginentryinterface.h\
    step/st_stepextractcurvesfromgrid3d.h \
    step/stl_stepcreategrid3d.h \
    step/stl_stepfiltergrid3d.h \
    step/stl_stepfiltergrid3dbyvalue.h \
    stl_grid3d.h \
    stl_grid3d.hpp \
    stl_grid3dbeamvisitor.h \
    stl_pluginentry.h \
    stl_steppluginmanager.h


SOURCES += \
    step/st_stepextractcurvesfromgrid3d.cpp \
    step/stl_stepcreategrid3d.cpp \
    step/stl_stepfiltergrid3d.cpp \
    step/stl_stepfiltergrid3dbyvalue.cpp \
    stl_grid3d.cpp \
    stl_grid3dbeamvisitor.cpp \
    stl_pluginentry.cpp \
    stl_steppluginmanager.cpp

