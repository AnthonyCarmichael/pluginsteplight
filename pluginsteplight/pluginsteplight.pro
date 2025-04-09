COMPUTREE += ctlibplugin ctlibclouds ctlibstep ctlibstepaddon ctlibio ctlibfilters ctlibaction ctlibstdactions ctlibmath ctliblas

MUST_USE_OPENCV = 1

include(../../computreev6/config/plugin_shared.pri)

TARGET = plug_steplight

QT += concurrent

QMAKE_CXXFLAGS += -openmp


HEADERS += \
    $$CT_PREFIX_LIB/ctlibplugin/pluginentryinterface.h\
    step/stl_stepcreategrid3d.h \
    step/stl_stepextractcurvesfromgrid3d.h \
    step/stl_stepfilterbyratio.h \
    step/stl_stepfiltergrid3d.h \
    step/stl_stepfiltergrid3dbyvalue.h \
    stl_grid3d.h \
    stl_grid3d.hpp \
    stl_grid3dbeamvisitor.h \
    stl_openactivecontours.h \
    stl_openactivecontours.hpp \
    stl_pluginentry.h \
    stl_steppluginmanager.h \
    stl_visitorgrid3dfastfilter.h \
    stl_visitorgrid3dsetvalue.h


SOURCES += \
    step/stl_stepcreategrid3d.cpp \
    step/stl_stepextractcurvesfromgrid3d.cpp \
    step/stl_stepfilterbyratio.cpp \
    step/stl_stepfiltergrid3d.cpp \
    step/stl_stepfiltergrid3dbyvalue.cpp \
    stl_grid3d.cpp \
    stl_grid3dbeamvisitor.cpp \
    stl_openactivecontours.cpp \
    stl_pluginentry.cpp \
    stl_steppluginmanager.cpp \
    stl_visitorgrid3dfastfilter.cpp \
    stl_visitorgrid3dsetvalue.cpp

