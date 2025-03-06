COMPUTREE += ctlibplugin ctlibclouds ctlibstep ctlibstepaddon ctlibio ctlibfilters ctlibaction ctlibstdactions ctlibmath ctliblas

MUST_USE_OPENCV = 1

include(../../computreev6/config/plugin_shared.pri)

TARGET = plug_steplight

QT += concurrent

QMAKE_CXXFLAGS += -openmp


HEADERS += \
    $$CT_PREFIX_LIB/ctlibplugin/pluginentryinterface.h\
    step/st_stepextractcurvesfrom3dgrid.h \
    step/stl_stepcreate3dgrid.h \
    step/stl_stepfilter3dgrid.h \
    stl_3dgrid.h \
    stl_3dgrid.hpp \
    stl_grid3dbeamvisitor.h \
    stl_pluginentry.h \
    stl_steppluginmanager.h


SOURCES += \
    step/st_stepextractcurvesfrom3dgrid.cpp \
    step/stl_stepcreate3dgrid.cpp \
    step/stl_stepfilter3dgrid.cpp \
    stl_3dgrid.cpp \
    stl_grid3dbeamvisitor.cpp \
    stl_pluginentry.cpp \
    stl_steppluginmanager.cpp

