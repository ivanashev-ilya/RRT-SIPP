#-------------------------------------------------
#
# Project created by QtCreator 2011-02-26T12:08:02
#
#-------------------------------------------------

TARGET = RRT_SIPP
CONFIG   += console
CONFIG   -= app_bundle
TEMPLATE = app
QMAKE_CXXFLAGS += -std=c++11 -O0 -Wall -Wextra

QMAKE_CXXFLAGS += -O0
QMAKE_CXXFLAGS -= -O1
QMAKE_CXXFLAGS -= -O2
QMAKE_CXXFLAGS -= -O3

win32 {
QMAKE_LFLAGS += -static -static-libgcc -static-libstdc++
}

INCLUDEPATH += C:/boost/boost_1_62_0/

SOURCES += \
    conflict_avoidance_table.cpp \
    dynamic_obstacles.cpp \
    lineofsight.cpp \
    rrt.cpp \
    rrt_sipp.cpp \
    rrt_star_sipp.cpp \
    tinyxml2.cpp \
    xmllogger.cpp \
    mission.cpp \
    map.cpp \
    config.cpp \
    main.cpp \
    environmentoptions.cpp

HEADERS += \
    conflict_avoidance_table.h \
    dynamic_obstacles.h \
    lineofsight.h \
    point.h \
    rrt.h \
    rrt_sipp.h \
    rrt_star_sipp.h \
    tinyxml2.h \
    node.h \
    gl_const.h \
    xmllogger.h \
    mission.h \
    map.h \
    ilogger.h \
    config.h \
    searchresult.h \
    environmentoptions.h

DISTFILES += \
    README.md
