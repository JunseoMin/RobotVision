#-------------------------------------------------
#
# Project created by QtCreator 2014-09-29T17:12:01
#
#-------------------------------------------------
QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = robotVisionApp
TEMPLATE = app

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += main.cpp\
    calibrationZhang.cpp \
        mainframe.cpp \
    imageform.cpp \
    kfc.cpp \
    optima.cpp \
    rectification.cpp \
    AXZB.cpp

HEADERS  += mainframe.h \
    calibrationZhang.h \
    imageform.h \
    kfc.h \
    optima.h \
    rectification.h \
    utills.hpp \
    AXZB.hpp

FORMS    += mainframe.ui \
    imageform.ui

RESOURCES += \
    images/mainframe.qrc

INCLUDEPATH += /usr/include/eigen3\
            /usr/include/\
            /usr/local/include\
            /usr/local/lib\
            /usr/local/lib/cmake/\
            /usr/include/opencv4 \


LIBS += -L/usr/local/lib -lglog -lceres
CONFIG += link_pkgconfig
PKGCONFIG += opencv4

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
