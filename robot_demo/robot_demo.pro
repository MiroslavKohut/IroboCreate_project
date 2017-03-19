#-------------------------------------------------
#
# Project created by QtCreator 2017-01-13T15:11:20
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = robot_demo
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    irobotcreate.cpp \
    movementcontrol.cpp \
    mapping.cpp \
    rplidar.cpp

HEADERS  += mainwindow.h \
    irobotcreate.h \
    movementcontrol.h \
    mapping.h \
    rplidar.h \
    structures.h

FORMS    += mainwindow.ui
