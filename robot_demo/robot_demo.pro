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
    movement_control.cpp

HEADERS  += mainwindow.h \
    irobotcreate.h \
    movement_control.h

FORMS    += mainwindow.ui
