#-------------------------------------------------
#
# Project created by QtCreator 2021-02-08T13:19:24
#
#-------------------------------------------------

QT       += core gui gamepad network



greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport serialport

TARGET = EM_System_App
TEMPLATE = app

win32:DEFINES += _WINSOCKAPI_
# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS


# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


CONFIG += c++11

SOURCES += \
        callbacks.cpp \
        daq.cpp \
        gamepadmonitor.cpp \
        imageProcessing.cpp \
        magneticmathfuntions.cpp \
        main.cpp \
        mainwindow.cpp \
        qcustomplot.cpp \
        s826.cpp

HEADERS += \
        826api.h \
        callbacks.h \
        daq.h \
        gamepadmonitor.h \
        imageProcessing.hpp \
        magneticmathfunctions.h \
        mainwindow.h \
        qcustomplot.h \
        s826.h

FORMS += \
        mainwindow.ui

#LIBS += "C:\Program Files (x86)\Sensoray\826\API\x64\s826.lib"
#LIB += "C:\Program Files (x86)\Sensoray\826\API\x32\s826.lib"
#LIBS += "C:\Windows\System32\s826.dll"
#LIB += "C:\Windows\SysWOW64\s826.dll"

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target


## These are important for finding the libraries of the s826 board that are in the /lib/ part of the project.
#win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../lib/S826/sdk_826_win_3.3.9/s826_3.3.9/api/x64/ -ls826
#else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../lib/S826/sdk_826_win_3.3.9/s826_3.3.9/api/x64/ -ls826d
#INCLUDEPATH += $$PWD/../../lib/S826/sdk_826_win_3.3.9/s826_3.3.9/api/x64
#DEPENDPATH += $$PWD/../../lib/S826/sdk_826_win_3.3.9/s826_3.3.9/api/x64

# We were using LIB instead of LIBS so it didn't work at first.
LIBS += $$PWD/../../lib/S826/sdk_826_win_3.3.9/s826_3.3.9/api/x64/s826.lib
INCLUDEPATH += $$PWD/../../lib/S826/sdk_826_win_3.3.9/s826_3.3.9/api/x64


# Information for the OpenCV Libraries from Local Directories
#win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../lib/opencv/build/x64/vc15/lib/ -lopencv_world451
#else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../lib/opencv/build/x64/vc15/lib/ -lopencv_world451d

#INCLUDEPATH += $$PWD/../../lib/opencv/build/include
#DEPENDPATH += $$PWD/../../lib/opencv/build/include
#LIBS += -L"$$PWD/../../lib/opencv/build/x64/vc15/bin/" \
#    -lopencv_world451 \
#    -lopencv_world451d


# Information for the OpenCV Libraries from System Directories
win32:CONFIG(release, debug|release): LIBS += -LC:/OpenCV-4.5.1/opencv/build/x64/vc15/lib/ -lopencv_world451
else:win32:CONFIG(debug, debug|release): LIBS += -LC:/OpenCV-4.5.1/opencv/build/x64/vc15/lib/ -lopencv_world451d

INCLUDEPATH += C:/OpenCV-4.5.1/opencv/build/include
DEPENDPATH += C:/OpenCV-4.5.1/opencv/build/include

LIBS += -L"C:/OpenCV-4.5.1/opencv/build/x64/vc15/bin/" \
    -lopencv_world451 \
    -lopencv_world451d

# NI DAQ Libraries
# For 32bit compiling
#LIBS += "C:/Program Files (x86)/National Instruments/NI-DAQ/DAQmx ANSI C Dev/lib/msvc/NIDAQmx.lib"
#INCLUDEPATH += "C:/Program Files (x86)/National Instruments/NI-DAQ/DAQmx ANSI C Dev/include"
# For 64 bit compiling
INCLUDEPATH +=  "C:\Program Files (x86)\National Instruments\Shared\ExternalCompilerSupport\C\include"
LIBS += "C:\Program Files (x86)\National Instruments\Shared\ExternalCompilerSupport\C\lib64\msvc/NIDAQmx.lib"

# Local 32bit libraries
#LIBS += "$$PWD/../../lib/USB-DAQ/DAQmx ANSI C Dev/x32/msvc/NIDAQmx.lib"
#INCLUDEPATH += "$$PWD/../../lib/USB-DAQ/DAQmx ANSI C Dev/x32/include"
# Local 64 bit libraries
#LIBS += "$$PWD/../../lib/USB-DAQ/DAQmx ANSI C Dev/x64/msvc/NIDAQmx.lib"
#INCLUDEPATH += "$$PWD/../../lib/USB-DAQ/DAQmx ANSI C Dev/x64/include"
LIBS += ws2_32.lib
LIBS += -lws2_32
