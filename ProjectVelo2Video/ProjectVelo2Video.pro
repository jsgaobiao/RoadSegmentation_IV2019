QT -= gui
QT += core

CONFIG += c++11 console
CONFIG -= app_bundle

SOURCES += \
        main.cpp \
    Calculation.cpp \
    CAL_TRAN.cpp \
    MyCalib.cpp \
    utils.cpp

HEADERS += \
    CAL_MAIN.H \
    utils.h


INCLUDEPATH += /usr/local/include
LIBS += -L/usr/local/lib/ -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc -lopencv_videoio -lopencv_viz
