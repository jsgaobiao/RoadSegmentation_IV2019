TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        main.cpp \
    utils.cpp \
    MyCalib.cpp \
    Calculation.cpp \
    CAL_TRAN.cpp

HEADERS += \
    utils.h \
    CAL_MAIN.H

INCLUDEPATH += /usr/local/include
LIBS += -L/usr/local/lib/ -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc -lopencv_videoio -lopencv_viz
