TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        main.cpp \
    CalcPlane.cpp \
    Calculation.cpp \
    ContourSegger.cpp \
    DmProc.cpp \
    RmProc.cpp

HEADERS += \
    ContourSegger.h \
    define.h \
    P_DWDX_INFOn.hh \
    P_PointCloud2n.hh \
    rcsheadern.hh

INCLUDEPATH += /usr/local/include
LIBS += -L/usr/local/lib/ -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc -lopencv_videoio -lopencv_viz
