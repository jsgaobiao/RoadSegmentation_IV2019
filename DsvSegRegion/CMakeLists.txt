cmake_minimum_required(VERSION 2.8)
project(DsvSegRegion) # Your project name

set(CMAKE_CXX_STANDARD 11) # This is equal to QMAKE_CXX_FLAGS += -std=c++0x

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
# Instruct CMake to run uic automatically when needed.
set(CMAKE_AUTOUIC ON)

# This will find the Qt5 files. You will need a QT5_DIR env variable
set(OpenCV_DIR /usr/local/share/OpenCV)
find_package(OpenCV REQUIRED) # Equivalent of QT += widgets
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
include_directories(${OpenCV_INCLUDE_DIRS})

set(SOURCES main.cpp CalcPlane.cpp Calculation.cpp ContourSegger.cpp DmProc.cpp RmProc.cpp)
set(HEADERS ContourSegger.h define.h)

# This will create you executable
# add_executable(model_view_playground ${SOURCES} ${HEADERS})

ADD_LIBRARY(DsvSegRegion SHARED ${SOURCES})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
