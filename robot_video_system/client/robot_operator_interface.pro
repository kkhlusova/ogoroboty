TEMPLATE = app
TARGET = robot_operator_interface
CONFIG += console c++11
CONFIG -= app_bundle

SOURCES += \
    robot_control_client.cpp \
    network_handler.cpp

HEADERS += \
    network_handler.h

QT +=
QMAKE_CXXFLAGS += -std=c++11 -pthread
LIBS += -pthread

# Указываем, что используем только стандартные библиотеки С++
# без дополнительных модулей Qt