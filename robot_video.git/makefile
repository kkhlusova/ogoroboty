RPI_HOST = pi@172.20.10.2 
SRC = main.cpp
TARGET = robot_video

$(TARGET): $(SRC)
    ssh $(RPI_HOST) "g++ -o $(TARGET) $(SRC) `pkg-config --cflags --libs Qt5Core Qt5Gui Qt5Widgets Qt5Multimedia Qt5MultimediaWidgets gstreamer-1.0`"

deploy: $(TARGET)
    scp $(TARGET) $(RPI_HOST):~/
    ssh $(RPI_HOST) "./$(TARGET)"
