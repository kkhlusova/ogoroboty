#!/bin/bash

# Скрипт для сборки системы видеонаблюдения и управления роботом

echo "=== Building Robot Video System ==="

# Проверка наличия необходимых зависимостей
echo "Checking dependencies..."

if ! command -v cmake &> /dev/null; then
    echo "CMake is not installed. Installing..."
    sudo apt-get update
    sudo apt-get install -y cmake
fi

if ! command -v pkg-config &> /dev/null; then
    echo "pkg-config is not installed. Installing..."
    sudo apt-get install -y pkg-config
fi

# Проверка наличия GStreamer
if ! pkg-config --exists gstreamer-1.0; then
    echo "GStreamer is not installed. Installing..."
    sudo apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-good gstreamer1.0-tools
fi

# Проверка наличия Qt5
if ! pkg-config --exists Qt5Core; then
    echo "Qt5 is not installed. Installing..."
    sudo apt-get install -y qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools
fi

# Проверка наличия V4L2
if [ ! -d "/usr/include/libv4l2" ]; then
    echo "V4L2 development headers are not installed. Installing..."
    sudo apt-get install -y libv4l-dev v4l-utils
fi

echo "Dependencies check completed."

# Сборка серверной части (на Raspberry Pi)
echo "Building server video component..."
cd /workspace/robot_video_system/server_video

if [ ! -d "build" ]; then
    mkdir build
fi

cd build
cmake ..
make -j$(nproc)

if [ $? -eq 0 ]; then
    echo "Server video component built successfully!"
else
    echo "Error building server video component."
    exit 1
fi

# Сборка клиентской части (на ноутбуке)
echo "Building client component..."
cd /workspace/robot_video_system/client

if [ ! -d "build" ]; then
    mkdir build
fi

cd build
cmake ..
make -j$(nproc)

if [ $? -eq 0 ]; then
    echo "Client component built successfully!"
else
    echo "Error building client component."
    exit 1
fi

echo "=== Build completed successfully! ==="
echo "Server executable: /workspace/robot_video_system/server_video/build/video_stream_server"
echo "Client executable: /workspace/robot_video_system/client/build/robot_control_client"
echo ""
echo "To run server (on Raspberry Pi):"
echo "  cd /workspace/robot_video_system/server_video/build"
echo "  ./video_stream_server [notebook_ip] [video_port] [command_port] [camera_id] [arduino_address]"
echo ""
echo "To run client (on notebook):"
echo "  cd /workspace/robot_video_system/client/build"
echo "  ./robot_control_client"