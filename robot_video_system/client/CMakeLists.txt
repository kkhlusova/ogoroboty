cmake_minimum_required(VERSION 3.10)
project(RobotControlClient)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Qt5
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

# Find GStreamer
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0>=1.4)
pkg_check_modules(GST_VIDEO REQUIRED gstreamer-video-1.0>=1.4)

# Include directories
include_directories(
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${GSTREAMER_INCLUDE_DIRS}
    ${GST_VIDEO_INCLUDE_DIRS}
)

# Link directories
link_directories(${GSTREAMER_LIBRARY_DIRS})

# Compiler options
add_definitions(${Qt5Core_DEFINITIONS} ${Qt5Widgets_DEFINITIONS})
add_definitions(${GSTREAMER_CFLAGS_OTHER} ${GST_VIDEO_CFLAGS_OTHER})

# Source files
set(SOURCES
    robot_control_client.cpp
    network_handler.cpp
)

# Create executable
add_executable(robot_control_client ${SOURCES})

# Link libraries
target_link_libraries(robot_control_client
    Qt5::Core
    Qt5::Widgets
    ${GSTREAMER_LIBRARIES}
    ${GST_VIDEO_LIBRARIES}
)

# Compiler options
target_compile_options(robot_control_client PRIVATE -Wall -Wextra -O2)

# MOC generation (for Qt signals/slots)
set_property(TARGET robot_control_client PROPERTY CXX_STANDARD 11)
qt5_wrap_cpp(MOC_FILES robot_control_client.cpp)