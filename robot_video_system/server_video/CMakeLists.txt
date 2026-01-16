cmake_minimum_required(VERSION 3.10)
project(VideoStreamServer)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find GStreamer
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0>=1.4)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0>=1.4)

# Include directories
include_directories(${GSTREAMER_INCLUDE_DIRS} ${GST_APP_INCLUDE_DIRS})
link_directories(${GSTREAMER_LIBRARY_DIRS} ${GST_APP_LIBRARY_DIRS})

# Compiler options
add_definitions(${GSTREAMER_CFLAGS_OTHER} ${GST_APP_CFLAGS_OTHER})

# Source files
set(SOURCES
    main.cpp
    video_stream_server.cpp
    robot_command_server.cpp
)

# Headers
set(HEADERS
    robot_command_server.h
)

# Create executable
add_executable(video_stream_server ${SOURCES})

# Link libraries
target_link_libraries(video_stream_server 
    ${GSTREAMER_LIBRARIES} 
    ${GST_APP_LIBRARIES}
    pthread
)

# Compiler options
target_compile_options(video_stream_server PRIVATE -Wall -Wextra -O2)