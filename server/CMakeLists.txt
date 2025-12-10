cmake_minimum_required(VERSION 3.10)
project(RobotController)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Настройка пути к библиотекам
include_directories(include)

# Исходные файлы
set(SOURCES
    src/main.cpp
    src/RobotController.cpp
    src/Protocol.cpp
)

# Создание исполняемого файла
add_executable(robot_controller ${SOURCES})

# Настройки компиляции
target_compile_options(robot_controller PRIVATE -Wall -Wextra -O2)

# Установка для установки в систему
install(TARGETS robot_controller DESTINATION bin)