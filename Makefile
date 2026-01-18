# Makefile для компиляции программы управления роботом OmegaBot

CXX = g++
CXXFLAGS = -std=c++11 -Wall -Wextra -O2
TARGET = robot_controller
SRC = raspberry_controller.cpp
ARDUINO_FILE = robot_controller.cpp

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRC)

install: $(TARGET)
	sudo cp $(TARGET) /usr/local/bin/

clean:
	rm -f $(TARGET)

compile-arduino:
	@echo "Для загрузки программы на Arduino используйте Arduino IDE"
	@echo "Выберите файл $(ARDUINO_FILE) и загрузите его на Arduino Uno"

help:
	@echo "Цели сборки:"
	@echo "  make all       - Компиляция программы для Raspberry Pi"
	@echo "  make install   - Установка программы в систему"
	@echo "  make clean     - Удаление скомпилированных файлов"
	@echo "  make help      - Показать эту справку"

.PHONY: all clean install compile-arduino help
