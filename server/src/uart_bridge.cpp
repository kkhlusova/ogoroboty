#include "uart_bridge.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <chrono>

UARTBridge::UARTBridge() {
    // Пустые callback-и по умолчанию
    sensorCallback = [](const SensorData&) {};
    ackCallback = [](CommandType, bool) {};
    errorCallback = [](uint8_t, const std::string&) {};
}

UARTBridge::~UARTBridge() {
    stop();
}

bool UARTBridge::initialize(const std::string& port, int baudrate) {
    std::lock_guard<std::mutex> lock(serialMutex);
    
    // Открытие последовательного порта
    serialFd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serialFd < 0) {
        std::cerr << "Failed to open serial port: " << port << std::endl;
        return false;
    }
    
    // Настройка параметров порта
    struct termios options;
    tcgetattr(serialFd, &options);
    
    // Установка скорости
    speed_t speed;
    switch (baudrate) {
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        default: speed = B115200; break;
    }
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);
    
    // 8N1
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    
    // Включение приема
    options.c_cflag |= (CLOCAL | CREAD);
    
    // Сырой ввод
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    
    // Настройка таймаутов
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = READ_TIMEOUT_MS / 100; // в десятых долях секунды
    
    // Применение настроек
    tcsetattr(serialFd, TCSANOW, &options);
    
    // Очистка буферов
    tcflush(serialFd, TCIOFLUSH);
    
    // Запуск потока чтения
    running = true;
    readerThread = std::make_unique<std::thread>(&UARTBridge::run, this);
    
    // Тест связи
    if (!sendPing()) {
        std::cerr << "Ping failed - Arduino not responding" << std::endl;
        return false;
    }
    
    std::cout << "UART bridge initialized successfully on " << port << std::endl;
    return true;
}

void UARTBridge::run() {
    std::vector<uint8_t> packet;
    
    while (running) {
        if (readPacket(packet)) {
            processPacket(packet);
        }
        
        // Небольшая пауза для предотвращения загрузки CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

bool UARTBridge::readPacket(std::vector<uint8_t>& packet) {
    std::lock_guard<std::mutex> lock(serialMutex);
    packet.clear();
    
    uint8_t buffer[256];
    ssize_t bytesRead;
    
    // Поиск заголовка
    bool foundHeader = false;
    while (!foundHeader && running) {
        bytesRead = read(serialFd, buffer, 1);
        if (bytesRead > 0) {
            if (buffer[0] == 0xAA) {
                bytesRead = read(serialFd, buffer + 1, 1);
                if (bytesRead > 0 && buffer[1] == 0x55) {
                    foundHeader = true;
                    packet.push_back(0xAA);
                    packet.push_back(0x55);
                }
            }
        } else {
            // Таймаут
            return false;
        }
    }
    
    if (!foundHeader) return false;
    
    // Чтение типа команды и длины
    bytesRead = read(serialFd, buffer, 2);
    if (bytesRead != 2) return false;
    
    packet.push_back(buffer[0]); // Type
    packet.push_back(buffer[1]); // Length
    
    uint8_t dataLength = buffer[1];
    
    // Чтение данных (если есть)
    if (dataLength > 0) {
        bytesRead = read(serialFd, buffer, dataLength);
        if (bytesRead != dataLength) return false;
        
        for (int i = 0; i < dataLength; i++) {
            packet.push_back(buffer[i]);
        }
    }
    
    // Чтение CRC
    bytesRead = read(serialFd, buffer, 1);
    if (bytesRead != 1) return false;
    packet.push_back(buffer[0]);
    
    // Чтение завершения
    bytesRead = read(serialFd, buffer, 2);
    if (bytesRead != 2) return false;
    
    if (buffer[0] != 0x55 || buffer[1] != 0xAA) {
        return false; // Неверное завершение
    }
    
    packet.push_back(0x55);
    packet.push_back(0xAA);
    
    return true;
}

uint8_t UARTBridge::calculateCRC(const uint8_t* data, size_t length) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (size_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void UARTBridge::processPacket(const std::vector<uint8_t>& packet) {
    if (packet.size() < 7) return; // Минимальный размер
    
    CommandType type = static_cast<CommandType>(packet[2]);
    uint8_t length = packet[3];
    
    // Проверка CRC
    uint8_t crcReceived = packet[4 + length];
    uint8_t crcCalculated = calculateCRC(&packet[2], length + 2);
    
    if (crcCalculated != crcReceived) {
        std::cerr << "CRC mismatch in received packet" << std::endl;
        return;
    }
    
    switch (type) {
        case CommandType::RESP_ACK: {
            if (length >= 2) {
                CommandType originalCmd = static_cast<CommandType>(packet[4]);
                bool success = packet[5] != 0;
                ackCallback(originalCmd, success);
            }
            break;
        }
        
        case CommandType::RESP_SENSOR_DATA: {
            if (length == sizeof(SensorData)) {
                SensorData data;
                memcpy(&data, &packet[4], sizeof(SensorData));
                sensorCallback(data);
            }
            break;
        }
        
        case CommandType::RESP_ERROR: {
            if (length >= 1) {
                uint8_t errorCode = packet[4];
                std::string message;
                if (length > 1) {
                    message.assign(reinterpret_cast<const char*>(&packet[5]), length - 1);
                }
                errorCallback(errorCode, message);
            }
            break;
        }
        
        default:
            std::cerr << "Unknown response type: " << static_cast<int>(type) << std::endl;
            break;
    }
}

bool UARTBridge::sendCommand(CommandType type, const uint8_t* data, size_t length) {
    if (!isConnected()) return false;
    
    std::vector<uint8_t> packet;
    packet.reserve(7 + length);
    
    // Заголовок
    packet.push_back(0xAA);
    packet.push_back(0x55);
    
    // Тип и длина
    packet.push_back(static_cast<uint8_t>(type));
    packet.push_back(static_cast<uint8_t>(length));
    
    // Данные
    if (data && length > 0) {
        for (size_t i = 0; i < length; i++) {
            packet.push_back(data[i]);
        }
    }
    
    // CRC
    uint8_t crc = calculateCRC(&packet[2], length + 2);
    packet.push_back(crc);
    
    // Завершение
    packet.push_back(0x55);
    packet.push_back(0xAA);
    
    // Отправка
    std::lock_guard<std::mutex> lock(serialMutex);
    ssize_t written = write(serialFd, packet.data(), packet.size());
    
    if (written != static_cast<ssize_t>(packet.size())) {
        std::cerr << "Failed to write complete packet to serial" << std::endl;
        return false;
    }
    
    tcdrain(serialFd); // Ждем отправки
    return true;
}

// Реализации команд
bool UARTBridge::sendPing() {
    return sendCommand(CommandType::CMD_PING);
}

bool UARTBridge::sendMove(uint8_t direction, uint8_t speed) {
    uint8_t data[2] = {direction, speed};
    return sendCommand(CommandType::CMD_MOVE, data, 2);
}

bool UARTBridge::sendTurn(int16_t angle) {
    uint8_t data[2] = {
        static_cast<uint8_t>((angle >> 8) & 0xFF),
        static_cast<uint8_t>(angle & 0xFF)
    };
    return sendCommand(CommandType::CMD_TURN, data, 2);
}

bool UARTBridge::sendStop() {
    return sendCommand(CommandType::CMD_STOP);
}

bool UARTBridge::requestSensorData() {
    return sendCommand(CommandType::CMD_GET_SENSORS);
}

bool UARTBridge::sendSetSpeed(uint8_t speed) {
    return sendCommand(CommandType::CMD_SET_SPEED, &speed, 1);
}

bool UARTBridge::sendBuzzer(uint16_t frequency) {
    uint8_t data[2] = {
        static_cast<uint8_t>((frequency >> 8) & 0xFF),
        static_cast<uint8_t>(frequency & 0xFF)
    };
    return sendCommand(CommandType::CMD_BUZZER, data, 2);
}

bool UARTBridge::sendScript(const std::vector<uint8_t>& script) {
    return sendCommand(CommandType::CMD_SCRIPT, script.data(), script.size());
}

bool UARTBridge::sendMoveUntilObstacle(uint8_t speed) {
    return sendCommand(CommandType::CMD_MOVE_UNTIL_OBSTACLE, &speed, 1);
}

void UARTBridge::registerSensorCallback(SensorCallback callback) {
    sensorCallback = callback;
}

void UARTBridge::registerAckCallback(AckCallback callback) {
    ackCallback = callback;
}

void UARTBridge::registerErrorCallback(ErrorCallback callback) {
    errorCallback = callback;
}

bool UARTBridge::isConnected() const {
    return serialFd >= 0 && running;
}

void UARTBridge::stop() {
    running = false;
    
    if (readerThread && readerThread->joinable()) {
        readerThread->join();
    }
    
    std::lock_guard<std::mutex> lock(serialMutex);
    if (serialFd >= 0) {
        close(serialFd);
        serialFd = -1;
    }
}