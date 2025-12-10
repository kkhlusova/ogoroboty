#include "RobotController.h"
#include <iostream>
#include <cstring>
#include <thread>
#include <chrono>

RobotController::RobotController(uint8_t address, const std::string& i2cBus) 
    : i2cFile(-1), deviceAddress(address), i2cBusPath(i2cBus), connected(false) {
}

RobotController::~RobotController() {
    disconnect();
}

bool RobotController::connect() {
    // Открываем I2C шину
    i2cFile = open(i2cBusPath.c_str(), O_RDWR);
    if (i2cFile < 0) {
        lastError = "Failed to open I2C bus: " + i2cBusPath;
        return false;
    }
    
    // Устанавливаем адрес устройства
    if (ioctl(i2cFile, I2C_SLAVE, deviceAddress) < 0) {
        lastError = "Failed to set I2C slave address";
        close(i2cFile);
        i2cFile = -1;
        return false;
    }
    
    // Тестовое соединение
    std::vector<uint8_t> testCmd = RobotProtocol::CommandBuilder::test();
    if (!sendCommand(testCmd)) {
        lastError = "Failed to communicate with Arduino at address 0x" + 
                   std::to_string(deviceAddress);
        close(i2cFile);
        i2cFile = -1;
        return false;
    }
    
    connected = true;
    std::cout << "Successfully connected to Arduino at address 0x" 
              << std::hex << static_cast<int>(deviceAddress) << std::dec << std::endl;
    
    // Небольшая задержка после подключения
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return true;
}

void RobotController::disconnect() {
    if (isConnected()) {
        // Останавливаем моторы перед отключением
        stop();
        
        close(i2cFile);
        i2cFile = -1;
        connected = false;
        std::cout << "Disconnected from Arduino" << std::endl;
    }
}

bool RobotController::isConnected() const {
    return connected;
}

bool RobotController::sendCommand(const std::vector<uint8_t>& command) {
    if (!isConnected()) {
        lastError = "Not connected to Arduino";
        return false;
    }
    
    if (command.empty()) {
        lastError = "Empty command";
        return false;
    }
    
    // Отправляем команду
    ssize_t result = write(i2cFile, command.data(), command.size());
    
    if (result != static_cast<ssize_t>(command.size())) {
        lastError = "Failed to send command to Arduino";
        return false;
    }
    
    // Небольшая задержка для обработки команды на Arduino
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return true;
}

RobotProtocol::RobotStatus RobotController::readStatus() {
    RobotProtocol::RobotStatus status = {0, 0, 0, 0, 0};
    
    if (!isConnected()) {
        lastError = "Not connected to Arduino";
        status.errorCode = 0xFF;
        return status;
    }
    
    // Запрашиваем статус
    std::vector<uint8_t> cmd = RobotProtocol::CommandBuilder::getStatus();
    if (!sendCommand(cmd)) {
        status.errorCode = 0xFE;
        return status;
    }
    
    // Читаем 5 байт статуса
    uint8_t buffer[5] = {0};
    ssize_t bytesRead = read(i2cFile, buffer, 5);
    
    if (bytesRead == 5) {
        status.batteryLevel = buffer[0];
        status.temperature = buffer[1];
        status.leftSpeed = buffer[2];
        status.rightSpeed = buffer[3];
        status.errorCode = buffer[4];
    } else {
        lastError = "Failed to read status from Arduino";
        status.errorCode = 0xFD;
    }
    
    return status;
}

RobotProtocol::RobotStatus RobotController::getStatus() {
    return readStatus();
}

bool RobotController::moveForward(uint8_t speed) {
    std::vector<uint8_t> cmd = RobotProtocol::CommandBuilder::forward(speed);
    return sendCommand(cmd);
}

bool RobotController::moveBackward(uint8_t speed) {
    std::vector<uint8_t> cmd = RobotProtocol::CommandBuilder::backward(speed);
    return sendCommand(cmd);
}

bool RobotController::turnLeft(uint8_t speed) {
    std::vector<uint8_t> cmd = RobotProtocol::CommandBuilder::turnLeft(speed);
    return sendCommand(cmd);
}

bool RobotController::turnRight(uint8_t speed) {
    std::vector<uint8_t> cmd = RobotProtocol::CommandBuilder::turnRight(speed);
    return sendCommand(cmd);
}

bool RobotController::stop() {
    std::vector<uint8_t> cmd = RobotProtocol::CommandBuilder::stop();
    return sendCommand(cmd);
}

bool RobotController::setMotorSpeeds(uint8_t leftSpeed, uint8_t rightSpeed) {
    std::vector<uint8_t> cmd = RobotProtocol::CommandBuilder::setSpeeds(leftSpeed, rightSpeed);
    return sendCommand(cmd);
}

bool RobotController::testMotors() {
    std::vector<uint8_t> cmd = RobotProtocol::CommandBuilder::test();
    return sendCommand(cmd);
}

std::string RobotController::getLastError() const {
    return lastError;
}

uint8_t RobotController::getDeviceAddress() const {
    return deviceAddress;
}
