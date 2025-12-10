#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

#include "Protocol.h"
#include <string>
#include <vector>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

class RobotController {
public:
    RobotController(uint8_t address = 0x08, const std::string& i2cBus = "/dev/i2c-1");
    ~RobotController();
    
    bool connect();
    void disconnect();
    bool isConnected() const;
    
    // Основные команды управления
    bool moveForward(uint8_t speed = 150);
    bool moveBackward(uint8_t speed = 150);
    bool turnLeft(uint8_t speed = 150);
    bool turnRight(uint8_t speed = 150);
    bool stop();
    bool setMotorSpeeds(uint8_t leftSpeed, uint8_t rightSpeed);
    bool testMotors();
    
    // Чтение статуса
    RobotProtocol::RobotStatus getStatus();
    
    // Утилиты
    std::string getLastError() const;
    uint8_t getDeviceAddress() const;
    
private:
    bool sendCommand(const std::vector<uint8_t>& command);
    RobotProtocol::RobotStatus readStatus();
    
    int i2cFile;
    uint8_t deviceAddress;
    std::string i2cBusPath;
    bool connected;
    std::string lastError;
    
    // Время ожидания для I2C операций (в микросекундах)
    static const int I2C_TIMEOUT = 10000;
};

#endif // ROBOTCONTROLLER_H
