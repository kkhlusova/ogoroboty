#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <cstdint>
#include <vector>

namespace RobotProtocol {
    
    // Команды протокола
    enum Command : uint8_t {
        CMD_FORWARD      = 0x01,
        CMD_BACKWARD     = 0x02,
        CMD_TURN_LEFT    = 0x03,
        CMD_TURN_RIGHT   = 0x04,
        CMD_STOP         = 0x05,
        CMD_SET_SPEEDS   = 0x06,
        CMD_TEST         = 0x07,
        CMD_GET_STATUS   = 0x08
    };
    
    // Структура статуса робота
    struct RobotStatus {
        uint8_t batteryLevel;
        uint8_t temperature;
        uint8_t leftSpeed;
        uint8_t rightSpeed;
        uint8_t errorCode;
        
        void print() const;
    };
    
    // Класс для создания команд
    class CommandBuilder {
    public:
        static std::vector<uint8_t> forward(uint8_t speed);
        static std::vector<uint8_t> backward(uint8_t speed);
        static std::vector<uint8_t> turnLeft(uint8_t speed);
        static std::vector<uint8_t> turnRight(uint8_t speed);
        static std::vector<uint8_t> stop();
        static std::vector<uint8_t> setSpeeds(uint8_t left, uint8_t right);
        static std::vector<uint8_t> test();
        static std::vector<uint8_t> getStatus();
        
    private:
        static std::vector<uint8_t> createCommand(Command cmd, uint8_t param1 = 0, uint8_t param2 = 0);
    };
    
} // namespace RobotProtocol

#endif // PROTOCOL_H
