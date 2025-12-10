#include "Protocol.h"
#include <iostream>
#include <iomanip>

namespace RobotProtocol {

void RobotStatus::print() const {
    std::cout << "=== Robot Status ===" << std::endl;
    std::cout << "Battery Level: " << static_cast<int>(batteryLevel) << "%" << std::endl;
    std::cout << "Temperature: " << static_cast<int>(temperature) << "°C" << std::endl;
    std::cout << "Left Speed: " << static_cast<int>(leftSpeed) << std::endl;
    std::cout << "Right Speed: " << static_cast<int>(rightSpeed) << std::endl;
    std::cout << "Error Code: 0x" << std::hex << static_cast<int>(errorCode) << std::dec << std::endl;
}

std::vector<uint8_t> CommandBuilder::createCommand(Command cmd, uint8_t param1, uint8_t param2) {
    std::vector<uint8_t> command;
    command.push_back(static_cast<uint8_t>(cmd));
    
    if (cmd != CMD_STOP && cmd != CMD_TEST && cmd != CMD_GET_STATUS) {
        command.push_back(param1);
    }
    
    if (cmd == CMD_SET_SPEEDS) {
        command.push_back(param2);
    }
    
    return command;
}

std::vector<uint8_t> CommandBuilder::forward(uint8_t speed) {
    return createCommand(CMD_FORWARD, speed);
}

std::vector<uint8_t> CommandBuilder::backward(uint8_t speed) {
    return createCommand(CMD_BACKWARD, speed);
}

std::vector<uint8_t> CommandBuilder::turnLeft(uint8_t speed) {
    return createCommand(CMD_TURN_LEFT, speed);
}

std::vector<uint8_t> CommandBuilder::turnRight(uint8_t speed) {
    return createCommand(CMD_TURN_RIGHT, speed);
}

std::vector<uint8_t> CommandBuilder::stop() {
    return createCommand(CMD_STOP);
}

std::vector<uint8_t> CommandBuilder::setSpeeds(uint8_t left, uint8_t right) {
    return createCommand(CMD_SET_SPEEDS, left, right);
}

std::vector<uint8_t> CommandBuilder::test() {
    return createCommand(CMD_TEST);
}

std::vector<uint8_t> CommandBuilder::getStatus() {
    return createCommand(CMD_GET_STATUS);
}

} // namespace RobotProtocol
