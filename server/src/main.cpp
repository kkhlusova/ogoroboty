#include "RobotController.h"
#include <iostream>
#include <cstdlib>
#include <signal.h>
#include <thread>
#include <chrono>

// Глобальный указатель на контроллер для обработки сигналов
RobotController* gRobotController = nullptr;

// Обработчик сигналов для корректного завершения
void signalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ", stopping motors and exiting..." << std::endl;
    
    if (gRobotController && gRobotController->isConnected()) {
        gRobotController->stop();
        gRobotController->disconnect();
    }
    
    exit(0);
}

// Демонстрационная программа
void demoSequence(RobotController& robot) {
    std::cout << "\n=== Starting Demo Sequence ===" << std::endl;
    
    // 1. Тест моторов
    std::cout << "1. Testing motors..." << std::endl;
    robot.testMotors();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // 2. Движение вперед
    std::cout << "2. Moving forward..." << std::endl;
    robot.moveForward(200);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // 3. Поворот влево
    std::cout << "3. Turning left..." << std::endl;
    robot.turnLeft(150);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // 4. Поворот вправо
    std::cout << "4. Turning right..." << std::endl;
    robot.turnRight(150);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // 5. Движение назад
    std::cout << "5. Moving backward..." << std::endl;
    robot.moveBackward(180);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // 6. Остановка
    std::cout << "6. Stopping..." << std::endl;
    robot.stop();
    
    std::cout << "Demo sequence completed!" << std::endl;
}

// Интерактивное управление
void interactiveControl(RobotController& robot) {
    char command;
    uint8_t speed = 150;
    
    std::cout << "\n=== Interactive Control ===" << std::endl;
    std::cout << "Commands:" << std::endl;
    std::cout << "  w - Move forward" << std::endl;
    std::cout << "  s - Move backward" << std::endl;
    std::cout << "  a - Turn left" << std::endl;
    std::cout << "  d - Turn right" << std::endl;
    std::cout << "  x - Stop" << std::endl;
    std::cout << "  + - Increase speed" << std::endl;
    std::cout << "  - - Decrease speed" << std::endl;
    std::cout << "  i - Set individual motor speeds" << std::endl;
    std::cout << "  g - Get status" << std::endl;
    std::cout << "  q - Quit" << std::endl;
    
    while (true) {
        std::cout << "\nCommand [w/s/a/d/x/+/-/i/g/q]: ";
        std::cin >> command;
        
        switch (command) {
            case 'w':
                robot.moveForward(speed);
                std::cout << "Moving forward at speed: " << static_cast<int>(speed) << std::endl;
                break;
                
            case 's':
                robot.moveBackward(speed);
                std::cout << "Moving backward at speed: " << static_cast<int>(speed) << std::endl;
                break;
                
            case 'a':
                robot.turnLeft(speed);
                std::cout << "Turning left at speed: " << static_cast<int>(speed) << std::endl;
                break;
                
            case 'd':
                robot.turnRight(speed);
                std::cout << "Turning right at speed: " << static_cast<int>(speed) << std::endl;
                break;
                
            case 'x':
                robot.stop();
                std::cout << "Stopped" << std::endl;
                break;
                
            case '+':
                if (speed < 250) speed += 10;
                std::cout << "Speed increased to: " << static_cast<int>(speed) << std::endl;
                break;
                
            case '-':
                if (speed > 10) speed -= 10;
                std::cout << "Speed decreased to: " << static_cast<int>(speed) << std::endl;
                break;
                
            case 'i': {
                uint8_t left, right;
                std::cout << "Enter left motor speed (0-255): ";
                std::cin >> left;
                std::cout << "Enter right motor speed (0-255): ";
                std::cin >> right;
                robot.setMotorSpeeds(left, right);
                std::cout << "Set speeds - L: " << static_cast<int>(left) 
                          << ", R: " << static_cast<int>(right) << std::endl;
                break;
            }
                
            case 'g': {
                auto status = robot.getStatus();
                status.print();
                break;
            }
                
            case 'q':
                std::cout << "Exiting interactive mode..." << std::endl;
                return;
                
            default:
                std::cout << "Unknown command. Try again." << std::endl;
                break;
        }
    }
}

int main(int argc, char* argv[]) {
    // Установка обработчиков сигналов
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    std::cout << "=== Raspberry Pi Robot Controller ===" << std::endl;
    std::cout << "Connecting to Arduino..." << std::endl;
    
    // Создание контроллера
    RobotController robot(0x08);  // Адрес по умолчанию 0x08
    
    // Сохраняем указатель для обработки сигналов
    gRobotController = &robot;
    
    // Подключение к Arduino
    if (!robot.connect()) {
        std::cerr << "Error: " << robot.getLastError() << std::endl;
        return 1;
    }
    
    // Выбор режима работы
    int choice;
    std::cout << "\nSelect mode:" << std::endl;
    std::cout << "1. Demo sequence" << std::endl;
    std::cout << "2. Interactive control" << std::endl;
    std::cout << "3. Exit" << std::endl;
    std::cout << "Choice: ";
    std::cin >> choice;
    
    switch (choice) {
        case 1:
            demoSequence(robot);
            break;
        case 2:
            interactiveControl(robot);
            break;
        case 3:
            std::cout << "Exiting..." << std::endl;
            break;
        default:
            std::cout << "Invalid choice. Exiting..." << std::endl;
            break;
    }
    
    // Остановка моторов и отключение
    robot.stop();
    robot.disconnect();
    
    std::cout << "Program terminated successfully." << std::endl;
    return 0;
}
