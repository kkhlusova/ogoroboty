#include "uart_bridge.h"
#include <iostream>
#include <chrono>
#include <thread>

int main() {
    UARTBridge bridge;
    
    // Callback для данных датчиков
    bridge.registerSensorCallback([](const SensorData& data) {
        std::cout << "\n=== Sensor Data ===" << std::endl;
        std::cout << "Front distance: " << data.distance_front << " cm" << std::endl;
        std::cout << "Left distance: " << data.distance_left << " cm" << std::endl;
        std::cout << "Right distance: " << data.distance_right << " cm" << std::endl;
        std::cout << "Temperature: " << (data.temperature / 10.0) << " C" << std::endl;
        std::cout << "Humidity: " << static_cast<int>(data.humidity) << " %" << std::endl;
        std::cout << "Battery: " << (data.battery_voltage / 1000.0) << " V" << std::endl;
        std::cout << "Timestamp: " << data.timestamp << " ms" << std::endl;
    });
    
    // Callback для подтверждений
    bridge.registerAckCallback([](CommandType cmd, bool success) {
        std::cout << "ACK: Command " << static_cast<int>(cmd) 
                  << " - " << (success ? "SUCCESS" : "FAILED") << std::endl;
    });
    
    // Callback для ошибок
    bridge.registerErrorCallback([](uint8_t code, const std::string& msg) {
        std::cout << "ERROR [" << static_cast<int>(code) << "]: " << msg << std::endl;
    });
    
    // Инициализация
    if (!bridge.initialize("/dev/ttyAMA0", 115200)) {
        std::cerr << "Failed to initialize UART bridge" << std::endl;
        return 1;
    }
    
    std::cout << "UART Bridge Test Started" << std::endl;
    std::cout << "Commands: p=ping, f=forward, s=stop, t=turn, g=get sensors, q=quit" << std::endl;
    
    // Основной цикл
    char input;
    while (true) {
        std::cout << "\nCommand> ";
        std::cin >> input;
        
        switch (input) {
            case 'p':
                bridge.sendPing();
                break;
                
            case 'f':
                bridge.sendMove(0, 150); // Вперед, скорость 150/255
                break;
                
            case 'b':
                bridge.sendMove(1, 100); // Назад, скорость 100/255
                break;
                
            case 's':
                bridge.sendStop();
                break;
                
            case 't':
                bridge.sendTurn(90); // Поворот на 90 градусов
                break;
                
            case 'g':
                bridge.requestSensorData();
                break;
                
            case 'q':
                bridge.stop();
                std::cout << "Exiting..." << std::endl;
                return 0;
                
            default:
                std::cout << "Unknown command" << std::endl;
                break;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    return 0;
}
