/*
 * Main server application for robot video streaming and command handling
 * Combines video streaming and command server in one application
 */

#include "video_stream_server.cpp"
#include "robot_command_server.h"
#include <QApplication>
#include <QTimer>
#include <QDebug>
#include <signal.h>
#include <iostream>
#include <thread>
#include <atomic>

// Глобальные переменные для обработки сигналов
std::atomic<bool> g_running{false};
VideoStreamServer* g_videoServer = nullptr;
RobotCommandServer* g_commandServer = nullptr;

// Обработчик сигналов для корректного завершения
void signalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ", stopping servers..." << std::endl;
    g_running = false;
    
    if (g_videoServer) {
        g_videoServer->stop();
    }
    
    if (g_commandServer) {
        g_commandServer->stopServer();
    }
    
    exit(0);
}

int main(int argc, char *argv[]) {
    // Установка обработчиков сигналов
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // Параметры по умолчанию
    std::string videoHost = "192.168.1.100"; // IP-адрес ноутбука (замените на нужный)
    int videoPort = 5000; // Порт для передачи видео
    int commandPort = 8888; // Порт для команд управления
    int cameraId = 0; // ID камеры (обычно 0 для первой камеры)
    uint8_t arduinoAddress = 0x08; // Адрес Arduino по умолчанию

    // Проверка аргументов командной строки
    if (argc >= 2) {
        videoHost = argv[1];
    }
    if (argc >= 3) {
        videoPort = std::stoi(argv[2]);
    }
    if (argc >= 4) {
        commandPort = std::stoi(argv[3]);
    }
    if (argc >= 5) {
        cameraId = std::stoi(argv[4]);
    }
    if (argc >= 6) {
        arduinoAddress = static_cast<uint8_t>(std::stoi(argv[5]));
    }

    std::cout << "=== Robot Control and Video Streaming Server ===" << std::endl;
    std::cout << "Video streaming to: " << videoHost << ":" << videoPort << std::endl;
    std::cout << "Command server on ports: TCP " << commandPort << ", UDP " << (commandPort+1) << std::endl;
    std::cout << "Camera ID: " << cameraId << ", Arduino address: 0x" << std::hex << (int)arduinoAddress << std::dec << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl;

    // Инициализация Qt приложения
    QCoreApplication app(argc, argv);

    // Создание видео сервера
    VideoStreamServer videoServer(videoHost, videoPort, cameraId);
    g_videoServer = &videoServer;

    // Создание сервера команд
    RobotCommandServer commandServer;
    g_commandServer = &commandServer;

    // Подключение к Arduino
    if (!commandServer.connectToArduino(arduinoAddress)) {
        std::cerr << "Failed to connect to Arduino, exiting..." << std::endl;
        return -1;
    }

    // Запуск сервера команд
    if (!commandServer.startServer(commandPort, commandPort + 1)) {
        std::cerr << "Failed to start command server, exiting..." << std::endl;
        return -1;
    }

    // Запуск видеостриминга
    if (!videoServer.start()) {
        std::cerr << "Failed to start video streaming server" << std::endl;
        return -1;
    }

    g_running = true;

    // Таймер для периодического опроса
    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout, [&app]() {
        if (!g_running) {
            app.quit();
        }
    });
    timer.start(100); // каждые 100 мс

    std::cout << "Servers are running..." << std::endl;

    // Запуск Qt event loop
    int result = app.exec();

    // Остановка серверов
    videoServer.stop();
    commandServer.stopServer();

    std::cout << "Servers stopped normally" << std::endl;
    return result;
}