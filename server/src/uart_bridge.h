#pragma once

#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>

// Типы команд (должны совпадать с Arduino)
enum class CommandType : uint8_t {
    CMD_PING = 0x01,
    CMD_MOVE = 0x02,
    CMD_TURN = 0x03,
    CMD_STOP = 0x04,
    CMD_GET_SENSORS = 0x05,
    CMD_SET_SPEED = 0x06,
    CMD_BUZZER = 0x07,
    CMD_SCRIPT = 0x08,
    CMD_MOVE_UNTIL_OBSTACLE = 0x09,
    
    RESP_ACK = 0x80,
    RESP_SENSOR_DATA = 0x81,
    RESP_ERROR = 0x82
};

// Структура данных датчиков
#pragma pack(push, 1)
struct SensorData {
    uint16_t distance_front;
    uint16_t distance_left;
    uint16_t distance_right;
    int16_t temperature;
    uint8_t humidity;
    uint16_t battery_voltage;
    uint32_t timestamp;
};
#pragma pack(pop)

// Callback-функции
using SensorCallback = std::function<void(const SensorData&)>;
using AckCallback = std::function<void(CommandType, bool)>;
using ErrorCallback = std::function<void(uint8_t, const std::string&)>;

class UARTBridge {
public:
    UARTBridge();
    ~UARTBridge();
    
    // Инициализация
    bool initialize(const std::string& port = "/dev/ttyAMA0", int baudrate = 115200);
    
    // Основные команды
    bool sendPing();
    bool sendMove(uint8_t direction, uint8_t speed);
    bool sendTurn(int16_t angle);
    bool sendStop();
    bool requestSensorData();
    bool sendSetSpeed(uint8_t speed);
    bool sendBuzzer(uint16_t frequency);
    bool sendScript(const std::vector<uint8_t>& script);
    bool sendMoveUntilObstacle(uint8_t speed);
    
    // Регистрация callback-ов
    void registerSensorCallback(SensorCallback callback);
    void registerAckCallback(AckCallback callback);
    void registerErrorCallback(ErrorCallback callback);
    
    // Проверка соединения
    bool isConnected() const;
    
    // Остановка
    void stop();
    
private:
    // Внутренняя работа
    void run();
    bool sendCommand(CommandType type, const uint8_t* data = nullptr, size_t length = 0);
    bool readPacket(std::vector<uint8_t>& packet);
    uint8_t calculateCRC(const uint8_t* data, size_t length);
    
    // Обработка входящих пакетов
    void processPacket(const std::vector<uint8_t>& packet);
    
    // Поток для чтения
    std::unique_ptr<std::thread> readerThread;
    std::atomic<bool> running{false};
    
    // Callback-и
    SensorCallback sensorCallback;
    AckCallback ackCallback;
    ErrorCallback errorCallback;
    
    // Мьютексы
    mutable std::mutex serialMutex;
    int serialFd{-1};
    
    // Таймауты
    static constexpr int READ_TIMEOUT_MS = 100;
    static constexpr int WRITE_TIMEOUT_MS = 100;
};