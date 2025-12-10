#pragma once
#include <Arduino.h>

// Константы протокола
#define UART_BAUD_RATE 115200
#define PACKET_HEADER_1 0xAA
#define PACKET_HEADER_2 0x55
#define PACKET_FOOTER_1 0x55
#define PACKET_FOOTER_2 0xAA

// Типы команд
enum CommandType : uint8_t {
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

// Структура для хранения полученной команды
struct ReceivedCommand {
    CommandType type;
    uint8_t length;
    uint8_t data[32];
};

// Структура для данных датчиков
struct SensorData {
    uint16_t distance_front;  // Расстояние спереди (см)
    uint16_t distance_left;   // Расстояние слева
    uint16_t distance_right;  // Расстояние справа
    int16_t temperature;      // Температура *10 (23.5°C = 235)
    uint8_t humidity;         // Влажность %
    uint16_t battery_voltage; // Напряжение аккумулятора (мВ)
    uint32_t timestamp;       // Время от Arduino (мс)
};

class UARTProtocol {
public:
    UARTProtocol();
    
    // Инициализация UART
    void begin();
    
    // Проверка наличия данных
    bool available();
    
    // Чтение команды из UART
    bool readCommand(ReceivedCommand& cmd);
    
    // Отправка ACK
    void sendAck(CommandType originalCmd, bool success = true);
    
    // Отправка данных датчиков
    void sendSensorData(const SensorData& data);
    
    // Отправка ошибки
    void sendError(uint8_t errorCode, const char* message = nullptr);
    
private:
    // Вычисление CRC8
    uint8_t calculateCRC(const uint8_t* data, uint8_t length);
    
    // Отправка пакета
    void sendPacket(CommandType type, const uint8_t* data = nullptr, uint8_t length = 0);
    
    // Буфер для приема
    uint8_t rxBuffer[64];
    uint8_t rxIndex;
    bool receivingPacket;
};