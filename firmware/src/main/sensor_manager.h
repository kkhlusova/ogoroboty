#pragma once
#include <Arduino.h>
#include <stdint.h>

/**
 * @class SensorManager
 * @brief Управление всеми датчиками робота
 * 
 * Поддерживает:
 * - Ультразвуковые датчики расстояния (HC-SR04)
 * - Инфракрасные датчики препятствий
 * - Датчик температуры и влажности DHT11/DHT22
 * - Мониторинг напряжения батареи
 * - Одометрию (примерную)
 */
class SensorManager {
public:
    /**
     * @brief Конструктор
     * @param trigPin Пин TRIG ультразвукового датчика
     * @param echoPin Пин ECHO ультразвукового датчика
     * @param irLeft Пин левого ИК датчика
     * @param irRight Пин правого ИК датчика
     * @param dhtPin Пин датчика DHT
     * @param batteryPin Аналоговый пин для измерения напряжения
     */
    SensorManager(uint8_t trigPin, uint8_t echoPin,
                 uint8_t irLeft, uint8_t irRight,
                 uint8_t dhtPin, uint8_t batteryPin);
    
    /**
     * @brief Инициализация всех датчиков
     * @return true если все датчики инициализированы успешно
     */
    bool init();
    
    /**
     * @brief Обновление всех показаний датчиков
     * @note Вызывать периодически (например, каждые 100 мс)
     */
    void updateAllSensors();
    
    // === Геттеры для ультразвуковых датчиков ===
    
    /**
     * @brief Расстояние спереди
     * @return Расстояние в сантиметрах (2-400 см)
     */
    uint16_t getDistanceFront() const;
    
    /**
     * @brief Расстояние слева
     * @return Расстояние в сантиметрах
     */
    uint16_t getDistanceLeft() const;
    
    /**
     * @brief Расстояние справа
     * @return Расстояние в сантиметрах
     */
    uint16_t getDistanceRight() const;
    
    /**
     * @brief Проверка наличия препятствия спереди
     * @param threshold Пороговое расстояние в см
     * @return true если препятствие ближе порога
     */
    bool isObstacleFront(uint16_t threshold = 30) const;
    
    // === Геттеры для ИК датчиков ===
    
    /**
     * @brief Проверка препятствия слева (ИК)
     * @return true если препятствие обнаружено
     */
    bool isObstacleLeftIR() const;
    
    /**
     * @brief Проверка препятствия справа (ИК)
     * @return true если препятствие обнаружено
     */
    bool isObstacleRightIR() const;
    
    // === Геттеры для температуры/влажности ===
    
    /**
     * @brief Температура
     * @return Температура в градусах Цельсия * 10 (235 = 23.5°C)
     */
    int16_t getTemperature() const;
    
    /**
     * @brief Влажность
     * @return Влажность в процентах (0-100%)
     */
    uint8_t getHumidity() const;
    
    // === Геттеры для батареи ===
    
    /**
     * @brief Напряжение батареи
     * @return Напряжение в милливольтах (например, 7800 = 7.8V)
     */
    uint16_t getBatteryVoltage() const;
    
    /**
     * @brief Уровень заряда батареи
     * @return Процент заряда (0-100%)
     */
    uint8_t getBatteryPercentage() const;
    
    /**
     * @brief Проверка низкого заряда батареи
     * @return true если заряд ниже 20%
     */
    bool isBatteryLow() const;
    
    // === Одометрия (примерное отслеживание пути) ===
    
    /**
     * @brief Сброс одометрии
     */
    void resetOdometer();
    
    /**
     * @brief Обновление одометрии (вызывать в loop при движении)
     * @param speed Текущая скорость (0-255)
     * @param isMoving Движется ли робот
     */
    void updateOdometer(uint8_t speed, bool isMoving);
    
    /**
     * @brief Пройденное расстояние
     * @return Расстояние в сантиметрах
     */
    uint32_t getTraveledDistance() const;
    
    /**
     * @brief Угол поворота
     * @return Суммарный угол поворота в градусах
     */
    int32_t getTotalTurnAngle() const;
    
    // === Диагностика ===
    
    /**
     * @brief Тест всех датчиков
     */
    void testAllSensors();
    
    /**
     * @brief Статус датчиков
     * @return true если все датчики работают
     */
    bool allSensorsOk() const;
    
    /**
     * @brief Получение последней ошибки
     * @return Код ошибки (0 = нет ошибок)
     */
    uint8_t getLastError() const;
    
private:
    // Пины датчиков
    uint8_t TRIG_PIN;
    uint8_t ECHO_PIN;
    uint8_t IR_LEFT_PIN;
    uint8_t IR_RIGHT_PIN;
    uint8_t DHT_PIN;
    uint8_t BATTERY_PIN;
    
    // Текущие показания
    uint16_t distanceFront;
    uint16_t distanceLeft;
    uint16_t distanceRight;
    bool obstacleLeftIR;
    bool obstacleRightIR;
    int16_t temperature;
    uint8_t humidity;
    uint16_t batteryVoltage;
    
    // Одометрия
    uint32_t lastOdometerUpdate;
    uint32_t traveledDistance;
    int32_t totalTurnAngle;
    uint8_t lastSpeed;
    
    // Состояние
    uint8_t lastError;
    bool sensorsInitialized;
    
    // Приватные методы
    uint16_t readUltrasonic(uint8_t trig, uint8_t echo);
    bool readIRSensor(uint8_t pin);
    bool readDHT();
    uint16_t readBatteryVoltage();
    
    // Фильтрация значений (медианный фильтр)
    uint16_t medianFilter(uint16_t newValue, uint16_t* buffer, uint8_t& index);
    
    // Буферы для фильтрации
    static const uint8_t FILTER_SIZE = 5;
    uint16_t distanceBuffer[FILTER_SIZE];
    uint8_t distanceBufferIndex;
    uint16_t voltageBuffer[FILTER_SIZE];
    uint8_t voltageBufferIndex;
};