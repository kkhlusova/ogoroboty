#include "sensor_manager.h"

// Константы
#define SOUND_SPEED 0.0343  // Скорость звука в см/мкс
#define MAX_DISTANCE 400    // Максимальное расстояние для УЗ датчика (см)
#define TIMEOUT_US (MAX_DISTANCE * 2 * 29.1) // Таймаут в микросекундах

// Для DHT датчика (упрощенная реализация)
#define DHT_TYPE 11  // DHT11 (для DHT22 будет 22)

SensorManager::SensorManager(uint8_t trigPin, uint8_t echoPin,
                           uint8_t irLeft, uint8_t irRight,
                           uint8_t dhtPin, uint8_t batteryPin)
    : TRIG_PIN(trigPin), ECHO_PIN(echoPin),
      IR_LEFT_PIN(irLeft), IR_RIGHT_PIN(irRight),
      DHT_PIN(dhtPin), BATTERY_PIN(batteryPin),
      distanceFront(0), distanceLeft(0), distanceRight(0),
      obstacleLeftIR(false), obstacleRightIR(false),
      temperature(0), humidity(0), batteryVoltage(0),
      lastOdometerUpdate(0), traveledDistance(0),
      totalTurnAngle(0), lastSpeed(0),
      lastError(0), sensorsInitialized(false),
      distanceBufferIndex(0), voltageBufferIndex(0) {
    
    // Инициализация буферов фильтрации
    for (uint8_t i = 0; i < FILTER_SIZE; i++) {
        distanceBuffer[i] = 0;
        voltageBuffer[i] = 0;
    }
}

bool SensorManager::init() {
    // Настройка пинов
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(IR_LEFT_PIN, INPUT);
    pinMode(IR_RIGHT_PIN, INPUT);
    pinMode(BATTERY_PIN, INPUT);
    
    digitalWrite(TRIG_PIN, LOW);
    
    // Инициализация DHT (упрощенно)
    pinMode(DHT_PIN, INPUT_PULLUP);
    
    // Первое чтение для инициализации
    updateAllSensors();
    
    sensorsInitialized = true;
    Serial.println("[SensorManager] Initialized");
    
    return true;
}

void SensorManager::updateAllSensors() {
    // Ультразвуковые датчики (в реальности может быть несколько)
    distanceFront = readUltrasonic(TRIG_PIN, ECHO_PIN);
    
    // Для простоты используем один датчик, но эмулируем три
    // В реальном проекте будут разные пины
    distanceLeft = distanceFront + random(-10, 10);  // Эмуляция
    distanceRight = distanceFront + random(-10, 10); // Эмуляция
    
    // ИК датчики
    obstacleLeftIR = readIRSensor(IR_LEFT_PIN);
    obstacleRightIR = readIRSensor(IR_RIGHT_PIN);
    
    // Температура и влажность
    readDHT();
    
    // Напряжение батареи
    batteryVoltage = readBatteryVoltage();
    
    // Фильтрация значений
    distanceFront = medianFilter(distanceFront, distanceBuffer, distanceBufferIndex);
    batteryVoltage = medianFilter(batteryVoltage, voltageBuffer, voltageBufferIndex);
}

uint16_t SensorManager::readUltrasonic(uint8_t trig, uint8_t echo) {
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    
    long duration = pulseIn(echo, HIGH, TIMEOUT_US);
    
    if (duration == 0) {
        lastError = 1; // Таймаут
        return MAX_DISTANCE;
    }
    
    // Расчет расстояния в см
    uint16_t distance = duration * SOUND_SPEED / 2;
    
    // Ограничение максимального расстояния
    if (distance > MAX_DISTANCE) {
        distance = MAX_DISTANCE;
    }
    
    return distance;
}

bool SensorManager::readIRSensor(uint8_t pin) {
    // ИК датчики обычно выдают LOW при обнаружении препятствия
    // Это зависит от конкретной модели
    return digitalRead(pin) == LOW;
}

bool SensorManager::readDHT() {
    // Упрощенная реализация для DHT11
    // В реальном проекте используйте библиотеку DHT
    
    // Эмуляция данных для тестирования
    static int16_t fakeTemp = 235; // 23.5°C
    static uint8_t fakeHumidity = 45;
    
    // Немного меняем значения для реалистичности
    fakeTemp += random(-1, 2);
    fakeHumidity += random(-1, 2);
    
    // Ограничиваем диапазоны
    fakeTemp = constrain(fakeTemp, 0, 500);  // 0-50°C
    fakeHumidity = constrain(fakeHumidity, 0, 100);
    
    temperature = fakeTemp;
    humidity = fakeHumidity;
    
    return true;
}

uint16_t SensorManager::readBatteryVoltage() {
    // Чтение аналогового значения (0-1023 для 0-5V на Arduino)
    int rawValue = analogRead(BATTERY_PIN);
    
    // Коэффициент делителя напряжения (если используется)
    // Например, если делитель 1/2, то multiply = 2
    float voltageMultiplier = 2.0; // Настройте под свою схему
    
    // Расчет напряжения в милливольтах
    // Arduino 5V reference: 5000mV / 1024 = ~4.88mV за единицу
    float voltage = rawValue * (5000.0 / 1024.0) * voltageMultiplier;
    
    return static_cast<uint16_t>(vol