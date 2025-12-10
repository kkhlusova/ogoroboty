#pragma once
#include <Arduino.h>
#include <stdint.h>

/**
 * @class MotorController
 * @brief Управление двигателями робота через драйвер L298N или аналогичный
 * 
 * Поддерживает:
 * - Движение вперед/назад с регулировкой скорости
 * - Повороты на заданный угол
 * - Плавный разгон/торможение
 * - Безопасную остановку
 */
class MotorController {
public:
    /**
     * @brief Конструктор
     * @param in1 Пин IN1 драйвера мотора
     * @param in2 Пин IN2 драйвера мотора
     * @param ena Пин ENA (ШИМ) драйвера мотора
     * @param in3 Пин IN3 драйвера мотора (для второго мотора)
     * @param in4 Пин IN4 драйвера мотора (для второго мотора)
     * @param enb Пин ENB (ШИМ) драйвера мотора (для второго мотора)
     */
    MotorController(uint8_t in1, uint8_t in2, uint8_t ena, 
                   uint8_t in3, uint8_t in4, uint8_t enb);
    
    /**
     * @brief Инициализация пинов и настроек
     */
    void init();
    
    /**
     * @brief Движение в заданном направлении
     * @param direction 0 = вперед, 1 = назад
     * @param speed Скорость (0-255)
     */
    void move(uint8_t direction, uint8_t speed);
    
    /**
     * @brief Поворот на заданный угол
     * @param angle Угол в градусах: >0 - вправо, <0 - влево
     * 
     * Алгоритм:
     * 1. Остановка
     * 2. Включение моторов в разные стороны
     * 3. Расчет времени поворота по углу
     * 4. Остановка после завершения
     */
    void turn(int16_t angle);
    
    /**
     * @brief Плавный поворот (один мотор вперед, другой назад)
     * @param speed Скорость поворота (0-255)
     * @param right true = поворот вправо, false = влево
     */
    void smoothTurn(uint8_t speed, bool right);
    
    /**
     * @brief Немедленная остановка
     */
    void stop();
    
    /**
     * @brief Плавная остановка
     * @param deceleration Скорость замедления (шаг уменьшения скорости)
     */
    void stopSmoothly(uint8_t deceleration = 10);
    
    /**
     * @brief Установка скорости
     * @param speed Скорость (0-255)
     */
    void setSpeed(uint8_t speed);
    
    /**
     * @brief Получение текущей скорости
     * @return Текущая скорость (0-255)
     */
    uint8_t getCurrentSpeed() const;
    
    /**
     * @brief Получение текущего направления
     * @return 0 = вперед, 1 = назад, 2 = остановлен
     */
    uint8_t getCurrentDirection() const;
    
    /**
     * @brief Калибровка моторов (устранение разницы в скорости)
     * @param leftCorrection Поправка для левого мотора (0.8-1.2)
     * @param rightCorrection Поправка для правого мотора (0.8-1.2)
     */
    void calibrate(float leftCorrection, float rightCorrection);
    
    /**
     * @brief Тест моторов (вращение в разные стороны)
     */
    void test();
    
private:
    // Пины для левого мотора
    uint8_t IN1, IN2, ENA;
    
    // Пины для правого мотора  
    uint8_t IN3, IN4, ENB;
    
    // Текущие параметры
    uint8_t currentSpeed;
    uint8_t currentDirection; // 0=вперед, 1=назад, 2=остановлен
    
    // Коэффициенты калибровки
    float leftMotorCorrection;
    float rightMotorCorrection;
    
    /**
     * @brief Установка направления вращения мотора
     * @param in1 Пин IN1
     * @param in2 Пин IN2
     * @param forward true = вперед, false = назад
     */
    void setMotorDirection(uint8_t in1, uint8_t in2, bool forward);
    
    /**
     * @brief Установка скорости мотора с коррекцией
     * @param speedPin Пин ШИМ
     * @param speed Скорость (0-255)
     * @param correction Коэффициент коррекции
     */
    void setMotorSpeed(uint8_t speedPin, uint8_t speed, float correction);
    
    /**
     * @brief Расчет времени поворота
     * @param angle Угол в градусах
     * @return Время в миллисекундах
     */
    uint32_t calculateTurnTime(int16_t angle) const;
};