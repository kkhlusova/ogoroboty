#include "motor_controller.h"

// Константы
#define DEFAULT_TURN_SPEED 150    // Скорость по умолчанию для поворотов
#define MIN_SPEED 50             // Минимальная скорость для движения
#define MAX_SPEED 255            // Максимальная скорость
#define TURN_TIME_PER_DEGREE 10  // Время на поворот 1 градуса (мс)
#define ACCELERATION_STEP 5      // Шаг ускорения

MotorController::MotorController(uint8_t in1, uint8_t in2, uint8_t ena,
                               uint8_t in3, uint8_t in4, uint8_t enb)
    : IN1(in1), IN2(in2), ENA(ena),
      IN3(in3), IN4(in4), ENB(enb),
      currentSpeed(0),
      currentDirection(2), // Остановлен
      leftMotorCorrection(1.0f),
      rightMotorCorrection(1.0f) {
}

void MotorController::init() {
    // Настройка пинов как выходов
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB, OUTPUT);
    
    // Изначально выключены
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    
    Serial.println("[MotorController] Initialized");
}

void MotorController::move(uint8_t direction, uint8_t speed) {
    if (speed > MAX_SPEED) speed = MAX_SPEED;
    if (speed < MIN_SPEED && speed != 0) speed = MIN_SPEED;
    
    currentDirection = direction;
    currentSpeed = speed;
    
    if (speed == 0) {
        stop();
        return;
    }
    
    // Установка направления
    bool forward = (direction == 0);
    setMotorDirection(IN1, IN2, forward);  // Левый мотор
    setMotorDirection(IN3, IN4, forward);  // Правый мотор
    
    // Установка скорости с коррекцией
    setMotorSpeed(ENA, speed, leftMotorCorrection);
    setMotorSpeed(ENB, speed, rightMotorCorrection);
    
    Serial.print("[MotorController] Moving ");
    Serial.print(forward ? "FORWARD" : "BACKWARD");
    Serial.print(" at speed ");
    Serial.println(speed);
}

void MotorController::turn(int16_t angle) {
    if (angle == 0) return;
    
    Serial.print("[MotorController] Turning ");
    Serial.print(angle);
    Serial.println(" degrees");
    
    // Останавливаем перед поворотом
    stop();
    delay(100);
    
    bool rightTurn = (angle > 0);
    uint8_t turnSpeed = DEFAULT_TURN_SPEED;
    
    // Настраиваем моторы для поворота
    if (rightTurn) {
        // Правый поворот: левый мотор вперед, правый назад
        setMotorDirection(IN1, IN2, true);   // Левый вперед
        setMotorDirection(IN3, IN4, false);  // Правый назад
    } else {
        // Левый поворот: правый мотор вперед, левый назад
        setMotorDirection(IN1, IN2, false);  // Левый назад
        setMotorDirection(IN3, IN4, true);   // Правый вперед
        angle = -angle; // Делаем положительным для расчетов
    }
    
    // Устанавливаем скорость
    setMotorSpeed(ENA, turnSpeed, leftMotorCorrection);
    setMotorSpeed(ENB, turnSpeed, rightMotorCorrection);
    
    // Ждем необходимое время для поворота
    uint32_t turnTime = calculateTurnTime(angle);
    delay(turnTime);
    
    // Останавливаемся
    stop();
    
    Serial.print("[MotorController] Turn completed in ");
    Serial.print(turnTime);
    Serial.println(" ms");
}

void MotorController::smoothTurn(uint8_t speed, bool right) {
    currentDirection = 2; // В режиме поворота
    
    if (right) {
        // Правый поворот: левый вперед, правый остановлен/медленнее
        setMotorDirection(IN1, IN2, true);
        setMotorDirection(IN3, IN4, true);
        
        uint8_t rightSpeed = speed / 2; // Правый мотор медленнее
        setMotorSpeed(ENA, speed, leftMotorCorrection);
        setMotorSpeed(ENB, rightSpeed, rightMotorCorrection);
    } else {
        // Левый поворот: правый вперед, левый остановлен/медленнее
        setMotorDirection(IN1, IN2, true);
        setMotorDirection(IN3, IN4, true);
        
        uint8_t leftSpeed = speed / 2; // Левый мотор медленнее
        setMotorSpeed(ENA, leftSpeed, leftMotorCorrection);
        setMotorSpeed(ENB, speed, rightMotorCorrection);
    }
    
    Serial.print("[MotorController] Smooth turn ");
    Serial.print(right ? "RIGHT" : "LEFT");
    Serial.print(" at speed ");
    Serial.println(speed);
}

void MotorController::stop() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    
    currentSpeed = 0;
    currentDirection = 2; // Остановлен
    
    Serial.println("[MotorController] Stopped");
}

void MotorController::stopSmoothly(uint8_t deceleration) {
    if (currentSpeed == 0) return;
    
    Serial.println("[MotorController] Smooth stopping...");
    
    // Постепенно уменьшаем скорость
    for (int s = currentSpeed; s >= 0; s -= deceleration) {
        if (s < 0) s = 0;
        
        setMotorSpeed(ENA, s, leftMotorCorrection);
        setMotorSpeed(ENB, s, rightMotorCorrection);
        delay(50);
    }
    
    stop();
}

void MotorController::setSpeed(uint8_t speed) {
    if (speed > MAX_SPEED) speed = MAX_SPEED;
    if (currentSpeed == 0) return; // Не меняем скорость если остановлены
    
    currentSpeed = speed;
    
    // Обновляем скорость если движемся
    if (currentDirection != 2) {
        setMotorSpeed(ENA, speed, leftMotorCorrection);
        setMotorSpeed(ENB, speed, rightMotorCorrection);
        
        Serial.print("[MotorController] Speed changed to ");
        Serial.println(speed);
    }
}

uint8_t MotorController::getCurrentSpeed() const {
    return currentSpeed;
}

uint8_t MotorController::getCurrentDirection() const {
    return currentDirection;
}

void MotorController::calibrate(float leftCorrection, float rightCorrection) {
    leftMotorCorrection = constrain(leftCorrection, 0.8f, 1.2f);
    rightMotorCorrection = constrain(rightCorrection, 0.8f, 1.2f);
    
    Serial.print("[MotorController] Calibrated: L=");
    Serial.print(leftMotorCorrection);
    Serial.print(", R=");
    Serial.println(rightMotorCorrection);
}

void MotorController::test() {
    Serial.println("[MotorController] Starting motor test...");
    
    // Тест вперед
    Serial.println("Test: Forward");
    move(0, 150);
    delay(1000);
    
    // Тест назад
    Serial.println("Test: Backward");
    move(1, 150);
    delay(1000);
    
    // Тест поворота вправо
    Serial.println("Test: Turn right 90");
    turn(90);
    delay(500);
    
    // Тест поворота влево
    Serial.println("Test: Turn left 90");
    turn(-90);
    delay(500);
    
    // Плавный останов
    Serial.println("Test: Smooth stop");
    stopSmoothly();
    
    Serial.println("[MotorController] Test completed");
}

// PRIVATE METHODS

void MotorController::setMotorDirection(uint8_t in1, uint8_t in2, bool forward) {
    if (forward) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    }
}

void MotorController::setMotorSpeed(uint8_t speedPin, uint8_t speed, float correction) {
    uint8_t correctedSpeed = static_cast<uint8_t>(speed * correction);
    correctedSpeed = constrain(correctedSpeed, 0, MAX_SPEED);
    analogWrite(speedPin, correctedSpeed);
}

uint32_t MotorController::calculateTurnTime(int16_t angle) const {
    // Базовое время поворота + поправка на скорость
    uint32_t baseTime = angle * TURN_TIME_PER_DEGREE;
    float speedFactor = 255.0f / currentSpeed;
    
    return static_cast<uint32_t>(baseTime * speedFactor);
}