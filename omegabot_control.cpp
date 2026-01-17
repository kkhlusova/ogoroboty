#include <Arduino.h>

// Определение пинов для двигателей
const int LEFT_MOTOR_FORWARD = 6;   // Левый двигатель движение
const int LEFT_MOTOR_STOP = 7;      // Левый двигатель стоп
const int RIGHT_MOTOR_FORWARD = 5;  // Правый двигатель движение
const int RIGHT_MOTOR_STOP = 4;     // Правый двигатель стоп

void setup() {
  // Инициализация пинов как выходов
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_STOP, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_STOP, OUTPUT);
  
  // Инициализация последовательного соединения
  Serial.begin(9600);
  
  // Устанавливаем начальное состояние двигателей (остановлены)
  stopRobot();
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    switch(command) {
      case 'W':
      case 'w':
        moveForward();
        break;
      case 'S':
      case 's':
        moveBackward();
        break;
      case 'A':
      case 'a':
        turnLeft();
        break;
      case 'D':
      case 'd':
        turnRight();
        break;
      case 'X':
      case 'x':
        stopRobot();
        break;
      default:
        // Неизвестная команда - останавливаем робота
        stopRobot();
        break;
    }
  }
}

// Функция движения вперед
void moveForward() {
  digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
  digitalWrite(LEFT_MOTOR_STOP, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_STOP, LOW);
}

// Функция движения назад
void moveBackward() {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_STOP, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_STOP, HIGH);
}

// Функция поворота влево
void turnLeft() {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_STOP, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_STOP, LOW);
}

// Функция поворота вправо
void turnRight() {
  digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
  digitalWrite(LEFT_MOTOR_STOP, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_STOP, HIGH);
}

// Функция остановки
void stopRobot() {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_STOP, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_STOP, LOW);
}