#include <Arduino.h>

#define RIGHT_PWR 4
#define LEFT_PWR 7
#define RIGHT_MOVE 5
#define LEFT_MOVE 6

#define HC_TRIG 11
#define HC_ECHO 10

bool forwardBlocked = false;

void setup() {
  pinMode(LEFT_MOVE, OUTPUT);
  pinMode(LEFT_PWR, OUTPUT);
  pinMode(RIGHT_MOVE, OUTPUT);
  pinMode(RIGHT_PWR, OUTPUT);

  pinMode(HC_TRIG, OUTPUT);
  pinMode(HC_ECHO, INPUT);

  Serial.begin(115200);
  delay(2000);

  Stop();
}

void loop() {
    updateDistanceSafety();

    if (Serial.available() > 0) {
        char command = Serial.read();
        
        switch(command) {
            case 'w': case 'W':
                moveForward();
                break;
            case 's': case 'S':
                moveBackward();
                break;
            case 'a': case 'A':
                turnLeft();
                break;
            case 'd': case 'D':
                turnRight();
                break;
            case ' ': case 'X': case 'x':
                Stop();
                break;
        }
    }

    delay(10);
}

// ВПРЕД
void moveForward() {
    if (!forwardBlocked) {
        digitalWrite(LEFT_MOVE, HIGH);
        digitalWrite(RIGHT_MOVE, HIGH);
        analogWrite(LEFT_PWR, 100);
        analogWrite(RIGHT_PWR, 100);
    } else {
        // Расстояние слишком маленькое - движение вперед заблокировано
        Stop();
        Serial.println("MoveForward blocked");
    }
}

//НАЗАД
void moveBackward() {
  digitalWrite(LEFT_MOVE, LOW);
  digitalWrite(RIGHT_MOVE, LOW);
  analogWrite(LEFT_PWR, 100);
  analogWrite(RIGHT_PWR, 100);
}

// ВЛЕВО
void turnLeft() {
  digitalWrite(LEFT_MOVE, LOW);
  digitalWrite(RIGHT_MOVE, HIGH);
  analogWrite(LEFT_PWR, 100);
  analogWrite(RIGHT_PWR, 100);
}

// ВПРАВО
void turnRight() {
  digitalWrite(LEFT_MOVE, HIGH);
  digitalWrite(RIGHT_MOVE, LOW);
  analogWrite(LEFT_PWR, 100);
  analogWrite(RIGHT_PWR, 100);
}

// СТОП
void Stop() {
  digitalWrite(LEFT_MOVE, LOW);
  digitalWrite(RIGHT_MOVE, LOW);
  analogWrite(LEFT_PWR, 0);
  analogWrite(RIGHT_PWR, 0);
}

float measureDistance() {
    // Отправляем импульс
    digitalWrite(HC_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(HC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(HC_TRIG, LOW);
    
    // Измеряем время отклика
    long duration = pulseIn(HC_ECHO, HIGH, 30000); // Таймаут 30 мс (~5 метров)
    
    // Рассчитываем расстояние
    float distance = duration * 0.034 / 2.0;
    
    // Если нет отклика, возвращаем большое расстояние
    if (duration == 0) {
        distance = 999.0;
    }
    
    return distance;
}

// Проверка и обновление флага блокировки
void updateDistanceSafety() {
    float distance = measureDistance();
    
    // Обновляем флаг блокировки
    forwardBlocked = (distance < 20.0);
    
    // Если расстояние маленькое и робот едет вперед - останавливаем
    if (forwardBlocked) {
        // Проверяем текущее направление
        // Если двигаемся вперед - останавливаем
        if (digitalRead(LEFT_PWR) == HIGH && digitalRead(RIGHT_PWR) == HIGH &&
            digitalRead(RIGHT_MOVE) == HIGH && digitalRead(LEFT_MOVE) == HIGH) {
            Stop();
            Serial.println("Emergency stop");
        }
    }
}
