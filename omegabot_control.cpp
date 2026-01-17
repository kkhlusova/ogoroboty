// motor_control.ino

// Пины для левого мотора
const int LEFT_MOVE = 6;
const int LEFT_PWR = 7;

// Пины для правого мотора
const int RIGHT_MOVE = 5;
const int RIGHT_PWR = 4;

void setup() {
  // Настраиваем пины как выходы
  pinMode(LEFT_MOVE, OUTPUT);
  pinMode(LEFT_PWR, OUTPUT);
  pinMode(RIGHT_MOVE, OUTPUT);
  pinMode(RIGHT_PWR, OUTPUT);

  // Инициализируем Serial (UART) на скорости 9600 бод
  Serial.begin(9600);

  // Останавливаем моторы при старте
  Stop();
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();

    switch (command) {
      case 'W': // Вперёд
        moveForward();
        break;
      case 'S': // Назад
        moveBackward();
        break;
      case 'A': // Влево (поворот на месте)
        moveLeft();
        break;
      case 'D': // Вправо (поворот на месте)
        moveRight();
        break;
      case 'X':
        Stop();
        break;
    }
  }
  
}

// ВПРЕД
void moveForward() {
  digitalWrite(LEFT_MOVE, HIGH);
  digitalWrite(RIGHT_MOVE, HIGH);
  analogWrite(LEFT_PWR, 200);
  analogWrite(RIGHT_PWR, 200);
}

//НАЗАД
void moveBackward() {
  digitalWrite(LEFT_MOVE, HIGH);
  digitalWrite(RIGHT_MOVE, HIGH);
  analogWrite(LEFT_PWR, -200);
  analogWrite(RIGHT_PWR, -200);
}

// НАЛЕВО ПОВОРОТ
void moveLeft() {
  digitalWrite(LEFT_MOVE, LOW);
  digitalWrite(RIGHT_MOVE, HIGH);
  analogWrite(LEFT_PWR, 0);
  analogWrite(RIGHT_PWR, 200);
}

// ВПРАВО
void moveRight() {
  digitalWrite(LEFT_MOVE, HIGH);
  digitalWrite(RIGHT_MOVE, LOW);
  analogWrite(LEFT_PWR, 200);
  analogWrite(RIGHT_PWR, 0);
}

// Левые вперед
void moveLeftBack() {
  digitalWrite(LEFT_MOVE, LOW);
  digitalWrite(RIGHT_MOVE, HIGH);
  analogWrite(LEFT_PWR, 0);
  analogWrite(RIGHT_PWR, -200);
}

// Правые вперед
void moveRightBack() {
  digitalWrite(LEFT_MOVE, HIGH);
  digitalWrite(RIGHT_MOVE, LOW);
  analogWrite(LEFT_PWR, -200);
  analogWrite(RIGHT_PWR, 0);
}

// СТОП
void Stop() {
  digitalWrite(LEFT_MOVE, LOW);
  digitalWrite(RIGHT_MOVE, LOW);
  analogWrite(LEFT_PWR, 0);
  analogWrite(RIGHT_PWR, 0);
}
