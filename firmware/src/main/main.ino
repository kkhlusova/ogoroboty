#include "uart_protocol.h"
#include "motor_controller.h"
#include "sensor_manager.h"

UARTProtocol uart;
MotorController motors;
SensorManager sensors;

SensorData currentSensorData;

void setup() {
    // Инициализация всех систем
    uart.begin();
    motors.init();
    sensors.init();
    
    // Тестовый сигнал
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    
    // Отправка начального ACK
    uart.sendAck(CMD_PING);
}

void loop() {
    // Чтение команд с UART
    ReceivedCommand cmd;
    if (uart.readCommand(cmd)) {
        processCommand(cmd);
    }
    
    // Периодический опрос датчиков
    static uint32_t lastSensorUpdate = 0;
    if (millis() - lastSensorUpdate > 100) { // 10 Hz
        updateSensorData();
        lastSensorUpdate = millis();
    }
    
    // Автономная безопасность
    checkSafety();
}

void processCommand(const ReceivedCommand& cmd) {
    switch (cmd.type) {
        case CMD_PING:
            uart.sendAck(CMD_PING);
            break;
            
        case CMD_MOVE:
            if (cmd.length >= 2) {
                uint8_t direction = cmd.data[0]; // 0=вперед, 1=назад
                uint8_t speed = cmd.data[1];
                motors.move(direction, speed);
                uart.sendAck(CMD_MOVE);
            }
            break;
            
        case CMD_TURN:
            if (cmd.length >= 2) {
                int16_t angle = (cmd.data[0] << 8) | cmd.data[1];
                motors.turn(angle);
                uart.sendAck(CMD_TURN);
            }
            break;
            
        case CMD_STOP:
            motors.stop();
            uart.sendAck(CMD_STOP);
            break;
            
        case CMD_GET_SENSORS:
            uart.sendSensorData(currentSensorData);
            break;
            
        case CMD_SET_SPEED:
            if (cmd.length >= 1) {
                motors.setSpeed(cmd.data[0]);
                uart.sendAck(CMD_SET_SPEED);
            }
            break;
            
        case CMD_BUZZER:
            if (cmd.length >= 1) {
                // Управление зуммером
                tone(BUZZER_PIN, cmd.data[0] * 100, 200);
                uart.sendAck(CMD_BUZZER);
            }
            break;
            
        default:
            uart.sendError(0xFF, "Unknown command");
            break;
    }
}

void updateSensorData() {
    currentSensorData.distance_front = sensors.getDistanceFront();
    currentSensorData.distance_left = sensors.getDistanceLeft();
    currentSensorData.distance_right = sensors.getDistanceRight();
    currentSensorData.temperature = sensors.getTemperature();
    currentSensorData.humidity = sensors.getHumidity();
    currentSensorData.battery_voltage = sensors.getBatteryVoltage();
    currentSensorData.timestamp = millis();
}

void checkSafety() {
    // Автоматическая остановка при препятствии
    if (currentSensorData.distance_front < 20) { // 20 см
        motors.stop();
        uart.sendError(0x10, "Obstacle detected");
    }
}
