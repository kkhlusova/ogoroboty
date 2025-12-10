#include "uart_protocol.h"

UARTProtocol::UARTProtocol() 
    : rxIndex(0), receivingPacket(false) {
}

void UARTProtocol::begin() {
    Serial.begin(UART_BAUD_RATE);
    while (!Serial) {
        ; // Ждем инициализации
    }
}

bool UARTProtocol::available() {
    return Serial.available() > 0;
}

uint8_t UARTProtocol::calculateCRC(const uint8_t* data, uint8_t length) {
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

bool UARTProtocol::readCommand(ReceivedCommand& cmd) {
    while (Serial.available()) {
        uint8_t byte = Serial.read();
        
        if (!receivingPacket) {
            // Ожидаем заголовок
            if (rxIndex == 0 && byte == PACKET_HEADER_1) {
                rxBuffer[rxIndex++] = byte;
            } else if (rxIndex == 1 && byte == PACKET_HEADER_2) {
                rxBuffer[rxIndex++] = byte;
                receivingPacket = true;
            } else {
                rxIndex = 0; // Сброс при неверном заголовке
            }
        } else {
            // Принимаем пакет
            rxBuffer[rxIndex++] = byte;
            
            // Минимальный полный пакет: HEADER(2) + TYPE(1) + LENGTH(1) + CRC(1) + FOOTER(2) = 7
            if (rxIndex >= 7) {
                // Проверяем завершение пакета
                if (rxBuffer[rxIndex-2] == PACKET_FOOTER_1 && 
                    rxBuffer[rxIndex-1] == PACKET_FOOTER_2) {
                    
                    // Извлекаем поля
                    CommandType type = static_cast<CommandType>(rxBuffer[2]);
                    uint8_t length = rxBuffer[3];
                    uint8_t crcReceived = rxBuffer[4 + length];
                    
                    // Проверяем длину
                    if (length > 32 || (4 + length + 3) != rxIndex) {
                        // Неверная длина
                        receivingPacket = false;
                        rxIndex = 0;
                        return false;
                    }
                    
                    // Проверяем CRC
                    uint8_t crcCalculated = calculateCRC(&rxBuffer[2], length + 2); // TYPE + LENGTH + DATA
                    if (crcCalculated != crcReceived) {
                        sendError(0x01, "CRC error");
                        receivingPacket = false;
                        rxIndex = 0;
                        return false;
                    }
                    
                    // Копируем данные
                    cmd.type = type;
                    cmd.length = length;
                    memcpy(cmd.data, &rxBuffer[4], length);
                    
                    // Сбрасываем состояние
                    receivingPacket = false;
                    rxIndex = 0;
                    
                    return true;
                }
            }
            
            // Защита от переполнения
            if (rxIndex >= sizeof(rxBuffer)) {
                receivingPacket = false;
                rxIndex = 0;
            }
        }
    }
    
    return false;
}

void UARTProtocol::sendPacket(CommandType type, const uint8_t* data, uint8_t length) {
    uint8_t packet[64];
    uint8_t index = 0;
    
    // Заголовок
    packet[index++] = PACKET_HEADER_1;
    packet[index++] = PACKET_HEADER_2;
    
    // Тип и длина
    packet[index++] = static_cast<uint8_t>(type);
    packet[index++] = length;
    
    // Данные
    if (data && length > 0) {
        memcpy(&packet[index], data, length);
        index += length;
    }
    
    // CRC (для TYPE + LENGTH + DATA)
    packet[index++] = calculateCRC(&packet[2], length + 2);
    
    // Завершение
    packet[index++] = PACKET_FOOTER_1;
    packet[index++] = PACKET_FOOTER_2;
    
    // Отправка
    Serial.write(packet, index);
    Serial.flush();
}

void UARTProtocol::sendAck(CommandType originalCmd, bool success) {
    uint8_t ackData[2] = {
        static_cast<uint8_t>(originalCmd),
        success ? 0x01 : 0x00
    };
    sendPacket(RESP_ACK, ackData, 2);
}

void UARTProtocol::sendSensorData(const SensorData& data) {
    uint8_t sensorData[sizeof(SensorData)];
    memcpy(sensorData, &data, sizeof(SensorData));
    sendPacket(RESP_SENSOR_DATA, sensorData, sizeof(SensorData));
}

void UARTProtocol::sendError(uint8_t errorCode, const char* message) {
    uint8_t errorData[33] = {errorCode};
    uint8_t length = 1;
    
    if (message) {
        uint8_t msgLen = min(strlen(message), 31);
        memcpy(&errorData[1], message, msgLen);
        length += msgLen;
        errorData[length] = '\0'; // Null-terminator
    }
    
    sendPacket(RESP_ERROR, errorData, length);
}