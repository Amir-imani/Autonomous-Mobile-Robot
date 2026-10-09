#include "Serial_Bridge.h"

Serial_Bridge::Serial_Bridge(HardwareSerial* port) {
    serial_port = port;
}

void Serial_Bridge::begin(uint32_t baudrate) {
    serial_port->begin(baudrate);
}

uint16_t Serial_Bridge::calculateCRC16IBM(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0U; i < length; ++i) {
        crc ^= (uint16_t)data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            if ((crc & 0x0001U) != 0U) crc = (crc >> 1U) ^ 0xA001U;
            else crc = crc >> 1U;
        }
    }
    return crc;
}

void Serial_Bridge::sendState(const RobotState& state) {
    serial_port->write(0xAA);
    serial_port->write(0xBB);
    const uint8_t* payload = (const uint8_t*)&state;
    serial_port->write(payload, sizeof(RobotState));
    uint16_t checksum = calculateCRC16IBM(payload, sizeof(RobotState));
    serial_port->write((uint8_t*)&checksum, 2);
}
