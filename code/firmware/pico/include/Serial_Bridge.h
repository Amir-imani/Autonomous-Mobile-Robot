#ifndef SERIAL_BRIDGE_H
#define SERIAL_BRIDGE_H

#include <Arduino.h>
#include "Robot_State.h"

class Serial_Bridge {
private:
    HardwareSerial* serial_port;
    uint16_t calculateCRC16IBM(const uint8_t* data, size_t length);

public:
    Serial_Bridge(HardwareSerial* port);
    void begin(uint32_t baudrate);
    void sendState(const RobotState& state);
};

#endif