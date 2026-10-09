#include "Laser_Scanner.h"

#include "Config.h"

Laser_Scanner::Laser_Scanner(HardwareSerial* port) {
    serial_port = port;
    counter = 0;
    distance_mm = 0;
    range_status = 0;
    new_data_ready = false;
}

void Laser_Scanner::begin() {
    static constexpr uint8_t LONG_RANGE_COMMAND[] = {
        0xA5U, 0x54U, 0x03U, 0xFCU
    };
    static constexpr uint8_t FAST_TIMING_COMMAND[] = {
        0xA5U, 0x10U, 0x00U, 0xB5U
    };
    static constexpr uint8_t ROI_8X8_COMMAND[] = {
        0xA5U, 0xBFU, 0x01U, 0x65U
    };
    static constexpr uint8_t CONTINUOUS_OUTPUT_COMMAND[] = {
        0xA5U, 0x65U, 0x01U, 0x0BU
    };

    serial_port->begin(LASER_BAUDRATE);
    delay(1000U);

    serial_port->write(LONG_RANGE_COMMAND,sizeof(LONG_RANGE_COMMAND));
    delay(20U);
    serial_port->write(FAST_TIMING_COMMAND,sizeof(FAST_TIMING_COMMAND));
    delay(20U);
    serial_port->write(ROI_8X8_COMMAND,sizeof(ROI_8X8_COMMAND));
    delay(20U);
    serial_port->write(CONTINUOUS_OUTPUT_COMMAND,sizeof(CONTINUOUS_OUTPUT_COMMAND));
    serial_port->flush();
}

void Laser_Scanner::update() {
    while (serial_port->available() > 0) {
        uint8_t incoming = serial_port->read();
        if (counter == 0) { if (incoming == 0x5A) { buffer[0] = incoming; counter = 1; } }
        else if (counter == 1) { if (incoming == 0x5A) { buffer[1] = incoming; counter = 2; } else { counter = 0; } }
        else if (counter == 2) { if (incoming == 0x15) { buffer[2] = incoming; counter = 3; } else { counter = 0; } }
        else if (counter == 3) { if (incoming == 0x03) { buffer[3] = incoming; counter = 4; } else { counter = 0; } }
        else { buffer[counter] = incoming; counter++; if (counter == 8) { parsePacket(); counter = 0; } }
    }
}

void Laser_Scanner::parsePacket() {
    uint8_t sum = 0;
    for (int i = 0; i < 7; i++) sum += buffer[i];
    if (sum == buffer[7]) {
        distance_mm = (buffer[4] << 8) | buffer[5];
        range_status = (buffer[6] >> 4) & 0x0F;
        new_data_ready = true;
    }
}
uint16_t Laser_Scanner::getDistanceMM() { new_data_ready = false; return distance_mm; }
bool Laser_Scanner::isDataValid() { return (range_status == 0); }
bool Laser_Scanner::hasNewData() { return new_data_ready; }
