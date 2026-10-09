#ifndef LASER_SCANNER_H
#define LASER_SCANNER_H

#include <Arduino.h>

class Laser_Scanner {
private:
    HardwareSerial* serial_port;
    uint8_t buffer[8];
    uint8_t counter;
    
    uint16_t distance_mm;
    uint8_t range_status;
    bool new_data_ready;

    void parsePacket();

public:
    Laser_Scanner(HardwareSerial* port);
    
    void begin();
    void update();
    
    uint16_t getDistanceMM();
    bool isDataValid();
    bool hasNewData();
};

#endif
