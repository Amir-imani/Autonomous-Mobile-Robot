#ifndef IMU_9DOF_H
#define IMU_9DOF_H

#include <Wire.h>
#include <Arduino.h>

class IMU_9DOF {
private:
    TwoWire* i2c_bus;
    const uint8_t MPU9250_ADDR = 0x68;
    const uint8_t AK8963_ADDR = 0x0C;

    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t mx, my, mz;

    void writeRegister(uint8_t dev_addr, uint8_t reg, uint8_t val);
    void readRegisters(uint8_t dev_addr, uint8_t reg, uint8_t count, uint8_t* dest);

public:
    IMU_9DOF(TwoWire* bus);
    bool begin();
    void update(); 

    void getAccel(float& x, float& y, float& z);
    void getGyro(float& x, float& y, float& z);
    void getMag(float& x, float& y, float& z);
};

#endif