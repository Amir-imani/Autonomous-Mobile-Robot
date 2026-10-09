#include "IMU_9DOF.h"

IMU_9DOF::IMU_9DOF(TwoWire* bus) {
    i2c_bus = bus;
    ax = ay = az = gx = gy = gz = mx = my = mz = 0;
}

void IMU_9DOF::writeRegister(uint8_t dev_addr, uint8_t reg, uint8_t val) {
    i2c_bus->beginTransmission(dev_addr);
    i2c_bus->write(reg);
    i2c_bus->write(val);
    i2c_bus->endTransmission();
}

void IMU_9DOF::readRegisters(uint8_t dev_addr, uint8_t reg, uint8_t count, uint8_t* dest) {
    i2c_bus->beginTransmission(dev_addr);
    i2c_bus->write(reg);
    if (i2c_bus->endTransmission(false) == 0) {
        i2c_bus->requestFrom(dev_addr, count);
        for (uint8_t i = 0; i < count && i2c_bus->available(); i++) {
            dest[i] = i2c_bus->read();
        }
    }
}

bool IMU_9DOF::begin() {
    writeRegister(MPU9250_ADDR, 0x6B, 0x00); 
    delay(10);
    
    writeRegister(MPU9250_ADDR, 0x1B, 0x08); 
    writeRegister(MPU9250_ADDR, 0x1C, 0x08); 

    writeRegister(MPU9250_ADDR, 0x6A, 0x00); 
    writeRegister(MPU9250_ADDR, 0x37, 0x02); 
    delay(10);

    writeRegister(AK8963_ADDR, 0x0A, 0x00); 
    delay(10);
    writeRegister(AK8963_ADDR, 0x0A, 0x16); 
    delay(10);

    return true;
}

void IMU_9DOF::update() {
    uint8_t raw_buf[14];
    readRegisters(MPU9250_ADDR, 0x3B, 14, raw_buf);

    ax = (raw_buf[0] << 8) | raw_buf[1];
    ay = (raw_buf[2] << 8) | raw_buf[3];
    az = (raw_buf[4] << 8) | raw_buf[5];
    gx = (raw_buf[8] << 8) | raw_buf[9];
    gy = (raw_buf[10] << 8) | raw_buf[11];
    gz = (raw_buf[12] << 8) | raw_buf[13];

    uint8_t status1;
    readRegisters(AK8963_ADDR, 0x02, 1, &status1); 
    
    if (status1 & 0x01) { 
        uint8_t mag_buf[7];
        readRegisters(AK8963_ADDR, 0x03, 7, mag_buf); 
        
        if (!(mag_buf[6] & 0x08)) {
            mx = (mag_buf[1] << 8) | mag_buf[0];
            my = (mag_buf[3] << 8) | mag_buf[2];
            mz = (mag_buf[5] << 8) | mag_buf[4];
        }
    }
}

void IMU_9DOF::getAccel(float& x, float& y, float& z) {
    x = (float)ax / 8192.0; y = (float)ay / 8192.0; z = (float)az / 8192.0;
}

void IMU_9DOF::getGyro(float& x, float& y, float& z) {
    x = (float)gx / 65.5; y = (float)gy / 65.5; z = (float)gz / 65.5;
}

void IMU_9DOF::getMag(float& x, float& y, float& z) {
    x = (float)mx * 0.15; 
    y = (float)my * 0.15;
    z = (float)mz * 0.15;
}