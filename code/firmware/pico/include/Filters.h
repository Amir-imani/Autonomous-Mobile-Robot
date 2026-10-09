#ifndef FILTERS_H
#define FILTERS_H

#include <Arduino.h>


class LowPassFilter {
private:
    float alpha; 
    float output;

public:
    LowPassFilter(float alpha_val = 0.1);
    float update(float input);
};


class IMU_Filter {
private:
    float alpha; 
    float roll, pitch, yaw;
    unsigned long last_time;

public:
    IMU_Filter(float alpha_val = 0.98);
    
    void update(float ax, float ay, float az, float gx, float gy, float gz);
    
    float getRoll();
    float getPitch();
    float getYaw();
};

#endif