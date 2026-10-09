#ifndef SERVO_MANAGER_H
#define SERVO_MANAGER_H

#include <Arduino.h>
#include <Servo.h>

class Servo_Manager {
private:
    uint8_t pin;
    Servo internal_servo;

    float current_angle;
    float target_angle;
    float max_speed_degrees_per_second;
    uint16_t min_pulse;
    uint16_t max_pulse;

    uint32_t last_update_time_ms;
    uint16_t update_interval_ms;

public:
    Servo_Manager(
        uint8_t servo_pin,
        float max_speed_deg_per_sec = 90.0f,
        uint16_t min_p = 1000U,
        uint16_t max_p = 2000U,
        uint16_t control_interval_ms = 20U
    );
    
    void begin();
    void setTargetAngle(float angle);
    void update();
    float getCurrentAngle();
    bool isAtTarget();
};

#endif
