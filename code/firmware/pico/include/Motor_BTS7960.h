#ifndef MOTOR_BTS7960_H
#define MOTOR_BTS7960_H

#include <Arduino.h>

class Motor_BTS7960 {
private:
    static constexpr int MIN_PWM_LIMIT = 0;
    static constexpr int MAX_PWM_LIMIT = 255;

    uint8_t pin_R_EN;
    uint8_t pin_L_EN;
    uint8_t pin_R_PWM;
    uint8_t pin_L_PWM;

    int max_pwm_limit;
    bool is_inverted;

public:
    Motor_BTS7960(
        uint8_t r_en,
        uint8_t l_en,
        uint8_t r_pwm,
        uint8_t l_pwm,
        bool invert = false
    );

    // Initializes the pins with the driver disabled.
    void begin();

    // Accepts a requested motor output from -100% to +100%.
    void drive(float power_percentage);

    // Sets both PWM outputs to zero and de-asserts both enable pins.
    void stop();

    // Limits the maximum PWM value to the range 0..255.
    void setSafetyLimit(int limit);
};

#endif