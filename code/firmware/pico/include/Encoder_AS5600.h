#ifndef ENCODER_AS5600_H
#define ENCODER_AS5600_H

#include <Arduino.h>
#include <Wire.h>

class Encoder_AS5600 {
private:
    static constexpr uint8_t AS5600_ADDRESS = 0x36U;
    static constexpr uint16_t AS5600_COUNTS_PER_REVOLUTION = 4096U;
    static constexpr uint16_t AS5600_HALF_REVOLUTION = 2048U;
    static constexpr float RPM_UPDATE_INTERVAL_SECONDS = 0.05f;

    TwoWire* i2c_bus;

    long last_raw_angle;
    long continuous_angle;
    long last_continuous_angle;

    uint32_t last_time;
    float current_rpm;

    bool is_inverted;
    float ratio;

    bool readRawAngle(uint16_t& angle);

public:
    Encoder_AS5600(
        TwoWire* bus,
        bool invert = false,
        float gear_ratio = 1.0f
    );

    bool begin();

    // Returns true only when the I2C angle read succeeds.
    bool update();

    float getRPM() const;
    float getTotalRevolutions() const;
    int32_t getContinuousTicks() const;

    // Returns the most recent valid 12-bit sensor angle without another read.
    uint16_t getRawAngle() const;
};

#endif
