#include "Encoder_AS5600.h"

#include <math.h>

Encoder_AS5600::Encoder_AS5600(
    TwoWire* bus,
    bool invert,
    float gear_ratio
)
    : i2c_bus(bus),
      last_raw_angle(0L),
      continuous_angle(0L),
      last_continuous_angle(0L),
      last_time(0U),
      current_rpm(0.0f),
      is_inverted(invert),
      ratio(gear_ratio) {
}

bool Encoder_AS5600::readRawAngle(uint16_t& angle) {
    if (i2c_bus == nullptr) {
        return false;
    }

    i2c_bus->beginTransmission(AS5600_ADDRESS);
    i2c_bus->write(0x0CU);

    if (i2c_bus->endTransmission(false) != 0U) {
        return false;
    }

    const uint8_t received = i2c_bus->requestFrom(
        AS5600_ADDRESS,
        static_cast<uint8_t>(2U)
    );

    if ((received != 2U) || (i2c_bus->available() < 2)) {
        return false;
    }

    const int high_byte = i2c_bus->read();
    const int low_byte = i2c_bus->read();

    if ((high_byte < 0) || (low_byte < 0)) {
        return false;
    }

    angle = static_cast<uint16_t>(
        (
            (static_cast<uint16_t>(high_byte) << 8U) |
            static_cast<uint16_t>(low_byte)
        ) &
        0x0FFFU
    );

    return true;
}

bool Encoder_AS5600::begin() {
    if ((i2c_bus == nullptr) ||
        !isfinite(ratio) ||
        (ratio <= 0.0f)) {
        current_rpm = 0.0f;
        return false;
    }

    uint16_t initial_angle = 0U;

    if (!readRawAngle(initial_angle)) {
        current_rpm = 0.0f;
        return false;
    }

    last_raw_angle = static_cast<long>(initial_angle);
    continuous_angle = 0L;
    last_continuous_angle = 0L;

    current_rpm = 0.0f;
    last_time = micros();

    return true;
}

bool Encoder_AS5600::update() {
    uint16_t current_raw_angle = 0U;

    if (!readRawAngle(current_raw_angle)) {
        // Preserve the last valid state and report the I2C failure.
        return false;
    }

    const uint32_t current_time = micros();

    long delta_angle =
        static_cast<long>(current_raw_angle) -
        last_raw_angle;

    if (delta_angle >
        static_cast<long>(AS5600_HALF_REVOLUTION)) {
        delta_angle -=
            static_cast<long>(
                AS5600_COUNTS_PER_REVOLUTION
            );
    } else if (
        delta_angle <
        -static_cast<long>(AS5600_HALF_REVOLUTION)
    ) {
        delta_angle +=
            static_cast<long>(
                AS5600_COUNTS_PER_REVOLUTION
            );
    }

    if (is_inverted) {
        delta_angle = -delta_angle;
    }

    continuous_angle += delta_angle;
    last_raw_angle = static_cast<long>(
        current_raw_angle
    );

    const float delta_time_seconds =
        static_cast<float>(
            current_time - last_time
        ) /
        1000000.0f;

    if (delta_time_seconds >=
        RPM_UPDATE_INTERVAL_SECONDS) {
        const long angle_difference =
            continuous_angle -
            last_continuous_angle;

        current_rpm =
            (
                static_cast<float>(angle_difference) *
                60.0f
            ) /
            (
                static_cast<float>(
                    AS5600_COUNTS_PER_REVOLUTION
                ) *
                delta_time_seconds *
                ratio
            );

        last_time = current_time;
        last_continuous_angle =
            continuous_angle;
    }

    return true;
}

float Encoder_AS5600::getRPM() const {
    return current_rpm;
}

float Encoder_AS5600::getTotalRevolutions() const {
    if (!isfinite(ratio) || (ratio <= 0.0f)) {
        return 0.0f;
    }

    return
        static_cast<float>(continuous_angle) /
        (
            static_cast<float>(
                AS5600_COUNTS_PER_REVOLUTION
            ) *
            ratio
        );
}

int32_t Encoder_AS5600::getContinuousTicks() const {
    return static_cast<int32_t>(continuous_angle);
}

uint16_t Encoder_AS5600::getRawAngle() const {
    return static_cast<uint16_t>(
        last_raw_angle
    );
}
