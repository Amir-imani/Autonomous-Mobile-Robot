#include "Filters.h"
#include <math.h>

// Filters.cpp: single-precision LowPassFilter implementation

LowPassFilter::LowPassFilter(float alpha_val) {
    alpha = alpha_val;
    output = 0.0f;
}

float LowPassFilter::update(float input) {
    output =
        (alpha * input) +
        ((1.0f - alpha) * output);

    return output;
}

// Filters.cpp: single-precision IMU_Filter implementation

IMU_Filter::IMU_Filter(float alpha_val) {
    alpha = alpha_val;
    roll = 0.0f;
    pitch = 0.0f;
    yaw = 0.0f;
    last_time = 0U;
}

void IMU_Filter::update(
    float ax,
    float ay,
    float az,
    float gx,
    float gy,
    float gz
) {
    const uint32_t current_time = micros();

    if (last_time == 0U) {
        last_time = current_time;
        return;
    }

    const float dt =
        static_cast<float>(current_time - last_time) / 1000000.0f;

    last_time = current_time;

    const float roll_acc =
        atan2f(ay, az) * 57.2957795f;

    const float horizontal_acceleration =
        sqrtf((ay * ay) + (az * az));

    const float pitch_acc =
        atan2f(-ax, horizontal_acceleration) * 57.2957795f;

    roll =
        alpha * (roll + (gx * dt)) +
        (1.0f - alpha) * roll_acc;

    pitch =
        alpha * (pitch + (gy * dt)) +
        (1.0f - alpha) * pitch_acc;

    yaw += gz * dt;
}

float IMU_Filter::getRoll() { return roll; }
float IMU_Filter::getPitch() { return pitch; }
float IMU_Filter::getYaw() { return yaw; }