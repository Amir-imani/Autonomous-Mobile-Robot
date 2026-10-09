// Robot_State.h
#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <stddef.h>
#include <stdint.h>
#include <limits>
#include <type_traits>

#if !defined(__BYTE_ORDER__) || (__BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__)
#error "RobotState requires a little-endian target"
#endif

enum RobotDiagnosticFlag : uint32_t {
    DIAGNOSTIC_COMMAND_RECEIVED = 1UL << 0U,
    DIAGNOSTIC_COMMAND_FRESH = 1UL << 1U,
    DIAGNOSTIC_DRIVE_ENABLED = 1UL << 2U,
    DIAGNOSTIC_SAFE_STATE_ACTIVE = 1UL << 3U,
    DIAGNOSTIC_LEFT_ENCODER_VALID = 1UL << 4U,
    DIAGNOSTIC_RIGHT_ENCODER_VALID = 1UL << 5U,
    DIAGNOSTIC_ENCODER_DATA_STALE = 1UL << 6U,
    DIAGNOSTIC_I2C_FAULT_LATCHED = 1UL << 7U,
    DIAGNOSTIC_SYSTEM_POWER_VALID = 1UL << 8U,
    DIAGNOSTIC_MOTORS_POWER_VALID = 1UL << 9U,
    DIAGNOSTIC_LIDAR_VALID = 1UL << 10U,
    DIAGNOSTIC_BENCH_MODE_ACTIVE = 1UL << 11U
};

enum RobotSystemStatusCode : uint8_t {
    SYSTEM_STATUS_READY = 0U,
    SYSTEM_STATUS_I2C_FAULT = 1U,
    SYSTEM_STATUS_ENCODER_STALE = 2U,
    SYSTEM_STATUS_COMMAND_STALE = 3U,
    SYSTEM_STATUS_DRIVE_DISABLED = 4U
};

struct __attribute__((packed, aligned(4))) RobotState {
    uint32_t timestamp_ms;          // Offset 0

    float left_rpm;                 // Offset 4
    float right_rpm;                // Offset 8
    float left_total_revs;          // Offset 12
    float right_total_revs;         // Offset 16

    float roll;                     // Offset 20
    float pitch;                    // Offset 24
    float yaw;                      // Offset 28
    float heading_mag;              // Offset 32

    float lidar_distance_mm;        // Offset 36
    float lidar_angle_deg;          // Offset 40

    float system_voltage_v;         // Offset 44
    float system_current_a;         // Offset 48
    float motors_voltage_v;         // Offset 52
    float motors_current_a;         // Offset 56

    int32_t left_encoder_ticks;      // Offset 60
    int32_t right_encoder_ticks;     // Offset 64
    uint32_t diagnostic_flags;       // Offset 68

    uint8_t lidar_is_valid;         // Offset 72: 0=false, 1=true
    uint8_t system_status_code;     // Offset 73
    uint16_t reserved_0;            // Offset 74: must be zero
};

static_assert(sizeof(float) == 4,
              "Wire protocol requires 32-bit float");
static_assert(std::numeric_limits<float>::is_iec559,
              "Wire protocol requires IEEE-754 float");
static_assert(std::is_standard_layout<RobotState>::value,
              "RobotState must have standard layout");
static_assert(std::is_trivially_copyable<RobotState>::value,
              "RobotState must be trivially copyable");

static_assert(alignof(RobotState) == 4, "Unexpected RobotState alignment");
static_assert(sizeof(RobotState) == 76, "Unexpected RobotState payload size");

static_assert(offsetof(RobotState, timestamp_ms) == 0, "Layout mismatch");
static_assert(offsetof(RobotState, left_rpm) == 4, "Layout mismatch");
static_assert(offsetof(RobotState, right_rpm) == 8, "Layout mismatch");
static_assert(offsetof(RobotState, left_total_revs) == 12, "Layout mismatch");
static_assert(offsetof(RobotState, right_total_revs) == 16, "Layout mismatch");
static_assert(offsetof(RobotState, roll) == 20, "Layout mismatch");
static_assert(offsetof(RobotState, pitch) == 24, "Layout mismatch");
static_assert(offsetof(RobotState, yaw) == 28, "Layout mismatch");
static_assert(offsetof(RobotState, heading_mag) == 32, "Layout mismatch");
static_assert(offsetof(RobotState, lidar_distance_mm) == 36, "Layout mismatch");
static_assert(offsetof(RobotState, lidar_angle_deg) == 40, "Layout mismatch");
static_assert(offsetof(RobotState, system_voltage_v) == 44, "Layout mismatch");
static_assert(offsetof(RobotState, system_current_a) == 48, "Layout mismatch");
static_assert(offsetof(RobotState, motors_voltage_v) == 52, "Layout mismatch");
static_assert(offsetof(RobotState, motors_current_a) == 56, "Layout mismatch");
static_assert(offsetof(RobotState, left_encoder_ticks) == 60,
              "Layout mismatch");
static_assert(offsetof(RobotState, right_encoder_ticks) == 64,
              "Layout mismatch");
static_assert(offsetof(RobotState, diagnostic_flags) == 68,
              "Layout mismatch");
static_assert(offsetof(RobotState, lidar_is_valid) == 72, "Layout mismatch");
static_assert(offsetof(RobotState, system_status_code) == 73,
              "Layout mismatch");
static_assert(offsetof(RobotState, reserved_0) == 74, "Layout mismatch");

extern RobotState robot_state;

#endif
