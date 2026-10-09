// main.cpp

#include <Arduino.h>
#include <Wire.h>

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "hardware/watchdog.h"
#include "pico/mutex.h"

#include "Config.h"
#include "Robot_State.h"

#include "Encoder_AS5600.h"
#include "Filters.h"
#include "IMU_9DOF.h"
#include "Laser_Scanner.h"
#include "Motor_BTS7960.h"
#include "Neuro_PID.h"
#include "Power_Monitor.h"
#include "Serial_Bridge.h"
#include "Servo_Manager.h"

// ============================================================================
// Pi 5 command wire protocol
// ============================================================================

static constexpr uint8_t COMMAND_PROTOCOL_VERSION = 1U;
static constexpr uint8_t COMMAND_FLAG_DRIVE_ENABLE = 0x01U;
static constexpr uint8_t COMMAND_ALLOWED_FLAGS =
    COMMAND_FLAG_DRIVE_ENABLE;

struct __attribute__((packed, aligned(4))) RobotCommand {
    uint32_t sequence;          // Offset 0
    float target_left_rpm;      // Offset 4
    float target_right_rpm;     // Offset 8
    uint8_t flags;              // Offset 12
    uint8_t protocol_version;   // Offset 13
    uint16_t reserved_0;        // Offset 14; must be zero
};

static_assert(sizeof(float) == 4U,
              "RobotCommand requires 32-bit float");
static_assert(alignof(RobotCommand) == 4U,
              "Unexpected RobotCommand alignment");
static_assert(sizeof(RobotCommand) == 16U,
              "Unexpected RobotCommand payload size");

static_assert(offsetof(RobotCommand, sequence) == 0U,
              "RobotCommand layout mismatch");
static_assert(offsetof(RobotCommand, target_left_rpm) == 4U,
              "RobotCommand layout mismatch");
static_assert(offsetof(RobotCommand, target_right_rpm) == 8U,
              "RobotCommand layout mismatch");
static_assert(offsetof(RobotCommand, flags) == 12U,
              "RobotCommand layout mismatch");
static_assert(offsetof(RobotCommand, protocol_version) == 13U,
              "RobotCommand layout mismatch");
static_assert(offsetof(RobotCommand, reserved_0) == 14U,
              "RobotCommand layout mismatch");

// ============================================================================
// Hardware configuration
// ============================================================================

static constexpr uint8_t PI_UART_TX_PIN = 16U;
static constexpr uint8_t PI_UART_RX_PIN = 17U;
static constexpr uint32_t PI_UART_BAUDRATE = 921600U;

static constexpr uint8_t SYSTEM_POWER_I2C_ADDRESS = 0x40U;
static constexpr uint8_t MOTORS_POWER_I2C_ADDRESS = 0x40U;

static constexpr uint8_t FRAME_HEADER_1 = 0xAAU;
static constexpr uint8_t FRAME_HEADER_2 = 0xBBU;

static constexpr float MAX_COMMAND_RPM = 100.0f;
static constexpr float ZERO_TARGET_EPSILON_RPM = 0.001f;

static constexpr uint32_t HARDWARE_WATCHDOG_TIMEOUT_MS = 250U;
static constexpr bool BENCH_TEST_MODE_ENABLED =
    BENCH_TEST_MODE != 0;

static_assert(BENCH_TEST_MODE == 0 || BENCH_TEST_MODE == 1,
              "BENCH_TEST_MODE must be 0 or 1");

static constexpr size_t LEFT_WHEEL = 0U;
static constexpr size_t RIGHT_WHEEL = 1U;
static constexpr size_t WHEEL_COUNT = 2U;

static constexpr size_t MAX_COMMAND_BYTES_PER_LOOP = 64U;

// ============================================================================
// Robot state and hardware objects
// ============================================================================

RobotState robot_state{};

Motor_BTS7960 motor_left(
    L_MOTOR_EN_R,
    L_MOTOR_EN_L,
    L_MOTOR_PWM_R,
    L_MOTOR_PWM_L,
    L_MOTOR_INVERT
);

Motor_BTS7960 motor_right(
    R_MOTOR_EN_R,
    R_MOTOR_EN_L,
    R_MOTOR_PWM_R,
    R_MOTOR_PWM_L,
    R_MOTOR_INVERT
);

Encoder_AS5600 encoder_left(
    &Wire,
    true,
    GEAR_RATIO
);

Encoder_AS5600 encoder_right(
    &Wire1,
    false,
    GEAR_RATIO
);

INA226 ina_system_power(SYSTEM_POWER_I2C_ADDRESS, &Wire);
INA219 ina_motors_power(MOTORS_POWER_I2C_ADDRESS, &Wire1);

Power_Monitor system_power_monitor(ina_system_power);
Power_Monitor motors_power_monitor(ina_motors_power);

IMU_9DOF imu(&Wire1);

Servo_Manager servo_lidar(
    PIN_SERVO_LIDAR,
    LIDAR_SERVO_SPEED_DEG_PER_SEC,
    SERVO_MIN_PULSE,
    SERVO_MAX_PULSE,
    SERVO_CONTROL_INTERVAL_MS
);

Laser_Scanner lidar_sensor(&Serial2);

IMU_Filter imu_filter(0.98f);
LowPassFilter rpm_filter_left(0.2f);
LowPassFilter rpm_filter_right(0.2f);

Neuro_PID neuro_controller;

Serial_Bridge telemetry_bridge(&Serial1);

// ============================================================================
// Cross-core Neural-PID exchange
// ============================================================================

struct PIDInputSnapshot {
    float error;
    float derivative_error;
    float integral_error;
};

struct PIDGainSnapshot {
    float kp;
    float ki;
    float kd;
};

struct PIDExchange {
    PIDInputSnapshot inputs[WHEEL_COUNT];
    PIDGainSnapshot gains[WHEEL_COUNT];
    bool safe_state;
};

struct PIDRuntimeState {
    float last_error;
    float filtered_derivative;
    float integral;

    float smooth_kp;
    float smooth_ki;
    float smooth_kd;
};

auto_init_mutex(pid_exchange_mutex);

static PIDExchange pid_exchange{
    {
        PIDInputSnapshot{},
        PIDInputSnapshot{}
    },
    {
        PIDGainSnapshot{},
        PIDGainSnapshot{}
    },
    true
};

static PIDGainSnapshot active_pid_gains[WHEEL_COUNT]{};
static PIDRuntimeState pid_runtime[WHEEL_COUNT]{};

// ============================================================================
// Scheduler and safety state
// ============================================================================

static uint32_t last_time_imu = 0U;
static uint32_t last_time_motor = 0U;
static uint32_t last_time_lidar = 0U;
static uint32_t last_time_telemetry = 0U;
static uint32_t last_time_diagnostic = 0U;

static uint32_t last_valid_sensor_time = 0U;
static uint32_t last_valid_command_time = 0U;

static bool has_valid_serial_command = false;
static bool command_drive_enabled = false;
static bool i2c_fault_latched = false;
static bool safe_state_active = true;
static bool left_encoder_read_valid = false;
static bool right_encoder_read_valid = false;
static bool system_power_read_valid = false;
static bool motors_power_read_valid = false;

static float target_left_rpm = 0.0f;
static float target_right_rpm = 0.0f;

static uint32_t last_command_sequence = 0U;

// ============================================================================
// Non-blocking command parser
// ============================================================================

enum class CommandRxState : uint8_t {
    WaitHeader1 = 0U,
    WaitHeader2,
    ReadPayload,
    ReadCrcLow,
    ReadCrcHigh
};

struct CommandParserState {
    CommandRxState state;
    size_t payload_index;
    uint8_t payload[sizeof(RobotCommand)];
    uint8_t received_crc_low;
};

static CommandParserState command_parser{
    CommandRxState::WaitHeader1,
    0U,
    {},
    0U
};

// CRC-16/IBM reflected polynomial.
// Initial value is 0xFFFF to match the telemetry bridge.
static uint16_t calculateCRC16IBM(
    const uint8_t* data,
    size_t length
) {
    uint16_t crc = 0xFFFFU;

    for (size_t i = 0U; i < length; ++i) {
        crc ^= static_cast<uint16_t>(data[i]);

        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            if ((crc & 0x0001U) != 0U) {
                crc =
                    static_cast<uint16_t>(
                        (crc >> 1U) ^ 0xA001U
                    );
            } else {
                crc = static_cast<uint16_t>(crc >> 1U);
            }
        }
    }

    return crc;
}

static void resetCommandParser() {
    command_parser.state = CommandRxState::WaitHeader1;
    command_parser.payload_index = 0U;
    command_parser.received_crc_low = 0U;
}

static void resetCommandParserWithCandidate(uint8_t candidate) {
    command_parser.payload_index = 0U;
    command_parser.received_crc_low = 0U;

    if (candidate == FRAME_HEADER_1) {
        command_parser.state = CommandRxState::WaitHeader2;
    } else {
        command_parser.state = CommandRxState::WaitHeader1;
    }
}

static bool validateRobotCommand(const RobotCommand& command) {
    if (command.protocol_version != COMMAND_PROTOCOL_VERSION) {
        return false;
    }

    if (command.reserved_0 != 0U) {
        return false;
    }

    if ((command.flags &
         static_cast<uint8_t>(~COMMAND_ALLOWED_FLAGS)) != 0U) {
        return false;
    }

    if (!isfinite(command.target_left_rpm) ||
        !isfinite(command.target_right_rpm)) {
        return false;
    }

    if (fabsf(command.target_left_rpm) > MAX_COMMAND_RPM) {
        return false;
    }

    if (fabsf(command.target_right_rpm) > MAX_COMMAND_RPM) {
        return false;
    }

    return true;
}

static void noteValidatedSerialCommand() {
    last_valid_command_time = millis();
    has_valid_serial_command = true;
}

static void applyValidatedRobotCommand(
    const RobotCommand& command
) {
    const bool drive_enabled =
        (command.flags & COMMAND_FLAG_DRIVE_ENABLE) != 0U;

    if (drive_enabled) {
        target_left_rpm = command.target_left_rpm;
        target_right_rpm = command.target_right_rpm;
    } else {
        target_left_rpm = 0.0f;
        target_right_rpm = 0.0f;
    }

    command_drive_enabled = drive_enabled;
    last_command_sequence = command.sequence;

    noteValidatedSerialCommand();
}

static void processCompletedCommandFrame(
    uint16_t received_crc
) {
    const uint16_t calculated_crc = calculateCRC16IBM(
        command_parser.payload,
        sizeof(command_parser.payload)
    );

    if (received_crc != calculated_crc) {
        return;
    }

    RobotCommand command{};

    memcpy(
        &command,
        command_parser.payload,
        sizeof(command)
    );

    if (!validateRobotCommand(command)) {
        return;
    }

    applyValidatedRobotCommand(command);
}

static void consumeCommandByte(uint8_t byte) {
    switch (command_parser.state) {
        case CommandRxState::WaitHeader1: {
            if (byte == FRAME_HEADER_1) {
                command_parser.state =
                    CommandRxState::WaitHeader2;
            }
            break;
        }

        case CommandRxState::WaitHeader2: {
            if (byte == FRAME_HEADER_2) {
                command_parser.payload_index = 0U;
                command_parser.state =
                    CommandRxState::ReadPayload;
            } else if (byte == FRAME_HEADER_1) {
                // Preserve an overlapping first header byte.
                command_parser.state =
                    CommandRxState::WaitHeader2;
            } else {
                command_parser.state =
                    CommandRxState::WaitHeader1;
            }
            break;
        }

        case CommandRxState::ReadPayload: {
            command_parser.payload[
                command_parser.payload_index
            ] = byte;

            ++command_parser.payload_index;

            if (command_parser.payload_index >=
                sizeof(command_parser.payload)) {
                command_parser.state =
                    CommandRxState::ReadCrcLow;
            }
            break;
        }

        case CommandRxState::ReadCrcLow: {
            command_parser.received_crc_low = byte;
            command_parser.state =
                CommandRxState::ReadCrcHigh;
            break;
        }

        case CommandRxState::ReadCrcHigh: {
            const uint16_t received_crc =
                static_cast<uint16_t>(
                    command_parser.received_crc_low
                ) |
                static_cast<uint16_t>(
                    static_cast<uint16_t>(byte) << 8U
                );

            const uint16_t calculated_crc =
                calculateCRC16IBM(
                    command_parser.payload,
                    sizeof(command_parser.payload)
                );

            if (received_crc == calculated_crc) {
                processCompletedCommandFrame(received_crc);
                resetCommandParser();
            } else {
                // The current byte may be the first byte of the
                // next frame if the damaged frame lost a byte.
                resetCommandParserWithCandidate(byte);
            }

            break;
        }
    }
}

static void pollCommandUART() {
    size_t processed_bytes = 0U;

    while ((processed_bytes < MAX_COMMAND_BYTES_PER_LOOP) &&
           (Serial1.available() > 0)) {
        const int received = Serial1.read();

        if (received < 0) {
            break;
        }

        consumeCommandByte(
            static_cast<uint8_t>(received)
        );

        ++processed_bytes;
    }
}

// ============================================================================
// Safe-state management
// ============================================================================

static void clearCore0PIDState() {
    for (size_t wheel = 0U;
         wheel < WHEEL_COUNT;
         ++wheel) {
        pid_runtime[wheel] = PIDRuntimeState{};
        active_pid_gains[wheel] = PIDGainSnapshot{};
    }
}

static void enterSafeState() {
    if (!safe_state_active) {
        clearCore0PIDState();

        mutex_enter_blocking(&pid_exchange_mutex);

        for (size_t wheel = 0U;
             wheel < WHEEL_COUNT;
             ++wheel) {
            pid_exchange.inputs[wheel] =
                PIDInputSnapshot{};
            pid_exchange.gains[wheel] =
                PIDGainSnapshot{};
        }

        pid_exchange.safe_state = true;

        mutex_exit(&pid_exchange_mutex);

        safe_state_active = true;
    }

    target_left_rpm = 0.0f;
    target_right_rpm = 0.0f;

    // stop() writes zero PWM and de-asserts both enable pins.
    motor_left.stop();
    motor_right.stop();
}

static void leaveSafeState() {
    if (!safe_state_active) {
        return;
    }

    clearCore0PIDState();

    mutex_enter_blocking(&pid_exchange_mutex);

    for (size_t wheel = 0U;
         wheel < WHEEL_COUNT;
         ++wheel) {
        pid_exchange.inputs[wheel] =
            PIDInputSnapshot{};
        pid_exchange.gains[wheel] =
            PIDGainSnapshot{};
    }

    pid_exchange.safe_state = false;

    mutex_exit(&pid_exchange_mutex);

    safe_state_active = false;
}

static void latchI2CFault() {
    i2c_fault_latched = true;
    robot_state.system_status_code =
        SYSTEM_STATUS_I2C_FAULT;

    if (!BENCH_TEST_MODE_ENABLED) {
        enterSafeState();
    }
}

// ============================================================================
// Control-loop helpers
// ============================================================================

static PIDInputSnapshot updatePIDInput(
    size_t wheel,
    float target_rpm,
    float measured_rpm
) {
    PIDRuntimeState& runtime = pid_runtime[wheel];

    if (fabsf(target_rpm) <= ZERO_TARGET_EPSILON_RPM) {
        runtime = PIDRuntimeState{};
        return PIDInputSnapshot{};
    }

    float error = target_rpm - measured_rpm;

    if (fabsf(error) < 1.0f) {
        error = 0.0f;
    }

    static constexpr float CONTROL_DT_SECONDS =
        static_cast<float>(TASK_MOTOR_INTERVAL_MS) /
        1000.0f;

    const float raw_derivative =
        (error - runtime.last_error) /
        CONTROL_DT_SECONDS;

    runtime.filtered_derivative =
        (0.2f * raw_derivative) +
        (0.8f * runtime.filtered_derivative);

    if (error != 0.0f) {
        runtime.integral +=
            error * CONTROL_DT_SECONDS;
    }

    runtime.integral = constrain(
        runtime.integral,
        -40.0f,
        40.0f
    );

    runtime.last_error = error;

    return PIDInputSnapshot{
        error * 0.1f,
        runtime.filtered_derivative * 0.01f,
        runtime.integral * 0.1f
    };
}

static float calculateMotorSignal(
    size_t wheel,
    float target_rpm,
    const PIDInputSnapshot& normalized_input,
    const PIDGainSnapshot& gains
) {
    if (fabsf(target_rpm) <= ZERO_TARGET_EPSILON_RPM) {
        return 0.0f;
    }

    PIDRuntimeState& runtime = pid_runtime[wheel];

    runtime.smooth_kp =
        (0.1f * gains.kp) +
        (0.9f * runtime.smooth_kp);

    runtime.smooth_ki =
        (0.1f * gains.ki) +
        (0.9f * runtime.smooth_ki);

    runtime.smooth_kd =
        (0.1f * gains.kd) +
        (0.9f * runtime.smooth_kd);

    const float error =
        normalized_input.error * 10.0f;

    const float derivative_error =
        normalized_input.derivative_error * 100.0f;

    const float integral_error =
        normalized_input.integral_error * 10.0f;

    const float feedforward =
        target_rpm * 0.65f;

    const float pid_correction =
        (runtime.smooth_kp * error) +
        (runtime.smooth_ki * integral_error) +
        (runtime.smooth_kd * derivative_error);

    return constrain(
        feedforward + pid_correction,
        -100.0f,
        100.0f
    );
}

static void runMotorControlTick() {
    // Sensor acquisition and robot_state updates always run, including while
    // Safe State is active.
    const float filtered_left_rpm =
        rpm_filter_left.update(
            encoder_left.getRPM()
        );

    const float filtered_right_rpm =
        rpm_filter_right.update(
            encoder_right.getRPM()
        );

    robot_state.left_rpm = filtered_left_rpm;
    robot_state.right_rpm = filtered_right_rpm;

    robot_state.left_total_revs =
        encoder_left.getTotalRevolutions();

    robot_state.right_total_revs =
        encoder_right.getTotalRevolutions();

    robot_state.left_encoder_ticks =
        encoder_left.getContinuousTicks();

    robot_state.right_encoder_ticks =
        encoder_right.getContinuousTicks();

    // Continue collecting both power rails during Safe State. The INA226 is
    // on Wire/i2c0; the INA219 is on Wire1/i2c1. Each monitor validates its
    // connection and rejects non-finite readings before updating telemetry.
    PowerReading system_power{};
    PowerReading motors_power{};

    system_power_read_valid =
        system_power_monitor.read(system_power);

    motors_power_read_valid =
        motors_power_monitor.read(motors_power);

    if (!system_power_read_valid ||
        !motors_power_read_valid) {
        latchI2CFault();

        if (!BENCH_TEST_MODE_ENABLED) {
            return;
        }
    }

    if (system_power_read_valid) {
        robot_state.system_voltage_v =
            system_power.voltage_v;
        robot_state.system_current_a =
            system_power.current_a;
    }

    if (motors_power_read_valid) {
        robot_state.motors_voltage_v =
            motors_power.voltage_v;
        robot_state.motors_current_a =
            motors_power.current_a;
    }

    // Safe State suppresses only controller execution and motor actuation.
    // All sensor values above remain updated.
    if (safe_state_active) {
        motor_left.stop();
        motor_right.stop();
        return;
    }

    PIDInputSnapshot inputs[WHEEL_COUNT]{
        updatePIDInput(
            LEFT_WHEEL,
            target_left_rpm,
            filtered_left_rpm
        ),
        updatePIDInput(
            RIGHT_WHEEL,
            target_right_rpm,
            filtered_right_rpm
        )
    };

    PIDGainSnapshot gains_snapshot[WHEEL_COUNT]{};

    mutex_enter_blocking(&pid_exchange_mutex);

    for (size_t wheel = 0U;
         wheel < WHEEL_COUNT;
         ++wheel) {
        pid_exchange.inputs[wheel] =
            inputs[wheel];

        gains_snapshot[wheel] =
            pid_exchange.gains[wheel];
    }

    mutex_exit(&pid_exchange_mutex);

    for (size_t wheel = 0U;
         wheel < WHEEL_COUNT;
         ++wheel) {
        active_pid_gains[wheel] =
            gains_snapshot[wheel];
    }

    const float left_control_signal =
        calculateMotorSignal(
            LEFT_WHEEL,
            target_left_rpm,
            inputs[LEFT_WHEEL],
            gains_snapshot[LEFT_WHEEL]
        );

    const float right_control_signal =
        calculateMotorSignal(
            RIGHT_WHEEL,
            target_right_rpm,
            inputs[RIGHT_WHEEL],
            gains_snapshot[RIGHT_WHEEL]
        );

    if (!isfinite(left_control_signal) ||
        !isfinite(right_control_signal)) {
        enterSafeState();
        return;
    }

    motor_left.drive(left_control_signal);
    motor_right.drive(right_control_signal);
}

static void runIMUTick() {
    imu.update();

    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;

    float gx = 0.0f;
    float gy = 0.0f;
    float gz = 0.0f;

    float mx = 0.0f;
    float my = 0.0f;
    float mz = 0.0f;

    imu.getAccel(ax, ay, az);
    imu.getGyro(gx, gy, gz);
    imu.getMag(mx, my, mz);

    imu_filter.update(
        ax,
        ay,
        az,
        gx,
        gy,
        gz
    );

    robot_state.roll = imu_filter.getRoll();
    robot_state.pitch = imu_filter.getPitch();
    robot_state.yaw = imu_filter.getYaw();
    robot_state.heading_mag = mx;
}

static void runLidarTick() {
    servo_lidar.update();

    if (servo_lidar.isAtTarget()) {
        static bool sweep_direction = true;

        servo_lidar.setTargetAngle(
            sweep_direction ? 180.0f : 0.0f
        );

        sweep_direction = !sweep_direction;
    }

    robot_state.lidar_angle_deg =
        servo_lidar.getCurrentAngle();

    robot_state.lidar_is_valid =
        lidar_sensor.isDataValid() ? 1U : 0U;

    if (lidar_sensor.hasNewData()) {
        robot_state.lidar_distance_mm =
            static_cast<float>(
                lidar_sensor.getDistanceMM()
            );
    }
}

static uint32_t buildDiagnosticFlags(
    bool sensor_data_stale,
    bool command_data_stale
) {
    uint32_t flags = 0U;

    if (has_valid_serial_command) {
        flags |= DIAGNOSTIC_COMMAND_RECEIVED;
    }
    if (!command_data_stale) {
        flags |= DIAGNOSTIC_COMMAND_FRESH;
    }
    if (command_drive_enabled) {
        flags |= DIAGNOSTIC_DRIVE_ENABLED;
    }
    if (safe_state_active) {
        flags |= DIAGNOSTIC_SAFE_STATE_ACTIVE;
    }
    if (left_encoder_read_valid) {
        flags |= DIAGNOSTIC_LEFT_ENCODER_VALID;
    }
    if (right_encoder_read_valid) {
        flags |= DIAGNOSTIC_RIGHT_ENCODER_VALID;
    }
    if (sensor_data_stale) {
        flags |= DIAGNOSTIC_ENCODER_DATA_STALE;
    }
    if (i2c_fault_latched) {
        flags |= DIAGNOSTIC_I2C_FAULT_LATCHED;
    }
    if (system_power_read_valid) {
        flags |= DIAGNOSTIC_SYSTEM_POWER_VALID;
    }
    if (motors_power_read_valid) {
        flags |= DIAGNOSTIC_MOTORS_POWER_VALID;
    }
    if (robot_state.lidar_is_valid != 0U) {
        flags |= DIAGNOSTIC_LIDAR_VALID;
    }
    if (BENCH_TEST_MODE_ENABLED) {
        flags |= DIAGNOSTIC_BENCH_MODE_ACTIVE;
    }

    return flags;
}

static void runTelemetryTick(uint32_t now_ms) {
    robot_state.timestamp_ms = now_ms;

    telemetry_bridge.sendState(robot_state);
}

static void runDiagnosticTick() {
    Serial.print("Seq:");
    Serial.print(last_command_sequence);

    Serial.print(" | Safe:");
    Serial.print(safe_state_active ? 1 : 0);

    Serial.print(" | Target_L:");
    Serial.print(target_left_rpm, 1);

    Serial.print(" | RPM_L:");
    Serial.print(robot_state.left_rpm, 1);

    Serial.print(" | Kp_L:");
    Serial.print(active_pid_gains[LEFT_WHEEL].kp, 2);

    Serial.print(" | Target_R:");
    Serial.print(target_right_rpm, 1);

    Serial.print(" | RPM_R:");
    Serial.print(robot_state.right_rpm, 1);

    Serial.print(" | Kp_R:");
    Serial.println(
        active_pid_gains[RIGHT_WHEEL].kp,
        2
    );
}

// ============================================================================
// Core 0 initialization
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial1.setTX(PI_UART_TX_PIN);
    Serial1.setRX(PI_UART_RX_PIN);
    telemetry_bridge.begin(PI_UART_BAUDRATE);

    Wire.setSDA(I2C0_SDA_PIN);
    Wire.setSCL(I2C0_SCL_PIN);
    Wire.setClock(100000);
    Wire.setTimeout(10);
    Wire.begin();

    Wire1.setSDA(I2C1_SDA_PIN);
    Wire1.setSCL(I2C1_SCL_PIN);
    Wire1.setClock(100000);
    Wire1.setTimeout(10);
    Wire1.begin();

    motor_left.begin();
    motor_right.begin();

    const bool left_encoder_ready =
        encoder_left.begin();

    const bool right_encoder_ready =
        encoder_right.begin();

    servo_lidar.begin();

    Serial2.setTX(LASER_TX_PIN);
    Serial2.setRX(LASER_RX_PIN);
    lidar_sensor.begin();

    const bool system_power_ready =
        system_power_monitor.begin();

    const bool motors_power_ready =
        motors_power_monitor.begin();

    const bool imu_ready = imu.begin();

    i2c_fault_latched =
        !left_encoder_ready ||
        !right_encoder_ready ||
        !system_power_ready ||
        !motors_power_ready ||
        !imu_ready;

    left_encoder_read_valid = left_encoder_ready;
    right_encoder_read_valid = right_encoder_ready;
    system_power_read_valid = system_power_ready;
    motors_power_read_valid = motors_power_ready;

    const uint32_t now_ms = millis();

    if (left_encoder_ready &&
        right_encoder_ready) {
        last_valid_sensor_time = now_ms;
    }

    last_time_imu = now_ms;
    last_time_motor = now_ms;