#ifndef CONFIG_H
#define CONFIG_H

// ==========================================
// Motor Driver BTS7960
// ==========================================
// Left
#define L_MOTOR_EN_R  2
#define L_MOTOR_EN_L  3
#define L_MOTOR_PWM_R 4
#define L_MOTOR_PWM_L 5
#define L_MOTOR_INVERT true

// Right
#define R_MOTOR_EN_R  12
#define R_MOTOR_EN_L  13
#define R_MOTOR_PWM_R 14
#define R_MOTOR_PWM_L 15
#define R_MOTOR_INVERT false


#define GEAR_RATIO 110.0

// ==========================================
// 2. Servo
// ==========================================

#define PIN_SERVO_LIDAR 10
#define PIN_SERVO_CAM   11
#define SERVO_MIN_PULSE 544
#define SERVO_MAX_PULSE 2400
#define LIDAR_SERVO_SPEED_DEG_PER_SEC 85.0f
#define SERVO_CONTROL_INTERVAL_MS    20U

// ==========================================
// 3. I2C & UART
// ==========================================
// I2C0 :
#define I2C0_SDA_PIN 0
#define I2C0_SCL_PIN 1

// I2C1 :
#define I2C1_SDA_PIN 26
#define I2C1_SCL_PIN 27

// UART : 
#define LASER_TX_PIN 8
#define LASER_RX_PIN 9
#define LASER_BAUDRATE 9600

// ==========================================
// RTOS Scheduler
// ==========================================

#define TASK_IMU_INTERVAL_MS    10  // 100 Hz
#define TASK_MOTOR_INTERVAL_MS  20  // 50 Hz
#define TASK_LIDAR_INTERVAL_MS  20  // 50 Hz servo command update
#define TASK_TELEMETRY_INTERVAL_MS 20  // 50 Hz binary telemetry
#define TASK_DIAGNOSTIC_INTERVAL_MS 100 // 10 Hz USB diagnostics

// ==========================================
// Bench safety profile
// ==========================================
// Set this to 0 before operating the robot with drive wheels on the ground.
// Bench mode preserves the hardware watchdog and command stop behavior, but
// prevents encoder or power-monitor I2C faults from disabling suspended wheels.
#define BENCH_TEST_MODE 1

#if BENCH_TEST_MODE
#define COMMAND_STALE_TIMEOUT_MS 500U
#define SENSOR_STALE_TIMEOUT_MS  150U
#else
#define COMMAND_STALE_TIMEOUT_MS 100U
#define SENSOR_STALE_TIMEOUT_MS  100U
#endif

#endif
