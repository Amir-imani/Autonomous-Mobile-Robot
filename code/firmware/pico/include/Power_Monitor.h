#ifndef POWER_MONITOR_H
#define POWER_MONITOR_H

#include <Arduino.h>
#include <Wire.h>

#include <INA219.h>
#include <INA226.h>

struct PowerReading {
    float voltage_v;
    float current_a;
};

class Power_Monitor {
public:
    static constexpr float DEFAULT_SHUNT_OHMS = 0.00333f;
    static constexpr float DEFAULT_MAX_CURRENT_AMPS = 15.0f;

    Power_Monitor(
        INA226& sensor,
        float shunt_ohms = DEFAULT_SHUNT_OHMS,
        float max_current_amps = DEFAULT_MAX_CURRENT_AMPS
    );

    Power_Monitor(
        INA219& sensor,
        float shunt_ohms = DEFAULT_SHUNT_OHMS,
        float max_current_amps = DEFAULT_MAX_CURRENT_AMPS
    );

    bool begin();
    bool read(PowerReading& reading);
    bool isConnected();
    bool isReady() const;

    uint16_t getCalibrationRegister() const;
    float getCurrentLSB() const;

private:
    enum class SensorType : uint8_t {
        INA226_SYSTEM = 0U,
        INA219_MOTORS = 1U
    };

    static constexpr uint8_t CALIBRATION_REGISTER = 0x05U;

    SensorType sensor_type;
    INA226* ina226_sensor;
    INA219* ina219_sensor;
    float shunt_ohms;
    float max_current_amps;
    uint16_t calibration_register;
    bool ready;

    uint16_t calculateExpectedCalibrationRegister() const;
    bool configureINA226();
    bool configureINA219();
};

#endif
