#include "Power_Monitor.h"

#include <math.h>

namespace {

constexpr float CURRENT_REGISTER_COUNTS = 32768.0f;
constexpr float INA226_CALIBRATION_CONSTANT = 0.00512f;
constexpr float INA219_CALIBRATION_CONSTANT = 0.04096f;

}  // namespace

Power_Monitor::Power_Monitor(
    INA226& sensor,
    float configured_shunt_ohms,
    float configured_max_current_amps
) :
    sensor_type(SensorType::INA226_SYSTEM),
    ina226_sensor(&sensor),
    ina219_sensor(nullptr),
    shunt_ohms(configured_shunt_ohms),
    max_current_amps(configured_max_current_amps),
    calibration_register(0U),
    ready(false) {
}

Power_Monitor::Power_Monitor(
    INA219& sensor,
    float configured_shunt_ohms,
    float configured_max_current_amps
) :
    sensor_type(SensorType::INA219_MOTORS),
    ina226_sensor(nullptr),
    ina219_sensor(&sensor),
    shunt_ohms(configured_shunt_ohms),
    max_current_amps(configured_max_current_amps),
    calibration_register(0U),
    ready(false) {
}

bool Power_Monitor::begin() {
    ready = false;
    calibration_register = 0U;

    if (!isfinite(shunt_ohms) ||
        !isfinite(max_current_amps) ||
        shunt_ohms <= 0.0f ||
        max_current_amps <= 0.0f) {
        return false;
    }

    const uint16_t expected_calibration = calculateExpectedCalibrationRegister();
    if (expected_calibration == 0U) return false;

    bool configured = false;
    if (sensor_type == SensorType::INA226_SYSTEM) configured = configureINA226();
    else configured = configureINA219();

    if (!configured || !isConnected()) return false;

    calibration_register = getCalibrationRegister();
    ready = calibration_register == expected_calibration;
    return ready;
}

bool Power_Monitor::read(PowerReading& reading) {
    if (!ready || !isConnected()) return false;
    float voltage_v = 0.0f;
    float current_a = 0.0f;
    if (sensor_type == SensorType::INA226_SYSTEM) {
        if (ina226_sensor == nullptr) return false;
        voltage_v = ina226_sensor->getBusVoltage();
        current_a = ina226_sensor->getCurrent();
    } else {
        if (ina219_sensor == nullptr) return false;
        voltage_v = ina219_sensor->getBusVoltage();
        current_a = ina219_sensor->getCurrent();
    }
    if (!isfinite(voltage_v) || !isfinite(current_a)) return false;
    reading.voltage_v = voltage_v;
    reading.current_a = current_a;
    return true;
}

bool Power_Monitor::isConnected() {
    if (sensor_type == SensorType::INA226_SYSTEM) return ina226_sensor != nullptr && ina226_sensor->isConnected();
    return ina219_sensor != nullptr && ina219_sensor->isConnected();
}

bool Power_Monitor::isReady() const { return ready; }

uint16_t Power_Monitor::getCalibrationRegister() const {
    if (sensor_type == SensorType::INA226_SYSTEM) {
        if (ina226_sensor == nullptr) return 0U;
        return ina226_sensor->getRegister(CALIBRATION_REGISTER);
    }
    if (ina219_sensor == nullptr) return 0U;
    return ina219_sensor->getRegister(CALIBRATION_REGISTER);
}

float Power_Monitor::getCurrentLSB() const {
    if (sensor_type == SensorType::INA226_SYSTEM) {
        if (ina226_sensor == nullptr) return 0.0f;
        return ina226_sensor->getCurrentLSB();
    }
    if (ina219_sensor == nullptr) return 0.0f;
    return ina219_sensor->getCurrentLSB();
}

uint16_t Power_Monitor::calculateExpectedCalibrationRegister() const {
    const float current_lsb = max_current_amps / CURRENT_REGISTER_COUNTS;
    const float calibration_constant = sensor_type == SensorType::INA226_SYSTEM ? INA226_CALIBRATION_CONSTANT : INA219_CALIBRATION_CONSTANT;
    const float raw_calibration = calibration_constant / (current_lsb * shunt_ohms);
    if (!isfinite(raw_calibration) || raw_calibration < 1.0f || raw_calibration > 65535.0f) return 0U;
    return static_cast<uint16_t>(lroundf(raw_calibration));
}

bool Power_Monitor::configureINA226() {
    if (ina226_sensor == nullptr || !ina226_sensor->begin()) return false;
    const bool sampling_configured = ina226_sensor->setAverage(INA226_4_SAMPLES) && ina226_sensor->setBusVoltageConversionTime(INA226_1100_us) && ina226_sensor->setShuntVoltageConversionTime(INA226_1100_us) && ina226_sensor->setModeShuntBusContinuous();
    const int calibration_status = ina226_sensor->setMaxCurrentShunt(max_current_amps,shunt_ohms,false);
    return sampling_configured && calibration_status == INA226_ERR_NONE && ina226_sensor->isCalibrated() && ina226_sensor->getAverage() == INA226_4_SAMPLES && ina226_sensor->getBusVoltageConversionTime() == INA226_1100_us && ina226_sensor->getShuntVoltageConversionTime() == INA226_1100_us;
}

bool Power_Monitor::configureINA219() {
    if (ina219_sensor == nullptr || !ina219_sensor->begin()) return false;
    const bool sampling_configured = ina219_sensor->setBusVoltageRange(32U) && ina219_sensor->setGain(2U) && ina219_sensor->setBusADC(0x03U) && ina219_sensor->setShuntADC(0x03U) && ina219_sensor->setModeShuntBusContinuous();
    const bool calibration_configured = ina219_sensor->setMaxCurrentShunt(max_current_amps,shunt_ohms);
    return sampling_configured && calibration_configured && ina219_sensor->isCalibrated() && ina219_sensor->getBusVoltageRange() == 32U && ina219_sensor->getGain() == 2U && ina219_sensor->getBusADC() == 0x03U && ina219_sensor->getShuntADC() == 0x03U;
}
