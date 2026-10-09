#include "Servo_Manager.h"

Servo_Manager::Servo_Manager(uint8_t servo_pin,float max_speed_deg_per_sec,uint16_t min_p,uint16_t max_p,uint16_t control_interval_ms) : pin(servo_pin), current_angle(90.0f), target_angle(90.0f), max_speed_degrees_per_second(max_speed_deg_per_sec), min_pulse(min_p), max_pulse(max_p), last_update_time_ms(0U), update_interval_ms(control_interval_ms) {}

void Servo_Manager::begin() { internal_servo.attach(pin,min_pulse,max_pulse); internal_servo.write(current_angle); last_update_time_ms=millis(); }
void Servo_Manager::setTargetAngle(float angle) { if (angle<0.0f) angle=0.0f; else if (angle>180.0f) angle=180.0f; target_angle=angle; }
void Servo_Manager::update() {
    const uint32_t now_ms=millis();
    const uint32_t elapsed_ms=static_cast<uint32_t>(now_ms-last_update_time_ms);
    if (elapsed_ms<update_interval_ms) return;
    last_update_time_ms=now_ms;
    const float remaining_degrees=target_angle-current_angle;
    if (fabsf(remaining_degrees)<=0.1f) { current_angle=target_angle; return; }
    const uint32_t maximum_elapsed_ms=static_cast<uint32_t>(update_interval_ms)*2U;
    const uint32_t bounded_elapsed_ms=elapsed_ms>maximum_elapsed_ms?maximum_elapsed_ms:elapsed_ms;
    const float maximum_step_degrees=max_speed_degrees_per_second*static_cast<float>(bounded_elapsed_ms)/1000.0f;
    if (fabsf(remaining_degrees)<=maximum_step_degrees) current_angle=target_angle;
    else if (remaining_degrees>0.0f) current_angle+=maximum_step_degrees;
    else current_angle-=maximum_step_degrees;
    const float pulse_span=static_cast<float>(max_pulse-min_pulse);
    const uint16_t pulse=static_cast<uint16_t>(static_cast<float>(min_pulse)+((current_angle/180.0f)*pulse_span)+0.5f);
    internal_servo.writeMicroseconds(pulse);
}
float Servo_Manager::getCurrentAngle(){return current_angle;}
bool Servo_Manager::isAtTarget(){return fabsf(target_angle-current_angle)<0.5f;}
