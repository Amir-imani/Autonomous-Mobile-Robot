#include "Motor_BTS7960.h"

#include <math.h>

Motor_BTS7960::Motor_BTS7960(uint8_t r_en,uint8_t l_en,uint8_t r_pwm,uint8_t l_pwm,bool invert)
    : pin_R_EN(r_en), pin_L_EN(l_en), pin_R_PWM(r_pwm), pin_L_PWM(l_pwm), max_pwm_limit(MAX_PWM_LIMIT), is_inverted(invert) {}

void Motor_BTS7960::begin() {
    pinMode(pin_R_EN, OUTPUT); pinMode(pin_L_EN, OUTPUT); pinMode(pin_R_PWM, OUTPUT); pinMode(pin_L_PWM, OUTPUT); stop();
}

void Motor_BTS7960::drive(float power_percentage) {
    if (!isfinite(power_percentage)) { stop(); return; }
    if (is_inverted) power_percentage = -power_percentage;
    power_percentage = constrain(power_percentage,-100.0f,100.0f);
    if ((power_percentage == 0.0f) || (max_pwm_limit <= MIN_PWM_LIMIT)) { stop(); return; }
    const float pwm_magnitude=(fabsf(power_percentage)/100.0f)*static_cast<float>(max_pwm_limit);
    const int pwm_value=constrain(static_cast<int>(pwm_magnitude),MIN_PWM_LIMIT,max_pwm_limit);
    if (pwm_value <= MIN_PWM_LIMIT) { stop(); return; }
    analogWrite(pin_R_PWM,0); analogWrite(pin_L_PWM,0);
    digitalWrite(pin_R_EN,HIGH); digitalWrite(pin_L_EN,HIGH);
    if (power_percentage > 0.0f) analogWrite(pin_R_PWM,pwm_value); else analogWrite(pin_L_PWM,pwm_value);
}
void Motor_BTS7960::stop() { analogWrite(pin_R_PWM,0); analogWrite(pin_L_PWM,0); digitalWrite(pin_R_EN,LOW); digitalWrite(pin_L_EN,LOW); }
void Motor_BTS7960::setSafetyLimit(int limit) { max_pwm_limit=constrain(limit,MIN_PWM_LIMIT,MAX_PWM_LIMIT); if (max_pwm_limit == MIN_PWM_LIMIT) stop(); }
