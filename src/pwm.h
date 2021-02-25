#ifndef PWM_H
#define PWM_H

#include <Arduino.h>

/*
 * PWM on pin D3
 */ 
class PwmD3
{
private:
    const uint32_t per = 375;

public:
    PwmD3();

    void updateDutyCycle(uint8_t pwmDutyCycle_percent);
    void setup(uint8_t initialPwmDutyCycle_percent);
};

#endif