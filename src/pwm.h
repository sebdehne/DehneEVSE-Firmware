#ifndef PWM_H
#define PWM_H

#include <Arduino.h>

/*
 * PWM on pin D3
 */
class PwmD3Class
{
private:
    const unsigned int per = 375;

    unsigned int currentPwmDutyCycle_percent = 100;
public:
    PwmD3Class();

    void setup();
    void updateDutyCycle(unsigned char pwmDutyCycle_percent);
    unsigned int getCurrentPwmDutyCycle_percent();
};

extern PwmD3Class PwmD3;

#endif