#ifndef SENSOR_VOLTAGE_H
#define SENSOR_VOLTAGE_H

#include "adc.h"

class SensorVoltage
{
private:
public:
    SensorVoltage();

    uint32_t phase1Millivolts;
    uint32_t phase2Millivolts;
    uint32_t phase3Millivolts;

    void read(AdcManager adcManager);
};

#endif