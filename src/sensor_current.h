#include "adc.h"

#ifndef SENSOR_CURRENT_H
#define SENSOR_CURRENT_H

class SensorCurrent
{
private:
public:
    SensorCurrent();

    uint32_t phase1Milliamps;
    uint32_t phase2Milliamps;
    uint32_t phase3Milliamps;

    void read(AdcManager adcManager);
};

#endif