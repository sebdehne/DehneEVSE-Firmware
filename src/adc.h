#include <Arduino.h>
#include "types.h"

#ifndef ADC_H
#define ADC_H

class AdcManager
{

public:
    AdcManager();

    void setup();
    void changeInputPin(uint8_t analogPinName);
    struct ADCMeasurement read(uint16_t numberOgSamples);

    uint32_t toMilliAmps(ADCMeasurement aDCMeasurement);
    uint32_t toMilliVolts(ADCMeasurement aDCMeasurement);
    ProximityPilotAmps toProximityPilot(ADCMeasurement aDCMeasurement);
    PilotVoltage toControlPilot(ADCMeasurement aDCMeasurement);

private:
};

struct ADCMeasurement
{
    uint16_t lowest;
    uint16_t highest;
};

#endif