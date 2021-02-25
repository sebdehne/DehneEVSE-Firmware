#ifndef ADC_H
#define ADC_H

#include <Arduino.h>
#include "types.h"

class AdcManager
{

public:
    AdcManager();

    void setup();
    void changeInputPin(uint8_t analogPinName);
    struct ADCMeasurement read(uint16_t numberOgSamples);

    uint32_t toMainsMilliAmpsRms(ADCMeasurement aDCMeasurement);
    uint32_t toMainsMilliVoltsRms(ADCMeasurement aDCMeasurement);
    ProximityPilotAmps toProximityPilot(ADCMeasurement aDCMeasurement);
    PilotVoltage toControlPilot(ADCMeasurement aDCMeasurement);

private:
    uint16_t adcValueToMilliVolts(ADCMeasurement aDCMeasurement);
};

struct ADCMeasurement
{
    uint16_t lowest;
    uint16_t highest;
};

#endif