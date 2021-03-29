#ifndef ADC_H
#define ADC_H

#include <Arduino.h>

enum ProximityPilotAmps
{
    Amp13,
    Amp20,
    Amp32,
    NoCable
};

enum PilotVoltage
{
    Volt_12,
    Volt_9,
    Volt_6,
    Volt_3,
    Fault
};

class AdcManagerClass
{

public:
    AdcManagerClass();

    void setup();

    bool updatePilotVoltageAndProximityPilotAmps();

    PilotVoltage currentPilotVoltage = Volt_12;
    ProximityPilotAmps currentProximityPilotAmps = Amp32;

    void changeInputPin(unsigned int analogPinName);
    struct ADCMeasurement read(unsigned int numberOgSamples, int avg, int shift);

private:
    const unsigned long pinProxymityPilot = PIN_A0;
    const unsigned long pinControlPilot = PIN_A1;

    unsigned long adcValueToMilliVolts(ADCMeasurement aDCMeasurement);

    ProximityPilotAmps toProximityPilot(ADCMeasurement aDCMeasurement);
    PilotVoltage toControlPilot(ADCMeasurement aDCMeasurement);
};

struct ADCMeasurement
{
    unsigned long lowest;
    unsigned long highest;
};

extern AdcManagerClass AdcManager;

#endif