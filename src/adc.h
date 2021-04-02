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

    unsigned long currentControlPilotAdc;
    PilotVoltage currentControlPilotVoltage = Volt_12;
    unsigned long currentProximityPilotAdc;
    ProximityPilotAmps currentProximityPilotAmps = Amp32;

    void changeInputPin(unsigned int analogPinName);
    struct ADCMeasurement read(unsigned int numberOgSamples, int avg, int shift);

private:
    const unsigned long pinProxymityPilot = PIN_A0;
    const unsigned long pinControlPilot = PIN_A1;

    unsigned long lastControlPilotAdc;
    unsigned long lastControlPilotChangedAt;
    unsigned long lastProximityPilotAdc;
    unsigned long lastProximityPilotChangedAt;

    unsigned long adcValueToMilliVolts(unsigned long aDCMeasurement);

    ProximityPilotAmps toProximityPilot(unsigned long aDCMeasurement);
    PilotVoltage toControlPilot(unsigned long aDCMeasurement);
};

struct ADCMeasurement
{
    unsigned long lowest;
    unsigned long highest;
};

extern AdcManagerClass AdcManager;

#endif