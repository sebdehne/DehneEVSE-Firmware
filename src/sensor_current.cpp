#include "sensor_current.h"

SensorCurrent::SensorCurrent()
{
}

void SensorCurrent::read(AdcManager adcManager)
{
    // pins A2 - A4
    const uint16_t numberOgSamples = 600; // try to catch at least 2 sine waves

    adcManager.changeInputPin(PIN_A2);
    ADCMeasurement reading = adcManager.read(numberOgSamples);
    phase1Milliamps = adcManager.toMilliAmps(reading);

    adcManager.changeInputPin(PIN_A3);
    reading = adcManager.read(numberOgSamples);
    phase2Milliamps = adcManager.toMilliAmps(reading);

    adcManager.changeInputPin(PIN_A4);
    reading = adcManager.read(numberOgSamples);
    phase3Milliamps = adcManager.toMilliAmps(reading);
}