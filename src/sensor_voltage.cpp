#include "sensor_voltage.h"

SensorVoltage::SensorVoltage()
{
}

void SensorVoltage::read(AdcManager adcManager)
{
    // pins A5 - A7
    const uint16_t numberOgSamples = 600; // try to catch at least 2 sine waves

    adcManager.changeInputPin(PIN_A5);
    ADCMeasurement reading = adcManager.read(numberOgSamples);
    phase1Millivolts = adcManager.toMilliVolts(reading);

    adcManager.changeInputPin(PIN_A6);
    reading = adcManager.read(numberOgSamples);
    phase2Millivolts = adcManager.toMilliVolts(reading);

    adcManager.changeInputPin(PIN_A7);
    reading = adcManager.read(numberOgSamples);
    phase3Millivolts = adcManager.toMilliVolts(reading);
}