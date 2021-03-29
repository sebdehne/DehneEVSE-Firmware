#include <Arduino.h>
#include "sensor_current.h"
#include "adc.h"

SensorCurrentClass SensorCurrent;

SensorCurrentClass::SensorCurrentClass()
{
}

unsigned long SensorCurrentClass::phase1Milliamps()
{
    AdcManager.changeInputPin(PIN_A2);
    ADCMeasurement reading = AdcManager.read(numberOgSamples, 8, 3);
    return reading.highest;
}

unsigned long SensorCurrentClass::phase2Milliamps()
{
    AdcManager.changeInputPin(PIN_A3);
    ADCMeasurement reading = AdcManager.read(numberOgSamples, 8, 3);
    return reading.highest;
}

unsigned long SensorCurrentClass::phase3Milliamps()
{
    AdcManager.changeInputPin(PIN_A4);
    ADCMeasurement reading = AdcManager.read(numberOgSamples, 8, 3);
    return reading.highest;
}
