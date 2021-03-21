#include <Arduino.h>
#include "sensor_voltage.h"
#include "adc.h"

SensorVoltageClass SensorVoltage;

SensorVoltageClass::SensorVoltageClass()
{
}

unsigned long SensorVoltageClass::phase1Millivolts()
{
    AdcManager.changeInputPin(PIN_A5);
    ADCMeasurement reading = AdcManager.read(numberOgSamples);
    return reading.highest;
}
unsigned long SensorVoltageClass::phase2Millivolts()
{
    AdcManager.changeInputPin(PIN_A6);
    ADCMeasurement reading = AdcManager.read(numberOgSamples);
    return reading.highest;
}
unsigned long SensorVoltageClass::phase3Millivolts()
{
    AdcManager.changeInputPin(PIN_A7);
    ADCMeasurement reading = AdcManager.read(numberOgSamples);
    return reading.highest;
}
