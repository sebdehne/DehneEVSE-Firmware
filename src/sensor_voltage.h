#ifndef SENSOR_VOLTAGE_H
#define SENSOR_VOLTAGE_H


class SensorVoltageClass
{
private:
    const unsigned int numberOgSamples = 5 * 800; // try to catch at least 5 sine waves

public:
    SensorVoltageClass();

    unsigned long phase1Millivolts();
    unsigned long phase2Millivolts();
    unsigned long phase3Millivolts();
    
};

extern SensorVoltageClass SensorVoltage;

#endif