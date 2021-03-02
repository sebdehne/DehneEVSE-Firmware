#ifndef SENSOR_CURRENT_H
#define SENSOR_CURRENT_H

class SensorCurrentClass
{
private:
    const unsigned int numberOgSamples = 5 * 800; // try to catch at least 5 sine waves

public:
    SensorCurrentClass();

    unsigned long phase1Milliamps();
    unsigned long phase2Milliamps();
    unsigned long phase3Milliamps();
};

extern SensorCurrentClass SensorCurrent;

#endif