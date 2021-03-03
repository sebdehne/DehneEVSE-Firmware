#include "LED.h"
#include <Arduino.h>

LEDClass::LEDClass() {}

void LEDClass::setup()
{
    // initialize LED digital pin as an output.
    pinMode(LED_BUILTIN, OUTPUT);
}

void LEDClass::blink(int times, int delayMS)
{
    while (times-- > 0)
    {
        digitalWrite(LED_BUILTIN, 1);
        delay(delayMS);
        digitalWrite(LED_BUILTIN, 0);
        delay(delayMS);
    }
}

LEDClass LED;
