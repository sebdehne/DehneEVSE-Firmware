#include <Arduino.h>
#include "contactor.h"
#include "logger.h"

ContactorClass Contactor;

ContactorClass::ContactorClass()
{
}

void ContactorClass::setup()
{
    pinMode(2, PinMode::OUTPUT);
    digitalWrite(2, LOW);
    Log.log("Contactor setup complete");
}

void ContactorClass::switchOff()
{
    digitalWrite(2, LOW);
}

void ContactorClass::switchOn()
{
    digitalWrite(2, HIGH);
}

bool ContactorClass::isOn()
{
    return digitalRead(2) == HIGH;
}
