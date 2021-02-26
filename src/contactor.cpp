#include <Arduino.h>
#include "contactor.h"
#include "logger.h"

Contactor::Contactor()
{
}

void Contactor::setup()
{
    pinMode(2, PinMode::OUTPUT);
    digitalWrite(2, LOW);
    Log.log("Contactor setup complete");
}

void Contactor::switchOff()
{
    digitalWrite(2, LOW);
}

void Contactor::switchOn()
{
    digitalWrite(2, HIGH);
}

bool Contactor::isOn()
{
    return digitalRead(2) == HIGH;
}