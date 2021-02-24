#include "charger.h"

Charger::Charger(boolean isOutDoorInput)
{
    isOutDoor = isOutDoorInput;
}

void Charger::setup(PwmD3 pwmD3)
{
    pwmD3.updateDutyCycle(100);
}

void Charger::tick(PwmD3 pwmD3, AdcManager adcManager)
{
    /*
     * 1. read current CP value
     * 2. read current PP value
     * 3. adjust status if needed
     */

    // 1. read current CP value
    adcManager.changeInputPin(pinControlPilot);
    ADCMeasurement cpADCMeasurement = adcManager.read(200);
    PilotVoltage pilotVoltage = adcManager.toControlPilot(cpADCMeasurement);

    // 2. read current PP value
    adcManager.changeInputPin(pinProxymityPilot);
    cpADCMeasurement = adcManager.read(200);
    ProximityPilotAmps proximityPilotAmps = adcManager.toProximityPilot(cpADCMeasurement);

    handleState(pilotVoltage, pwmD3, proximityPilotAmps);

    // TODO:
    // - Relay control
    // - A_Unconnected -> Volt_6, Volt_3, Volt_0, Volt_12Neg??

    // 3. calcCurrentAmp(pp, chargeCurrentAmpsInput, chargerState)
}

void Charger::handleState(PilotVoltage pilotVoltage, PwmD3 pwmD3, ProximityPilotAmps proximityPilotAmps)
{
    if (chargerState == A_Unconnected)
    {
        if (pilotVoltage == Volt_12)
        {
            // still unconnected
            pwmD3.updateDutyCycle(100);
            // TODO close contactor
            chargerState = A_Unconnected;
        }
        else if (pilotVoltage == Volt_9)
        {
            // got connected
            pwmD3.updateDutyCycle(100);
            // TODO close contactor
            delay(250);
            chargerState = B_Connected;
        }
        else
        {
            // error
            pwmD3.updateDutyCycle(0); // = 12Neg?
            // TODO close contactor
            chargerState = F_Error;
        }
    }
    else if (chargerState == B_Connected)
    {
        if (pilotVoltage == Volt_12)
        {
            // got disconnected
            pwmD3.updateDutyCycle(100);
            // TODO close contactor
            chargerState = A_Unconnected;
        } else if (pilotVoltage == Volt_9) {
            // TODO close contactor
            pwmD3.updateDutyCycle(calcPwmPercent(proximityPilotAmps));
            chargerState = B_Connected;
        }
    } 
    // TODO
}

uint8_t Charger::calcPwmPercent(ProximityPilotAmps proximityPilotAmps) {
    // TODO
    return 50;
}

void Charger::setChargeCurrentAmps(uint8_t chargeCurrentAmpsInput)
{
    chargeCurrentAmps = chargeCurrentAmpsInput;
}
