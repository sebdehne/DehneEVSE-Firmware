#include "charger.h"

Charger::Charger(bool isOutDoorInput, uint8_t defaultChargeCurrentAmps)
{
    isOutDoor = isOutDoorInput;
    chargeCurrentAmps = defaultChargeCurrentAmps;
}

void Charger::setup(PwmD3 pwmD3, Contactor contactor, Logger *loggerIn)
{
    logger = loggerIn;
    pwmD3.updateDutyCycle(100);
    contactor.switchOff();
    Serial.println("Charger setup complete");
}

PilotVoltage Charger::readCPValue(AdcManager adcManager)
{
    adcManager.changeInputPin(pinControlPilot);
    ADCMeasurement cpADCMeasurement = adcManager.read(200);
    return adcManager.toControlPilot(cpADCMeasurement);
}

ProximityPilotAmps Charger::readPPValue(AdcManager adcManager)
{
    adcManager.changeInputPin(pinProxymityPilot);
    ADCMeasurement ppADCMeasurement = adcManager.read(200);
    return adcManager.toProximityPilot(ppADCMeasurement);
}

ChargerData Charger::getData()
{
    ChargerData data;
    data.chargeCurrentAmps = chargeCurrentAmps;
    data.chargerState = chargerState;
    data.pilotVoltage = pilotVoltage;
    data.proximityPilotAmps = proximityPilotAmps;
    return data;
}

void Charger::tick(PwmD3 pwmD3, AdcManager adcManager, Contactor contactor)
{
    // 1. read current CP value
    pilotVoltage = readCPValue(adcManager);

    // 2. read current PP value
    proximityPilotAmps = readPPValue(adcManager);

    // 3. run state logic
    if (chargerState == A_Unconnected)
    {
        handleState_A(pwmD3, contactor);
    }
    else if (chargerState == B_Connected)
    {
        handleState_B(pwmD3, contactor);
    }
    else if (chargerState == C_Charging || chargerState == D_Ventilation)
    {
        handleState_C_OR_D(pwmD3, contactor);
    }
    else if (chargerState == E_ShutOff)
    {
        handleState_E(pwmD3, contactor);
    }
    else
    {
        handleState_F(pwmD3, contactor);
    }
}

void Charger::handleState_A(PwmD3 pwmD3, Contactor contactor)
{
    if (pilotVoltage == Volt_12)
    {
        // still unconnected
        pwmD3.updateDutyCycle(100);
        contactor.switchOff();
        chargerState = A_Unconnected;
    }
    else
    {
        // got connected
        pwmD3.updateDutyCycle(100);
        contactor.switchOff();
        chargerState = B_Connected;
        Serial.println("Charging cable connected");
        logger->log("Charging cable connected");
    }
}

void Charger::handleState_B(PwmD3 pwmD3, Contactor contactor)
{

    if (pilotVoltage == Volt_12)
    {
        // got disconnected
        pwmD3.updateDutyCycle(100);
        contactor.switchOff();
        chargerState = A_Unconnected;
        Serial.println("Charging cable disconnected");
        logger->log("Charging cable disconnected");
    }
    else if (pilotVoltage == Volt_9)
    {
        // announce max charging rate
        pwmD3.updateDutyCycle(calcPwmPercent());
        contactor.switchOff();
        chargerState = B_Connected;
    }
    else if (pilotVoltage == Volt_6)
    {
        // announce max charging rate
        pwmD3.updateDutyCycle(calcPwmPercent());
        // car is ready
        contactor.switchOff();
        chargerState = C_Charging;
        Serial.println("Car ready to charge");
        logger->log("Car ready to charge");
    }
    else if (pilotVoltage == Volt_3)
    {
        // announce max charging rate
        pwmD3.updateDutyCycle(calcPwmPercent());
        // car is ready - with ventilation
        contactor.switchOff();
        chargerState = D_Ventilation;
        Serial.println("Car ready to charge ventilation");
        logger->log("Car ready to charge ventilation");
    }
    else
    {
        // go to ERROR until disconnected
        pwmD3.updateDutyCycle(100);
        contactor.switchOff();
        chargerState = F_Error;
        Serial.println("Charging error");
        logger->log("Charging error");
    }
}

void Charger::handleState_C_OR_D(PwmD3 pwmD3, Contactor contactor)
{
    /*
     * TODO When calcPwmPercent is below 9% -> give the car 5 seconds and then force status A
     */
    if (pilotVoltage == Volt_12)
    {
        pwmD3.updateDutyCycle(100);
        contactor.switchOff();
        chargerState = A_Unconnected;
        Serial.println("Disconnected while charging");
        logger->log("Disconnected while charging");
    }
    else if (pilotVoltage == Volt_9)
    {
        // announce max charging rate
        pwmD3.updateDutyCycle(calcPwmPercent());
        // done charging
        contactor.switchOff();
        chargerState = B_Connected;
        Serial.println("charging stopped");
        logger->log("charging stopped");
    }
    else if (pilotVoltage == Volt_6)
    {
        // announce max charging rate
        pwmD3.updateDutyCycle(calcPwmPercent());
        // continue charging
        contactor.switchOn();
        chargerState = C_Charging;
    }
    else if (pilotVoltage == Volt_3)
    {
        // announce max charging rate
        if (isOutDoor)
        {
            pwmD3.updateDutyCycle(calcPwmPercent());
            contactor.switchOn();
        }
        else
        {
            pwmD3.updateDutyCycle(100); // cannot charge inside
            contactor.switchOff();
        }

        chargerState = D_Ventilation;
    }
    else
    {
        // go to ERROR until disconnected
        pwmD3.updateDutyCycle(100);
        contactor.switchOff();
        chargerState = F_Error;
        Serial.println("charging stopped");
        logger->log("charging stopped");
    }
}

void Charger::handleState_E(PwmD3 pwmD3, Contactor contactor)
{
    pwmD3.updateDutyCycle(100);
    contactor.switchOff();
    chargerState = A_Unconnected;
}
void Charger::handleState_F(PwmD3 pwmD3, Contactor contactor)
{
    if (pilotVoltage == Volt_12)
    {
        // got disconnected -> back to A
        pwmD3.updateDutyCycle(100);
        contactor.switchOff();
        chargerState = A_Unconnected;
        Serial.println("Leaving error");
        logger->log("Leaving error");
    }
    else
    {
        // stay here
        pwmD3.updateDutyCycle(100);
        contactor.switchOff();
        chargerState = F_Error;
    }
}

uint8_t Charger::calcPwmPercent()
{
    uint32_t amps = chargeCurrentAmps;
    if (proximityPilotAmps == Amp13 && amps > 13)
    {
        amps = 13;
    }
    else if (proximityPilotAmps == Amp20 && amps > 20)
    {
        amps = 20;
    }
    else if (amps > 32)
    {
        amps = 32;
    }

    /*
     * 10% =>  6A
     * 15% => 10A
     * 25% => 16A
     * 40% => 25A
     * 50% => 32A
     * 
     * PWM_percent = (1540X + 1500) / 1000
     */
    return ((1540 * amps) + 1500) / 1000;
}

void Charger::setChargeCurrentAmps(uint8_t chargeCurrentAmpsInput)
{
    chargeCurrentAmps = chargeCurrentAmpsInput;
}
