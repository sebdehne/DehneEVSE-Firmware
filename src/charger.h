
#ifndef CHARGER_H
#define CHARGER_H

#include "pwm.h"
#include "adc.h"
#include "contactor.h"
#include "config.h"
#include "logger.h"

enum ChargerState
{
    A_Unconnected,
    B_Connected,
    C_Charging,
    D_Ventilation,
    E_ShutOff,
    F_Error
};

struct ChargerData
{
    ChargerState chargerState;
    uint8_t chargeCurrentAmps;
    PilotVoltage pilotVoltage;
    ProximityPilotAmps proximityPilotAmps;
};

class Charger
{
private:
    const uint8_t pinProxymityPilot = PIN_A0;
    const uint8_t pinControlPilot = PIN_A1;

    bool isOutDoor;
    ChargerState chargerState = A_Unconnected;
    uint8_t chargeCurrentAmps;
    PilotVoltage pilotVoltage;
    ProximityPilotAmps proximityPilotAmps;

    void handleState_A(PwmD3 pwmD3, Contactor contactor);
    void handleState_B(PwmD3 pwmD3, Contactor contactor);
    void handleState_C_OR_D(PwmD3 pwmD3, Contactor contactor);
    void handleState_F(PwmD3 pwmD3, Contactor contactor);
    uint8_t calcPwmPercent();
    PilotVoltage readCPValue(AdcManager adcManager);
    ProximityPilotAmps readPPValue(AdcManager adcManager);

    uint32_t chargeRateTooLowStarted;

public:
    Charger(bool isOutDoor, uint8_t defaultChargeCurrentAmps);

    void setup(PwmD3 pwmD3, Contactor contactor);

    void tick(PwmD3 pwmD3, AdcManager adcManager, Contactor contactor);
    void setChargeCurrentAmps(uint8_t chargeCurrentAmps);

    ChargerData getData();
};

#endif
