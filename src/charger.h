
#include "pwm.h"
#include "adc.h"

#ifndef CHARGER_H
#define CHARGER_H

enum ChargerState
{
    A_Unconnected,
    B_Connected,
    C_Charging,
    D_Ventilation,
    E_ShutOff,
    F_Error
};

class Charger
{
private:
    boolean isOutDoor;
    ChargerState chargerState = A_Unconnected;
    uint8_t chargeCurrentAmps = 0;
    uint8_t pwmPercent();
    const uint8_t pinProxymityPilot = PIN_A0;
    const uint8_t pinControlPilot = PIN_A1;

    void handleState(PilotVoltage pilotVoltage, PwmD3 pwmD3, ProximityPilotAmps proximityPilotAmps);
    uint8_t calcPwmPercent(ProximityPilotAmps proximityPilotAmps);

public:
    Charger(boolean isOutDoor);

    void setup(PwmD3 pwmD3);

    void tick(PwmD3 pwmD3, AdcManager adcManager);
    void setChargeCurrentAmps(uint8_t chargeCurrentAmps);
};

#endif
