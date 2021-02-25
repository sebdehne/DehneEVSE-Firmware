
#include "pwm.h"
#include "adc.h"
#include "contactor.h"
#include "config.h"

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

struct ChargerData
{
    ChargerState chargerState;
    bool isOutDoor;
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

    void handleState_A(PilotVoltage pilotVoltage, PwmD3 pwmD3, ProximityPilotAmps proximityPilotAmps, Contactor contactor);
    void handleState_B(PilotVoltage pilotVoltage, PwmD3 pwmD3, ProximityPilotAmps proximityPilotAmps, Contactor contactor);
    void handleState_C_OR_D(PilotVoltage pilotVoltage, PwmD3 pwmD3, ProximityPilotAmps proximityPilotAmps, Contactor contactor);
    void handleState_E(PilotVoltage pilotVoltage, PwmD3 pwmD3, ProximityPilotAmps proximityPilotAmps, Contactor contactor);
    void handleState_F(PilotVoltage pilotVoltage, PwmD3 pwmD3, ProximityPilotAmps proximityPilotAmps, Contactor contactor);
    uint8_t calcPwmPercent(ProximityPilotAmps proximityPilotAmps);
    PilotVoltage readCPValue(AdcManager adcManager);
    ProximityPilotAmps readPPValue(AdcManager adcManager);

public:
    Charger(bool isOutDoor, uint8_t defaultChargeCurrentAmps);

    void setup(PwmD3 pwmD3, Contactor contactor);

    void tick(PwmD3 pwmD3, AdcManager adcManager, Contactor contactor);
    void setChargeCurrentAmps(uint8_t chargeCurrentAmps);

    ChargerData getData();
};


#endif
