#include <Arduino.h>
#include "adc.h"
#include "logger.h"

AdcManagerClass AdcManager;

AdcManagerClass::AdcManagerClass()
{
}

void AdcManagerClass::setup()
{
    /*
     * A) Pins setup
     */
#ifdef mkrwifi1010
    int pins[] = {PIN_A0, PIN_A1, PIN_A2, PIN_A3, PIN_A4, PIN_A5, PIN_A6};
    int pins_size = 7;
#endif
#ifdef nano_33_iot
    int pins[] = {PIN_A0, PIN_A1, PIN_A2, PIN_A3, PIN_A4, PIN_A5, PIN_A6, PIN_A7};
    int pins_size = 8;
#endif

    for (int i = 0; i < pins_size; i++)
    {
        PinDescription pdesc = g_APinDescription[pins[i]];

        // make PIN INPUT
        PORT->Group[pdesc.ulPort].DIRCLR.reg = (uint32_t)(1 << pdesc.ulPin);

        // Connect the ADC (Peripheral B)
        uint32_t temp;
        if (pdesc.ulPin & 1)
        {

            // Get whole current setup for both odd and even pins and remove odd one
            temp = (PORT->Group[pdesc.ulPort].PMUX[pdesc.ulPin >> 1].reg) & PORT_PMUX_PMUXE(0xF);
            // Set new muxing
            PORT->Group[pdesc.ulPort].PMUX[pdesc.ulPin >> 1].reg = temp | PORT_PMUX_PMUXO(PIO_ANALOG);
        }
        else
        {

            temp = (PORT->Group[pdesc.ulPort].PMUX[pdesc.ulPin >> 1].reg) & PORT_PMUX_PMUXO(0xF);
            PORT->Group[pdesc.ulPort].PMUX[pdesc.ulPin >> 1].reg = temp | PORT_PMUX_PMUXE(PIO_ANALOG);
        }

        // Enable port mux
        PORT->Group[pdesc.ulPort].PINCFG[pdesc.ulPin].reg |= PORT_PINCFG_PMUXEN;
    }

    /*
     * B) ADC setup
     * MAX sample time is 0,1ms which allowes for 200 samples per 20ms - good enough for sampling a 50Hz-sine wave
     * 
     * All those options will give the same timing:
     * (some avergaing is good to even out some noise)
     * 
     * PRESCALE | Nsamples
     *    DIV8  | 16
     *   DIV16  | 8 *
     *   DIV32  | 4
     *   DIV64  | 2
     *  DIV128  | 1
     */

    // disable ADC
    ADC->CTRLA.bit.ENABLE = 0;
    while (ADC->STATUS.bit.SYNCBUSY == 1)
        ;

    ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV16 | ADC_CTRLB_FREERUN;
    while (ADC->STATUS.bit.SYNCBUSY == 1)
        ;

    // Averaging 8 samples
    ADC->CTRLB.bit.RESSEL = 1;
    while (ADC->STATUS.bit.SYNCBUSY == 1)
        ;
    ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_8 | ADC_AVGCTRL_ADJRES(3);
    while (ADC->STATUS.bit.SYNCBUSY == 1)
        ;

    // REFCTRL
    ADC->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC1; // 1/2 VDDANA reference
    while (ADC->STATUS.bit.SYNCBUSY == 1)
        ;

    ADC->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_GND | // negative to GND
                         ADC_INPUTCTRL_GAIN_DIV2;
    while (ADC->STATUS.bit.SYNCBUSY == 1)
        ;

    Log.log("ADC setup complete");
}

void AdcManagerClass::changeInputPin(unsigned int analogPinName)
{
    // Disable ADC
    ADC->CTRLA.bit.ENABLE = 0;
    while (ADC->STATUS.bit.SYNCBUSY == 1)
        ;

    // Selection for the positive ADC input
    ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[analogPinName].ulADCChannelNumber;
    while (ADC->STATUS.bit.SYNCBUSY == 1)
        ;

    // Enable ADC
    ADC->CTRLA.bit.ENABLE = 1;
    while (ADC->STATUS.bit.SYNCBUSY == 1)
        ;
}

// 100us per sample. 200 samples == 20000us/20ms
struct ADCMeasurement AdcManagerClass::read(unsigned int numberOgSamples)
{

    struct ADCMeasurement aDCMeasurement;
    while (numberOgSamples-- > 0)
    {
        // no need to start - using free running
        //ADC->SWTRIG.bit.START = 1;

        // Waiting for the to complete
        while (ADC->INTFLAG.bit.RESRDY == 0)
            ;

        unsigned long newValue = ADC->RESULT.reg;
        while (ADC->STATUS.bit.SYNCBUSY == 1)
            ;

        if (newValue > aDCMeasurement.highest)
        {
            aDCMeasurement.highest = newValue;
        }
        if (newValue < aDCMeasurement.lowest)
        {
            aDCMeasurement.lowest = newValue;
        }
    }

    return aDCMeasurement;
}

bool AdcManagerClass::updatePilotVoltageAndProximityPilotAmps()
{
    AdcManager.changeInputPin(pinControlPilot);
    ADCMeasurement cpADCMeasurement = AdcManager.read(200);
    PilotVoltage newPilotVoltage = AdcManager.toControlPilot(cpADCMeasurement);

    AdcManager.changeInputPin(pinProxymityPilot);
    ADCMeasurement ppADCMeasurement = AdcManager.read(200);
    ProximityPilotAmps newProximityPilotAmps = AdcManager.toProximityPilot(ppADCMeasurement);

    bool changeDetected = false;
    if (newPilotVoltage != currentPilotVoltage)
    {
        char buf[100];
        snprintf(buf, 100, "Pilot voltage changed from %d to %d", currentPilotVoltage, newPilotVoltage);
        Log.log(buf);
        currentPilotVoltage = newPilotVoltage;
        changeDetected = true;
    }
    if (newProximityPilotAmps != currentProximityPilotAmps)
    {
        char buf[100];
        snprintf(buf, 100, "Proximity voltage changed from %d to %d", currentProximityPilotAmps, newProximityPilotAmps);
        Log.log(buf);
        currentProximityPilotAmps = newProximityPilotAmps;
        changeDetected = true;
    }

    return changeDetected;
}

unsigned long AdcManagerClass::adcValueToMilliVolts(ADCMeasurement aDCMeasurement)
{
    // 0 => 0V | 4095 => 3.3V
    uint32_t microVolts = aDCMeasurement.highest * 806;
    return (uint16_t)(microVolts / 1000);
}

unsigned long AdcManagerClass::toMainsMilliAmpsRms(ADCMeasurement aDCMeasurement)
{
    // TODO
    return 0;
}

unsigned long AdcManagerClass::toMainsMilliVoltsRms(ADCMeasurement aDCMeasurement)
{
    // TODO
    return 0;
}

ProximityPilotAmps AdcManagerClass::toProximityPilot(ADCMeasurement aDCMeasurement)
{
    /*
     * 4095 / 330V = 12,409090909090909
     * 
     * (spice simulated)
     * 
     *   330Ω => 0.708V =>  869 ADC
     *  1000Ω => 1,660V => 2059 ADC
     *  2700Ω => 2,880V => 3573 ADC
     * 
     * https://en.wikipedia.org/wiki/SAE_J1772
     * 
     * 32A - 150 Ω - 330 Ω  - (220  Ω => 0.33V)
     * 20A - 330 Ω – 1 kΩ   - (680  Ω => 1.25V)
     * 13A - 1 k Ω - 2.7 kΩ - (1.5 kΩ => 2.20V)
     */
    unsigned long millivolts = adcValueToMilliVolts(aDCMeasurement);

    if (millivolts < 708)
    {
        return Amp32;
    }
    else if (millivolts < 1660)
    {
        return Amp20;
    }
    else
    {
        return Amp13;
    }
}

PilotVoltage AdcManagerClass::toControlPilot(ADCMeasurement aDCMeasurement)
{
    /*
     * 4095 / 330V = 12,409090909090909
     * 
     * (spice simulated)
     * CP volt | Volt on ADC 
     *    12.0 | 3.13
     *    10.5 | 2.95
     *     7.5 | 2.56        
     *     4,5 | 2.17
     *     1,5 | 1.78
     *       0 | 1.58
     *   -12.0 | 0
     * 
     */

    unsigned long millivolts = adcValueToMilliVolts(aDCMeasurement);

    if (millivolts < 1780)
    {
        return Fault;
    }
    else if (millivolts < 2170)
    {
        return Volt_3;
    }
    else if (millivolts < 2560)
    {
        return Volt_6;
    }
    else if (millivolts < 2950)
    {
        return Volt_9;
    }
    else
    {
        return Volt_12;
    }
}
