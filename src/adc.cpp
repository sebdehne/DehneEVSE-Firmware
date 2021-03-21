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
     * MAX sample time is 0,025ms which allowes for 800 samples per 20ms - good enough for sampling a 50Hz-sine wave
     * 
     * All those options will give the same timing:
     * (some avergaing is good to even out some noise)
     * 
     * 0,1ms sample time:
     * PRESCALE | Nsamples
     *    DIV8  | 16 *
     *   DIV16  | 8
     *   DIV32  | 4
     *   DIV64  | 2
     *  DIV128  | 1
     * 
     * 0,05ms sample time:
     * PRESCALE | Nsamples
     *    DIV4  | 16
     *    DIV8  | 8
     *   DIV16  | 4
     *   DIV32  | 2
     *   DIV64  | 1
     * 
     * 0,025ms sample time:
     * PRESCALE | Nsamples
     *    DIV4  | 8
     *    DIV8  | 4
     *   DIV16  | 2
     *   DIV32  | 1 (most stable output)
     * 
     */

    // disable ADC
    ADC->CTRLA.bit.ENABLE = 0;
    while (ADC->STATUS.bit.SYNCBUSY == 1)
        ;

    ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV32 | ADC_CTRLB_FREERUN;
    while (ADC->STATUS.bit.SYNCBUSY == 1)
        ;

    // Averaging 8 samples
    //ADC->CTRLB.bit.RESSEL = 1;
    //while (ADC->STATUS.bit.SYNCBUSY == 1)
    //    ;
    //ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_8 | ADC_AVGCTRL_ADJRES(3);
    //while (ADC->STATUS.bit.SYNCBUSY == 1)
    //    ;

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

    read(10); // changing input produced invalid data - consume it first
}

// ***25us*** per sample
// for 50Hz -> 800 sampels to catch 1 cycle
// for 1kHz -> 40 sampels catches 1 cycle
struct ADCMeasurement AdcManagerClass::read(unsigned int numberOgSamples)
{

    int avg = 8;
    int shift = 3;
    unsigned long sample;
    unsigned int remaining = numberOgSamples / avg;

    struct ADCMeasurement aDCMeasurement;
    aDCMeasurement.highest = 0;
    aDCMeasurement.lowest = 1 << 12;
    while (remaining-- > 0)
    {

        sample = 0;
        for (int i = 0; i < avg; i++)
        {
            // no need to start - using free running
            //ADC->SWTRIG.bit.START = 1;

            // Waiting for the to complete
            while (ADC->INTFLAG.bit.RESRDY == 0)
                ;

            unsigned long newValue = ADC->RESULT.reg;
            while (ADC->STATUS.bit.SYNCBUSY == 1)
                ;

            sample = sample + newValue;
        }

        sample = sample >> shift;

        if (sample > aDCMeasurement.highest)
        {
            aDCMeasurement.highest = sample;
        }
        if (sample < aDCMeasurement.lowest)
        {
            aDCMeasurement.lowest = sample;
        }
    }

    return aDCMeasurement;
}

bool AdcManagerClass::updatePilotVoltageAndProximityPilotAmps()
{
    AdcManager.changeInputPin(pinControlPilot);
    ADCMeasurement cpADCMeasurement = AdcManager.read(160); // 4 cycles
    PilotVoltage newPilotVoltage = AdcManager.toControlPilot(cpADCMeasurement);

    AdcManager.changeInputPin(pinProxymityPilot);
    ADCMeasurement ppADCMeasurement = AdcManager.read(160); // 4 cycles
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


ProximityPilotAmps AdcManagerClass::toProximityPilot(ADCMeasurement aDCMeasurement)
{
    /*
     * 4095 / 330V = 12,409090909090909
     * 
     * Measured:
     * 
     * open     -> 2484 mV (no cable)
     * 1500 Ohm -> 1890 mV (13A) (2187)
     *  680 Ohm -> 1466 mV (20A) (1678)
     *  220 Ohm ->  793 mV (32A) (1129)
     * 
     * https://en.wikipedia.org/wiki/SAE_J1772
     * 
     */
    unsigned long millivolts = adcValueToMilliVolts(aDCMeasurement);

    if (millivolts < 1129)
    {
        return Amp32;
    }
    else if (millivolts < 1678)
    {
        return Amp20;
    }
    else if (millivolts < 2187)
    {
        return Amp13;
    }
    else
    {
        return NoCable;
    }
}

PilotVoltage AdcManagerClass::toControlPilot(ADCMeasurement aDCMeasurement)
{
    /*
     * R9 = 47k
     * R10 = 200k
     * R11 = 82k
     * 
     * Board-A values:
     * Open      : 2500mV
     * Connected : 1767mV
     * Ready     : 1099mV
     * 
     * Open:      2324mV (step: 634)
     * Connected: 1690mV (step: 535)
     * Ready:     1155mV (step: 535)
     * Vent :     620mV
     * Fault :    
     * Car 2.6k -> millivolts=2546
     * 
     */

    unsigned long millivolts = adcValueToMilliVolts(aDCMeasurement);

    if (millivolts < 352)
    {
        return Fault;
    }
    else if (millivolts < 887)
    {
        return Volt_3;
    }
    else if (millivolts < 1422)
    {
        return Volt_6;
    }
    else if (millivolts < 2007)
    {
        return Volt_9;
    }
    else
    {
        return Volt_12;
    }
}
