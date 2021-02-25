#include <Arduino.h>
#include "adc.h"
#include "types.h"

AdcManager::AdcManager()
{
}

void AdcManager::setup()
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

    Serial.println("ADC setup complete");
}

void AdcManager::changeInputPin(uint8_t analogPinName)
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
struct ADCMeasurement AdcManager::read(uint16_t numberOgSamples)
{

    struct ADCMeasurement aDCMeasurement;
    while (numberOgSamples-- > 0)
    {
        // no need to start - using free running
        //ADC->SWTRIG.bit.START = 1;

        // Waiting for the to complete
        while (ADC->INTFLAG.bit.RESRDY == 0)
            ;

        int newValue = ADC->RESULT.reg;
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

uint32_t AdcManager::toMilliAmps(ADCMeasurement aDCMeasurement)
{
    // TODO
    return 0;
}

uint32_t AdcManager::toMilliVolts(ADCMeasurement aDCMeasurement)
{
    // TODO
    return 0;
}

ProximityPilotAmps AdcManager::toProximityPilot(ADCMeasurement aDCMeasurement)
{

    // TODO
    /*
     * https://en.wikipedia.org/wiki/SAE_J1772
     * 
     * (spice simulated)
     * 13A - 1 k Ω - 2.7 kΩ - (1.5 kΩ => 2.20V)
     * 20A - 330 Ω – 1 kΩ   - (680  Ω => 1.25V)
     * 32A - 150 Ω - 330 Ω  - (220  Ω => 0.33V)
     */
    return Amp13;
}

PilotVoltage AdcManager::toControlPilot(ADCMeasurement aDCMeasurement)
{
    // TODO
    return Volt_12;
}
