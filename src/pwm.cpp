#include <Arduino.h>
#include "pwm.h"
#include "logger.h"

PwmD3::PwmD3() {}

/*
 * Using dual-slope PWM
 * 
 * PWM_freq = GCLK_TCC / (2 * divider * PER) => 1000Hz = 48MHz / (2 * 64 * 375)
 * 
 * Thus: DIV=64 & PER=375
 */
void PwmD3::setup(uint8_t initialPwmDutyCycle_percent)
{
  // uses GCLK4
  // MRK 1010:      PA11
  // Nanon 33 IoT:  PB11

  /*
   * A) Output PIN configufation:
   */
  // Enable the port multiplexer for the digital pin D3
  PORT->Group[g_APinDescription[3].ulPort].PINCFG[g_APinDescription[3].ulPin].bit.PMUXEN = 1;

  // Connect the TCC0 timer to digital output D3 - port pins are paired odd PMUO and even PMUXE
  PORT->Group[g_APinDescription[3].ulPort].PMUX[g_APinDescription[3].ulPin >> 1].reg = PORT_PMUX_PMUXO_F;

  /*
  * B) Clock setup, which is sent to the TCC0 module
  */
  // B.1) Enable the TCC bus clock (CLK_TCCx_APB)
  // B.1.a) GENDIV - Generic Clock Generator division factor
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) | // Divide the 48MHz clock source by divisor 1: 48MHz/1=48MHz
                    GCLK_GENDIV_ID(4);   // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY)
    ; // Wait for synchronization

  // B.1.b) GENCTRL
  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |         // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |       // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M | // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);        // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY)
    ; // Wait for synchronization

  // B.1.c) CLKCTRL
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |       // Enable
                     GCLK_CLKCTRL_GEN_GCLK4 |   // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC0_TCC1; // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY)
    ; // Wait for synchronization

  /*
   * C) Tcc0 module setup
   */
  // C.1 Set PRESCALER to 64
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV64; // Divide GCLK4 by 1014 gives 46.875kHz
  while (TCC0->SYNCBUSY.bit.ENABLE)
    ; // Wait for synchronization

  // C.2 Waveform operation setup
  REG_TCC0_WAVE |= TCC_WAVE_POL(0xF) |      // Reverse the output polarity on all TCC0 outputs
                   TCC_WAVE_WAVEGEN_DSBOTH; // Setup dual slope PWM on TCC0
  while (TCC0->SYNCBUSY.bit.WAVE)
    ; // Wait for synchronization

  // C.3 SET PER
  REG_TCC0_PER = per;
  while (TCC0->SYNCBUSY.bit.PER)
    ; // Wait for synchronization

  // Set start duty cycle
  updateDutyCycle(initialPwmDutyCycle_percent);

  // C.4 Enable
  REG_TCC0_CTRLA |= TCC_CTRLA_ENABLE; // Enable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE)
    ; // Wait for synchronization


  Log.log("PWM setup complete");
}

void PwmD3::updateDutyCycle(uint8_t pwmDutyCycle_percent)
{
#ifdef mkrwifi1010
  REG_TCC0_CC3 = (per * pwmDutyCycle_percent) / 100; // TCC0 on D3 -> WO[3] -> CC3
  while (TCC0->SYNCBUSY.bit.CC3)
    ; // Wait for synchronization
#endif
#ifdef nano_33_iot
  REG_TCC0_CC1 = (per * pwmDutyCycle_percent) / 100; // TCC0 on D3 -> WO[5] -> CC1
  while (TCC0->SYNCBUSY.bit.CC1)
    ; // Wait for synchronization
#endif
}


