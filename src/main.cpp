#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFiNINA.h>
#include <CRC32.h>
#include <WDTZero.h>

#include "config.h"
#include "adc.h"
#include "pwm.h"
#include "charger.h"
#include "contactor.h"
#include "sensor_voltage.h"
#include "sensor_current.h"
#include "smart_home_server_client.h"
#include "logger.h"

SmartHomeServerClient smartHomeServerClient;
WDTZero wdt;
AdcManager adcManager;
PwmD3 pwmD3;
Charger charger(IS_OUTDOOR, DEFAULT_CHARGE_CURRENT_AMPS);
Contactor contactor;
SensorVoltage sensorVoltage;
SensorCurrent sensorCurrent;
Logger logger;

void setup()
{
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  // setup Serial
  Serial.begin(115200);
#ifdef DEBUG
  while (!Serial)
    ;
#endif

  // Setup WDT
  wdt.setup(WDT_SOFTCYCLE32S);

  // setup & start PWM @ D3
  pwmD3.setup(100);

  // Setup ADC for pins A0-A7
  adcManager.setup();

  // setup Contactor
  contactor.setup();

  // last step: setup charger
  charger.setup(pwmD3, contactor, &logger);

  // setup server communication
  smartHomeServerClient.setup(&charger, &sensorVoltage, &sensorCurrent, &logger);

  Serial.print("Startup OK; version: ");
  Serial.println(VERSION);
}

void loop()
{
  wdt.clear();

  // read voltage and current
  sensorCurrent.read(adcManager);
  sensorVoltage.read(adcManager);

  // handle charger logic
  charger.tick(pwmD3, adcManager, contactor);

  // handle server communication (has read-timeout of 500ms)
  smartHomeServerClient.tick();

}
