#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFiNINA.h>
#include <CRC32.h>
#include <WDTZero.h>

#include "config.h"
#include "adc.h"
#include "pwm.h"
#include "contactor.h"
#include "smart_home_server_client.h"
#include "logger.h"

WDTZero wdt;

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
  PwmD3.setup();

  // Setup ADC for pins A0-A7
  AdcManager.setup();

  // setup Contactor
  Contactor.setup();

  // setup server communication
  SmartHomeServerClient.setup();

  char logMsg[100];
  snprintf(logMsg, 100, "Startup OK; version: %d", VERSION);
  Log.log(logMsg);
}

void loop()
{
  wdt.clear();

  bool dataChanged = AdcManager.updatePilotVoltageAndProximityPilotAmps();

  SmartHomeServerClient.tick(dataChanged);
}
