#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFiNINA.h>
#include <CRC32.h>
#include <WDTZero.h>

#include "config.h"
#include "secrets.h"

const u_int8_t VERSION = 1;

const char MY_SSID[] = SECRET_SSID;
const char MY_PASS[] = SECRET_PASS;
const u_int8_t MY_CLIENT_ID = CLIENT_ID;

WiFiClient client;
int status = WL_IDLE_STATUS;

WDTZero MyWatchDoggy;
IPAddress server;
uint8_t pwmDutyCycle_percent = 0;

void setup()
{
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  // setup ADC & DAC
  analogWriteResolution(10);
  analogReadResolution(12);

  // setup Serial
  Serial.begin(115200);
#ifdef DEBUG
  while (!Serial)
    ;
#endif

  MyWatchDoggy.setup(WDT_SOFTCYCLE32S);

  // parse server-ip
  if (!server.fromString(SERVER_IP))
  {
    Serial.print("Could not parse IP-address: ");
    Serial.println(SERVER_IP);
    while (1)
      ;
  }

  // configure TCP read timeout
  client.setTimeout(10 * 1000); // 10 seconds

  // start PWM @ D3
  PWM_1kHz_tcc0_PIN_D3_start(pwmDutyCycle_percent);

  Serial.print("Startup OK; version: ");
  Serial.println(VERSION);
}

void loop()
{
  MyWatchDoggy.clear();

  PWM_1kHz_tcc0_PIN_D3_adjust_duty_cycle(pwmDutyCycle_percent);

  pwmDutyCycle_percent += 10;
  if (pwmDutyCycle_percent >= 100)
  {
    pwmDutyCycle_percent = 0;
  }

  // For later:
  //
  // writing to the DAC: 10 bits (0-1023) on A0
  // analogWrite(PIN_A0, value);
  //
  // reading from ADC: 12 bits (0-4096) on A1-A6
  // analogRead(PIN_A1);

  // ensure wifi is up
  status = WiFi.status();
  if (status != WL_CONNECTED)
  {
    connectToWifi();
  }

  // ensure TCP to server is up
  if (!client.connected())
  {
    delay(5000);
    client.stop();
    if (client.connect(server, 9091))
    {
      Serial.print("Connected to server: ");
      Serial.print(SERVER_IP);
      Serial.print(":");
      Serial.println(9091);

      // send CLientId
      client.write(MY_CLIENT_ID);
      // send Firmware version
      client.write(VERSION);
    }
    else
    {
      Serial.println("Could not connect to server");
      return;
    }
  }

  // read next request from server
  byte b;
  if (!client.readBytes(&b, 1))
  {
    // nothing here right now
    Serial.println("Timeout while reading next command. Giving up connection");
    client.stop();
    return;
  }
  u_int8_t requestType = (u_int8_t)b;

#ifdef DEBUG
  Serial.print("Got requestType: ");
  Serial.println(requestType);
#endif

  // msg-length - 32bit int
  byte buf[4];
  if (!client.readBytes(buf, 4))
  {
    client.stop();
    return;
  }
  u_int32_t msgLen = toInt(buf);

#ifdef DEBUG
  Serial.print("Got msgLen: ");
  Serial.println(msgLen);
#endif

  if (requestType == REQUEST_TYPE_PING)
  {
    Serial.println("Responding to PING");
    // write pong response

    // responseType: pong
    client.write(RESPONSE_TYPE_PONG);
    // msgLen: 0
    intToByteArray(0, buf);
    client.write(buf, 4);
  }

  else if (requestType == REQUEST_TYPE_FIRMWARE)
  {
    const uint32_t firmwareSize = msgLen - 4;

    // read CRC32
    if (!client.readBytes(buf, 4))
    {
      client.stop();
      return;
    }
    u_int32_t crc32 = toInt(buf);

    // write content to flash:
    uint32_t remaining = firmwareSize;
    InternalStorage.open(firmwareSize);
    while (remaining > 0)
    {
      if (!client.readBytes(&b, 1)) // reading a byte with timeout
        break;
      InternalStorage.write(b);
      remaining--;
    }
    InternalStorage.close();

    if (remaining > 0)
    {
      Serial.print("Timeout downloading update file at ");
      Serial.print(remaining);
      Serial.println(" bytes. Can't continue with update.");
      client.stop();
      return;
    }

    // calc crc32
    CRC32 crc;
    uint8_t *addr = (uint8_t *)InternalStorage.STORAGE_START_ADDRESS;
    for (size_t i = 0; i < firmwareSize; i++)
    {
      crc.update(*addr);
      addr++;
    }
    uint32_t firmwareChecksum = crc.finalize();

    if (firmwareChecksum != crc32)
    {
      Serial.print("Received ");
      Serial.print(firmwareSize);
      Serial.print(" bytes, but received CRC32=");
      Serial.print(crc32);
      Serial.print(" doesnt match calculated CRC32=");
      Serial.println(firmwareChecksum);
    }
    else
    {
      Serial.println("Rebooting to new firmware now");
      Serial.flush();
      InternalStorage.apply();
    }
  }
}

void connectToWifi()
{
  while (status != WL_CONNECTED)
  {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(MY_SSID);
    status = WiFi.begin(MY_SSID, MY_PASS);
    delay(10000);
  }
}

u_int32_t toInt(byte buf[])
{
  u_int32_t result = 0;
  result = result + buf[0];
  result <<= 8;
  result = result + buf[1];
  result <<= 8;
  result = result + buf[2];
  result <<= 8;
  result = result + buf[3];
  return result;
}

void intToByteArray(uint32_t input, byte dst[])
{
  dst[0] = (input >> 24) & 0xFF;
  dst[1] = (input >> 16) & 0xFF;
  dst[2] = (input >> 8) & 0xFF;
  dst[3] = input & 0xFF;
}

/*
 * Using dual-slope PWM
 * 
 * Freq = GCLK_TCC / (2 * divider * PER) => 1000Hz = 48MhZ / (2 * 64 * 375)
 * 
 * Thus: DIV=64 & PER=375
 */
int per = 375;
void PWM_1kHz_tcc0_PIN_D3_start(uint8_t dutyCycle)
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

    // Set the PWM signal to output 50% duty cycle

#ifdef mkrwifi1010
  REG_TCC0_CC3 = (per * dutyCycle) / 100; // TCC0 CC3/WO[3] - on D3
#endif
#ifdef nano_33_iot
  REG_TCC0_CC5 = (per * dutyCycle) / 100; // TCC0 CC5/WO[5] - on D3
#endif
  while (TCC0->SYNCBUSY.bit.CC3)
    ; // Wait for synchronization

  // C.4 Enable
  REG_TCC0_CTRLA |= TCC_CTRLA_ENABLE; // Enable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE)
    ; // Wait for synchronization
}

void PWM_1kHz_tcc0_PIN_D3_adjust_duty_cycle(uint8_t dutyCycle)
{
#ifdef mkrwifi1010
  REG_TCC0_CC3 = (per * dutyCycle) / 100; // TCC0 CC3/WO[3] - on D3
#endif
#ifdef nano_33_iot
  REG_TCC0_CC5 = (per * dutyCycle) / 100; // TCC0 CC5/WO[5] - on D3
#endif
  while (TCC0->SYNCBUSY.bit.CC3)
    ; // Wait for synchronization
}