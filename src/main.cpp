#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFiNINA.h>
#include <CRC32.h>
#include <WDTZero.h>

#include "config.h"
#include "adc.h"
#include "pwm.h"
#include "charger.h"
#include "secrets.h"
#include "contactor.h"
#include "sensor_voltage.h"
#include "sensor_current.h"

const u_int8_t VERSION = 1;

const char MY_SSID[] = SECRET_SSID;
const char MY_PASS[] = SECRET_PASS;

WiFiClient client;
int status = WL_IDLE_STATUS;

WDTZero wdt;
IPAddress server;

AdcManager adcManager;
PwmD3 pwmD3;
Charger charger(IS_OUTDOOR, DEFAULT_CHARGE_CURRENT_AMPS);
Contactor contactor;
SensorVoltage sensorVoltage;
SensorCurrent sensorCurrent;
uint32_t lastRequestFromServer;

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

  // parse server-ip
  if (!server.fromString(SERVER_IP))
  {
    Serial.print("Could not parse IP-address: ");
    Serial.println(SERVER_IP);
    while (1)
      ;
  }

  // configure TCP read timeout
  client.setTimeout(500);

  // setup & start PWM @ D3
  pwmD3.setup(100);

  // Setup ADC for pins A0-A7
  adcManager.setup();

  // setup Contactor
  contactor.setup();

  // last step: setup charger
  charger.setup(pwmD3, contactor);

  Serial.print("Startup OK; version: ");
  Serial.println(VERSION);
}

int p = 10;
void loop()
{
  wdt.clear();

  // handle charger logic
  charger.tick(pwmD3, adcManager, contactor);

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
      lastRequestFromServer = millis();

      // send CLientId
      writeSerial();
      // send Firmware version
      client.write(VERSION);
    }
    else
    {
      Serial.println("Could not connect to server");
      return;
    }
  }

  // check for request from server
  byte b;
  if (client.readBytes(&b, 1))
  {
    lastRequestFromServer = millis();
    handleRequestFromServer((u_int8_t)b);
    return;
  }

  // server timeout
  if (millis() - lastRequestFromServer > SERVER_READ_TIMEOUT_IN_MS)
  {
    // server timeout
    Serial.println("Timeout while reading next command. Giving up connection");
    client.stop();
    return;
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

void handleRequestFromServer(u_int8_t requestType)
{
  byte b;

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

void writeUint32Array(uint32_t word[], size_t length)
{
  for (size_t i = 0; i < length; i++)
  {
    writeUint32(word[i]);
  }
}

void writeUint32(uint32_t word)
{
  byte dst[4];

  dst[0] = (word >> 24) & 0xFF;
  dst[1] = (word >> 16) & 0xFF;
  dst[2] = (word >> 8) & 0xFF;
  dst[3] = word & 0xFF;

  client.write(dst, 4);
}

void writeSerial()
{
  uint32_t serial[4];

  volatile uint32_t *ptr1 = (volatile uint32_t *)0x0080A00C;
  writeUint32(*ptr1);
  volatile uint32_t *ptr = (volatile uint32_t *)0x0080A040;
  writeUint32(*ptr);
  ptr++;
  writeUint32(*ptr);
  ptr++;
  writeUint32(*ptr);
}