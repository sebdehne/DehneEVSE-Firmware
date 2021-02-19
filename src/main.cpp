#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFiNINA.h>
#include <CRC32.h>

#include "config.h"
#include "secrets.h"

const u_int8_t VERSION = 1;

const char MY_SSID[] = SECRET_SSID;
const char MY_PASS[] = SECRET_PASS;
const u_int8_t MY_CLIENT_ID = CLIENT_ID;

WiFiClient client;
int status = WL_IDLE_STATUS;

IPAddress server;

void setup()
{
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  while (!Serial)
    ;

  if (!server.fromString(SERVER_IP))
  {
    Serial.print("Could not parse IP-address: ");
    Serial.println(SERVER_IP);
    while (1)
      ;
  }

  client.setTimeout(10 * 1000); // 10 seconds

  Serial.print("Startup OK; version: ");
  Serial.println(VERSION);
}

void loop()
{

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