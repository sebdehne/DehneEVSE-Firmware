#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFiNINA.h>
#include <Arduino_CRC32.h>

#include "config.h"

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
      Serial.println("Connected to server");

      // send CLientId
      client.write(MY_CLIENT_ID);
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

  if (requestType == 1)
  {
    Serial.println("Responding to PING");
    // write pong response
    client.write(1); // responseType: pong
    intToByteArray(1, buf);
    client.write(buf, 4); // msgLen: 1
    client.write(VERSION);
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