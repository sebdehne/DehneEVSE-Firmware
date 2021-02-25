#ifndef SMART_HOME_SERVER_CLIENT_H
#define SMART_HOME_SERVER_CLIENT_H

#include <WiFiNINA.h>
#include <CRC32.h>
#include "secrets.h"
#include "charger.h"
#include "sensor_voltage.h"
#include "sensor_current.h"
#include "logger.h"

const char MY_SSID[] = SECRET_SSID;
const char MY_PASS[] = SECRET_PASS;

#define REQUEST_TYPE_PING 1
#define REQUEST_TYPE_FIRMWARE 2
#define REQUEST_TYPE_GET_DATA 3

#define RESPONSE_TYPE_PONG 1
#define RESPONSE_TYPE_DATA 2

class SmartHomeServerClient
{
private:
    CRC32 crc32;

    uint8_t requestInitialBytes[5];
    uint8_t bytesRead = 0;

    int wifiStatus = WL_IDLE_STATUS;
    WiFiClient client;
    IPAddress server;
    uint32_t lastRequestFromServer;
    Charger *charger;
    SensorVoltage *sensorVoltage;
    SensorCurrent *sensorCurrent;
    Logger *logger;

    void connectToWifi();

    void handleRequest(uint8_t requestType, uint32_t msgLen);
    void handlePing();
    void handleFirmwareUpgrade(u_int32_t msgLen);
    void handleGetData();

    void writeUint32(uint32_t src, uint8_t dst[], size_t dstOffset);
    void writeInt32(int32_t src, uint8_t dst[], size_t dstOffset);
    void writeUint32Array(uint32_t src[], size_t srcLen, uint8_t dst[], size_t dstOffset);
    void writeCharArray(char src[], size_t srcLength, uint8_t dst[], size_t dstOffset);
    void writeSerial16Bytes(uint8_t dst[], size_t dstOffset);

    uint32_t toInt(byte src[], size_t srcOffset);

public:
    SmartHomeServerClient();
    void setup(Charger *charger, SensorVoltage *sensorVoltage, SensorCurrent *sensorCurrent, Logger *logger);

    void tick();
};

#endif