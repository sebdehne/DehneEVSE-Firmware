#ifndef SMART_HOME_SERVER_CLIENT_H
#define SMART_HOME_SERVER_CLIENT_H

#include <WiFiNINA.h>
#include <CRC32.h>
#include "secrets.h"
#include "charger.h"
#include "sensor_voltage.h"
#include "sensor_current.h"
#include "logger.h"
#include "config.h"

const char MY_SSID[] = SECRET_SSID;
const char MY_PASS[] = SECRET_PASS;

#define REQUEST_TYPE_PING 1
#define REQUEST_TYPE_FIRMWARE 2
#define REQUEST_TYPE_GET_DATA 3
#define REQUEST_TYPE_SET_CHARGE_RATE 4

#define RESPONSE_TYPE_PONG 1
#define RESPONSE_TYPE_DATA 2
#define RESPONSE_TYPE_SET_CHARGE_RATE 3

class SmartHomeServerClient
{
private:
    CRC32 crc32;

    byte requestInitialBytes[5];
    int requestInitialBytesRead = 0;

    // non-firmware requests
    byte requestBodyBytes[REQUEST_BODY_BUFFER_SIZE];
    int requestBodyBytesRead = 0;

    // firmware
    byte requestFirmwareCRC32Bytes[4];
    int requestFirmwareCRC32BytesRead = 0;
    int requestFirmwareBytesStored = 0;

    int wifiStatus = WL_IDLE_STATUS;
    WiFiClient client;
    IPAddress server;
    unsigned long lastRequestFromServer;
    unsigned long lastConnectionReset;
    Charger *charger;
    SensorVoltage *sensorVoltage;
    SensorCurrent *sensorCurrent;

    void connectToWifi();
    void prepareReadingNextRequest();

    int readTimedInto(byte dst[], int dstOffset, int length);
    int readTimed();

    bool handleFirmwareUpgrade(int msgLen);
    void handleRequest(int requestType, int msgLen);
    void handlePing();
    void handleGetData();

    void writeUint32(unsigned int src, byte dst[], int dstOffset);
    void writeInt32(int src, byte dst[], int dstOffset);
    void writeCharArray(char src[], int srcLength, byte dst[], int dstOffset);
    void writeSerial16Bytes(byte dst[], int dstOffset);

    int toInt(byte src[], int srcOffset);
    unsigned int toUInt(byte src[], int srcOffset);

public:
    SmartHomeServerClient();
    void setup(Charger *charger, SensorVoltage *sensorVoltage, SensorCurrent *sensorCurrent);

    void tick();
};

#endif