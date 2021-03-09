#ifndef SMART_HOME_SERVER_CLIENT_H
#define SMART_HOME_SERVER_CLIENT_H

#include <WiFiNINA.h>
#include <CRC32.h>
#include "secrets.h"
#include "logger.h"

const char MY_SSID[] = SECRET_SSID;
const char MY_PASS[] = SECRET_PASS;

#define REQUEST_TYPE_PING 1
#define REQUEST_TYPE_FIRMWARE 2
#define REQUEST_TYPE_COLLECT_DATA 3
#define REQUEST_TYPE_SET_PWM_PERCENT 4
#define REQUEST_TYPE_SET_CONTACTOR_STATE 5

#define RESPONSE_TYPE_PONG 1
#define RESPONSE_TYPE_COLLECT_DATA 2
#define RESPONSE_TYPE_SET_PWM_PERCENT 3
#define RESPONSE_TYPE_SET_CONTACTOR_STATE 4

#define NOTIFY_TYPE_DATA_CHANGED 100

class SmartHomeServerClientClass
{
private:
    CRC32 crc32;

    unsigned char requestInitialBytes[5];
    int requestInitialBytesRead = 0;

    // non-firmware requests
    unsigned char requestBodyBytes[REQUEST_BODY_BUFFER_SIZE];
    int requestBodyBytesRead = 0;

    // firmware
    unsigned char requestFirmwareCRC32Bytes[4];
    int requestFirmwareCRC32BytesRead = 0;
    int requestFirmwareBytesStored = 0;

    int wifiStatus = WL_IDLE_STATUS;
    WiFiClient client;
    IPAddress server;
    unsigned long lastRequestFromServer;
    unsigned long lastConnectionReset;

    void connectToWifi();
    void prepareReadingNextRequest();

    int readInto(byte dst[], int dstOffset, int length);
    int readSingle();

    bool handleFirmwareUpgrade(int msgLen);
    void handleRequest(int requestType, int msgLen);
    void handlePing();
    void handleCollectData();

public:
    SmartHomeServerClientClass();
    void setup();

    void tick(bool dataChangeDetected);
};

extern SmartHomeServerClientClass SmartHomeServerClient;

#endif