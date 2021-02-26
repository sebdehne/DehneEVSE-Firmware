#include <Arduino.h>
#include "smart_home_server_client.h"
#include "config.h"
#include <WiFiNINA.h>
#include <ArduinoOTA.h>
#include <CRC32.h>

SmartHomeServerClient::SmartHomeServerClient()
{
}

void SmartHomeServerClient::setup(Charger *chargerInput, SensorVoltage *sensorVoltageInput, SensorCurrent *sensorCurrentInput)
{
    charger = chargerInput;
    sensorVoltage = sensorVoltageInput;
    sensorCurrent = sensorCurrentInput;

    // parse server-ip
    if (!server.fromString(SERVER_IP))
    {
        char logBuf[100];
        snprintf(logBuf, 100, "Could not parse IP-address: %s", SERVER_IP);
        Log.log(logBuf);
        while (1)
            ;
    }

    Log.log("SmartHomeServerClient setup complete");
}

void SmartHomeServerClient::tick()
{
    // ensure wifi is up
    wifiStatus = WiFi.status();
    if (wifiStatus != WL_CONNECTED)
    {
        connectToWifi();
    }

    // ensure TCP to server is up
    if (!client.connected())
    {
        client.stop();

        if (millis() - lastConnectionReset < 5000)
        {
            // 5 seconds back-off time
            return;
        }

        lastConnectionReset = millis();
        if (client.connect(server, 9091))
        {
            char buf[100];
            snprintf(buf, 100, "Connected to server: %s:%d", SERVER_IP, 9091);
            Log.log(buf);

            lastRequestFromServer = millis();
            prepareReadingNextRequest();

            // send CLientId
            byte sendingBuf[17];
            writeSerial16Bytes(sendingBuf, 0);
            sendingBuf[16] = VERSION;
            // send Firmware version
            client.write(sendingBuf, 17);
        }
        else
        {
            Log.log("Could not connect to server");
            return;
        }
    }

    // try to read more of the initial 5 bytes (type + msgLen)
    while (requestInitialBytesRead < 5)
    {
        int read = readTimedInto(requestInitialBytes, requestInitialBytesRead, 5 - requestInitialBytesRead);
        if (read > 0)
            requestInitialBytesRead += read;
        else
            break; // timeout is OK, we will come back here

        if (requestInitialBytesRead == 5)
        {
            int msgLen = toInt(requestInitialBytes, 1);
            char logBuf[100];
            snprintf(logBuf, 100, "Got: requestType=%d msgLen=%d", requestInitialBytes[0], msgLen);
            Log.log(logBuf);
        }
    }

    if (requestInitialBytesRead == 5)
    {
        int msgLen = toInt(requestInitialBytes, 1);
        int requestType = (int)requestInitialBytes[0];

        if (requestType == REQUEST_TYPE_FIRMWARE)
        {
            Log.log("Handling firmware upgrade");
            if (handleFirmwareUpgrade(msgLen))
            {
                prepareReadingNextRequest();
            }
        }
        else
        {

            // read entire request body into buffer
            while (requestBodyBytesRead < msgLen)
            {
                int read = readTimedInto(requestBodyBytes, requestBodyBytesRead, msgLen - requestBodyBytesRead);
                if (read > 0)
                {
                    requestBodyBytesRead += read;
                }
                else
                {
                    break;
                }
            }

            if (requestBodyBytesRead == msgLen)
            {
                handleRequest(requestType, msgLen);
                prepareReadingNextRequest();
            }
        }
    }

    // server timeout
    if (millis() - lastRequestFromServer > SERVER_TIMEOUT_IN_MS)
    {
        // server timeout
        Log.log("Timeout while reading next command. Giving up connection");
        client.stop();
        return;
    }
}

void SmartHomeServerClient::prepareReadingNextRequest()
{
    requestInitialBytesRead = 0;
    requestBodyBytesRead = 0;
    requestFirmwareCRC32BytesRead = 0;
    requestFirmwareBytesStored = 0;
    lastRequestFromServer = millis();
}

void SmartHomeServerClient::handleRequest(int requestType, int msgLen)
{

    if (requestType == REQUEST_TYPE_PING)
    {
        handlePing();
        Log.log("Sendt pong response");
    }
    else if (requestType == REQUEST_TYPE_GET_DATA)
    {
        handleGetData();
        Log.log("Sendt data response");
    }
    else if (requestType == REQUEST_TYPE_SET_CHARGE_RATE)
    {
        int chargeRate = requestBodyBytes[0];
        charger->setChargeCurrentAmps(chargeRate);

        byte sendingBuffer[4 + 1];
        sendingBuffer[0] = RESPONSE_TYPE_SET_CHARGE_RATE; // response type: charge-rate
        writeUint32(0, sendingBuffer, 1);                 // msg leng

        // send
        client.write(sendingBuffer, 5);

        Log.log("Sendt chargeRate response");
    }
    else
    {
        char buf[100];
        snprintf(buf, 100, "Unknown request type %d", requestType);
        Log.log(buf);
    }

    client.flush();
}

void SmartHomeServerClient::handleGetData()
{
    /*
     * 
     * [field]            | size in bytes
     * chargingState      | 1
     * chargeCurrentAmps  | 1
     * pilotVoltage       | 1
     * proximityPilotAmps | 1
     * phase1Millivolts   | 4
     * phase2Millivolts   | 4
     * phase3Millivolts   | 4
     * phase1Milliamps    | 4
     * phase2Milliamps    | 4
     * phase3Milliamps    | 4
     * wifi RSSI          | 4 (signed int)
     * system uptime      | 4
     * logBuffer          | Log.bytesLogged
     * 
     */

    int sendingBufferSize = 1 +              // 1: response type
                            4 +              // 4: msgLen
                            (1 * 4) +        // 1'er
                            (4 * 8) +        // 4'er
                            Log.bytesLogged; // logging buffer;

    byte sendingBuffer[sendingBufferSize];

    sendingBuffer[0] = RESPONSE_TYPE_DATA;                    // response type
    writeUint32(sendingBufferSize - 1 - 4, sendingBuffer, 1); // sendingBufferSize - type - msgLen

    ChargerData chargerData = charger->getData();
    sendingBuffer[5] = chargerData.chargerState;
    sendingBuffer[6] = chargerData.chargeCurrentAmps;
    sendingBuffer[7] = chargerData.pilotVoltage;
    sendingBuffer[8] = chargerData.proximityPilotAmps;

    writeUint32(sensorVoltage->phase1Millivolts, sendingBuffer, 9);
    writeUint32(sensorVoltage->phase2Millivolts, sendingBuffer, 13);
    writeUint32(sensorVoltage->phase3Millivolts, sendingBuffer, 17);
    writeUint32(sensorCurrent->phase1Milliamps, sendingBuffer, 21);
    writeUint32(sensorCurrent->phase2Milliamps, sendingBuffer, 25);
    writeUint32(sensorCurrent->phase3Milliamps, sendingBuffer, 29);
    writeInt32(WiFi.RSSI(), sendingBuffer, 33);
    writeInt32(millis(), sendingBuffer, 37);
    writeCharArray(Log.logBuffer, Log.bytesLogged, sendingBuffer, 41);
    Log.bytesLogged = 0; // clear local log buffer

    client.write(sendingBuffer, sendingBufferSize);
}

bool SmartHomeServerClient::handleFirmwareUpgrade(int msgLen)
{
    while (requestFirmwareCRC32BytesRead < 4)
    {
        int read = readTimedInto(requestFirmwareCRC32Bytes, requestFirmwareCRC32BytesRead, 4 - requestFirmwareCRC32BytesRead);
        if (read > 0)
            requestFirmwareCRC32BytesRead += read;
        else
            break; // timeout is OK, we will come back here
    }

    if (requestFirmwareCRC32BytesRead < 4)
        return false;

    const int firmwareSize = msgLen - 4; // - 4 bytes crc32
    // read firmware, 1 bytes at a time
    int b;
    while (requestFirmwareBytesStored < firmwareSize)
    {
        b = readTimed();
        if (b < 0) // timeout
            break;

        if (requestFirmwareBytesStored == 0)
        {
            InternalStorage.open(firmwareSize);
        }

        InternalStorage.write(b);
        requestFirmwareBytesStored++;
    }

    if (requestFirmwareBytesStored < firmwareSize)
        return false;

    InternalStorage.close();

    // calc crc32
    unsigned receivedCRC32 = toUInt(requestFirmwareCRC32Bytes, 0);
    crc32.reset();
    byte *addr = (byte *)InternalStorage.STORAGE_START_ADDRESS;
    for (int i = 0; i < firmwareSize; i++)
    {

        crc32.update(*addr);
        addr++;
    }
    unsigned int firmwareChecksum = crc32.finalize();

    if (firmwareChecksum != receivedCRC32)
    {
        char logBuf[200];
        snprintf(logBuf, 200, "CRC32-error: firmwareSize=%d receivedCRC32=%u firmwareChecksum=%u", firmwareSize, receivedCRC32, firmwareChecksum);
        Log.log(logBuf);
    }
    else
    {
        Log.log("Rebooting to new firmware now");
        Serial.flush();
        delay(1000);
        InternalStorage.apply();
    }

    return true;
}

void SmartHomeServerClient::handlePing()
{
    // write pong response
    byte sendingBuffer[4 + 1];
    sendingBuffer[0] = RESPONSE_TYPE_PONG; // response type: pong
    writeUint32(0, sendingBuffer, 1);      // msg leng

    // responseType: pong
    client.write(sendingBuffer, 5);
}

void SmartHomeServerClient::connectToWifi()
{
    while (wifiStatus != WL_CONNECTED)
    {
        char buf[100];
        snprintf(buf, 100, "Attempting to connect to SSID: %s", MY_SSID);
        Log.log(buf);

        wifiStatus = WiFi.begin(MY_SSID, MY_PASS);
        delay(10000);
    }
}

int SmartHomeServerClient::toInt(byte src[], int srcOffset)
{
    int result = 0;
    result = result + src[srcOffset + 0];
    result <<= 8;
    result = result + src[srcOffset + 1];
    result <<= 8;
    result = result + src[srcOffset + 2];
    result <<= 8;
    result = result + src[srcOffset + 3];
    return result;
}
unsigned int SmartHomeServerClient::toUInt(byte src[], int srcOffset)
{
    unsigned int result = 0;
    result = result + src[srcOffset + 0];
    result <<= 8;
    result = result + src[srcOffset + 1];
    result <<= 8;
    result = result + src[srcOffset + 2];
    result <<= 8;
    result = result + src[srcOffset + 3];
    return result;
}

void SmartHomeServerClient::writeUint32(unsigned int src, byte dst[], int dstOffset)
{
    dst[dstOffset + 0] = (src >> 24) & 0xFF;
    dst[dstOffset + 1] = (src >> 16) & 0xFF;
    dst[dstOffset + 2] = (src >> 8) & 0xFF;
    dst[dstOffset + 3] = src & 0xFF;
}
void SmartHomeServerClient::writeInt32(int src, byte dst[], int dstOffset)
{
    dst[dstOffset + 0] = (src >> 24) & 0xFF;
    dst[dstOffset + 1] = (src >> 16) & 0xFF;
    dst[dstOffset + 2] = (src >> 8) & 0xFF;
    dst[dstOffset + 3] = src & 0xFF;
}
void SmartHomeServerClient::writeCharArray(char src[], int srcLength, byte dst[], int dstOffset)
{
    for (int i = 0; i < srcLength; i++)
    {
        dst[dstOffset + i] = src[i];
    }
}
void SmartHomeServerClient::writeSerial16Bytes(byte dst[], int dstOffset)
{
    volatile uint32_t *ptr1 = (volatile uint32_t *)0x0080A00C;
    writeUint32(*ptr1, dst, dstOffset);
    volatile uint32_t *ptr = (volatile uint32_t *)0x0080A040;
    writeUint32(*ptr, dst, dstOffset + 4);
    ptr++;
    writeUint32(*ptr, dst, dstOffset + 8);
    ptr++;
    writeUint32(*ptr, dst, dstOffset + 12);
}

int SmartHomeServerClient::readTimedInto(byte dst[], int dstOffset, int length)
{
    int bytesRead = 0;
    int b;
    while (bytesRead < length)
    {
        b = readTimed();
        if (b < 0)
            break;

        dst[dstOffset + bytesRead++] = b;
    }

    return bytesRead;
}

int SmartHomeServerClient::readTimed()
{
    unsigned long startTime = millis();
    int b;
    while (1)
    {
        if (millis() - startTime > TCP_READ_TIMEOUT_IN_MS)
        {
            break;
        }

        b = client.read();
        if (b >= 0)
        {
            return b;
        }
    }

    return -1;
}