#include <Arduino.h>
#include "smart_home_server_client.h"
#include "config.h"
#include <WiFiNINA.h>
#include <ArduinoOTA.h>
#include <CRC32.h>

SmartHomeServerClient::SmartHomeServerClient()
{
}

void SmartHomeServerClient::setup(Charger *chargerInput, SensorVoltage *sensorVoltageInput, SensorCurrent *sensorCurrentInput, Logger *loggerIn)
{
    logger = loggerIn;
    charger = chargerInput;
    sensorVoltage = sensorVoltageInput;
    sensorCurrent = sensorCurrentInput;

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

    Serial.println("SmartHomeServerClient setup complete");
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
        delay(5000);
        client.stop();
        if (client.connect(server, 9091))
        {
            char buf[100];
            snprintf(buf, 100, "Connected to server: %s:%d", SERVER_IP, 9091);
            logger->log(buf);
            Serial.print("Connected to server: ");
            Serial.print(SERVER_IP);
            Serial.print(":");
            Serial.println(9091);

            lastRequestFromServer = millis();
            bytesRead = 0;

            // send CLientId
            uint8_t sendingBuf[17];
            writeSerial16Bytes(sendingBuf, 0);
            sendingBuf[16] = VERSION;
            // send Firmware version
            client.write(sendingBuf, 17);
        }
        else
        {
            Serial.println("Could not connect to server");
            return;
        }
    }

    // try to read more of the initial 5 bytes (type + msgLen)
    if (bytesRead < 5)
    {
        int read = client.readBytes(requestInitialBytes, 5 - bytesRead);
        if (read > 0)
        {
            bytesRead += read;
        }
    }

    if (bytesRead == 5)
    {
        lastRequestFromServer = millis();
        bytesRead = 0;
        uint32_t msgLen = toInt(requestInitialBytes, 1);
        handleRequest((uint8_t)requestInitialBytes[0], msgLen);
        return;
    }

    // server timeout
    if (millis() - lastRequestFromServer > SERVER_READ_TIMEOUT_IN_MS)
    {
        // server timeout
        logger->log("Timeout while reading next command. Giving up connection");

        Serial.println("Timeout while reading next command. Giving up connection");
        client.stop();
        return;
    }
}

void SmartHomeServerClient::handleRequest(u_int8_t requestType, uint32_t msgLen)
{

#ifdef DEBUG
    Serial.print("Got requestType: ");
    Serial.println(requestType);
    Serial.print("Got msgLen: ");
    Serial.println(msgLen);
#endif

    if (requestType == REQUEST_TYPE_PING)
    {
        handlePing();
    }

    else if (requestType == REQUEST_TYPE_FIRMWARE)
    {
        handleFirmwareUpgrade(msgLen);
    }
    else if (requestType == REQUEST_TYPE_GET_DATA)
    {
        handleGetData();
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
     * bytesLogged        | 4
     * logBuffer          | 1024
     * 
     * Total msgLen = 1064
     */

    Serial.println("Responding to GET_DATA");

    uint8_t sendingBuffer[1064 + 4 + 1];

    sendingBuffer[0] = RESPONSE_TYPE_DATA; // response type
    writeUint32(1064, sendingBuffer, 1);   // msg leng

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
    writeUint32(logger->bytesLogged, sendingBuffer, 41);
    writeCharArray(logger->logBuffer, logger->bytesLogged, sendingBuffer, 45);
    logger->bytesLogged = 0; // clear log buffer
    client.write(sendingBuffer, 1064 + 4 + 1);

    Serial.println("DONE writing");
}

void SmartHomeServerClient::handleFirmwareUpgrade(u_int32_t msgLen)
{
    const uint32_t firmwareSize = msgLen - 4;

    // read CRC32
    uint8_t buf[4];
    if (!client.readBytes(buf, 4))
    {
        client.stop();
        return;
    }
    u_int32_t receivedCRC32 = toInt(buf, 0);

    // write content to flash:
    uint32_t remaining = firmwareSize;
    InternalStorage.open(firmwareSize);
    byte b;
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
        char buf[100];
        snprintf(buf, 100, "Timeout downloading update file at %u bytes. Can't continue with update.", (unsigned int)remaining);
        logger->log(buf);
        client.stop();
        return;
    }

    // calc crc32
    crc32.reset();
    uint8_t *addr = (uint8_t *)InternalStorage.STORAGE_START_ADDRESS;
    for (size_t i = 0; i < firmwareSize; i++)
    {
        crc32.update(*addr);
        addr++;
    }
    uint32_t firmwareChecksum = crc32.finalize();

    if (firmwareChecksum != receivedCRC32)
    {
        Serial.print("Received ");
        Serial.print(firmwareSize);
        Serial.print(" bytes, but received CRC32=");
        Serial.print(receivedCRC32);
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

void SmartHomeServerClient::handlePing()
{
    Serial.println("Responding to PING");
    char buf[100];
    snprintf(buf, 100, "Responding to PING");
    logger->log(buf);

    // write pong response

    uint8_t sendingBuffer[4 + 1];
    sendingBuffer[0] = RESPONSE_TYPE_PONG; // response type: pong
    writeUint32(0, sendingBuffer, 1);      // msg leng

    // responseType: pong
    client.write(sendingBuffer, 5);
}

void SmartHomeServerClient::connectToWifi()
{
    while (wifiStatus != WL_CONNECTED)
    {
        Serial.print("Attempting to connect to SSID: ");
        Serial.println(MY_SSID);
        char buf[100];
        snprintf(buf, 100, "Attempting to connect to SSID: %s", MY_SSID);
        logger->log(buf);

        wifiStatus = WiFi.begin(MY_SSID, MY_PASS);
        delay(10000);
    }
}

uint32_t SmartHomeServerClient::toInt(byte src[], size_t srcOffset)
{
    uint32_t result = 0;
    result = result + src[srcOffset + 0];
    result <<= 8;
    result = result + src[srcOffset + 1];
    result <<= 8;
    result = result + src[srcOffset + 2];
    result <<= 8;
    result = result + src[srcOffset + 3];
    return result;
}

void SmartHomeServerClient::writeUint32(uint32_t src, uint8_t dst[], size_t dstOffset)
{
    dst[dstOffset + 0] = (src >> 24) & 0xFF;
    dst[dstOffset + 1] = (src >> 16) & 0xFF;
    dst[dstOffset + 2] = (src >> 8) & 0xFF;
    dst[dstOffset + 3] = src & 0xFF;
}
void SmartHomeServerClient::writeInt32(int32_t src, uint8_t dst[], size_t dstOffset)
{
    dst[dstOffset + 0] = (src >> 24) & 0xFF;
    dst[dstOffset + 1] = (src >> 16) & 0xFF;
    dst[dstOffset + 2] = (src >> 8) & 0xFF;
    dst[dstOffset + 3] = src & 0xFF;
}
void SmartHomeServerClient::writeUint32Array(uint32_t src[], size_t srcLen, uint8_t dst[], size_t dstOffset)
{
    for (size_t i = 0; i < srcLen; i++)
    {
        writeUint32(src[i], dst, dstOffset + (i * 4));
    }
}
void SmartHomeServerClient::writeCharArray(char src[], size_t srcLength, uint8_t dst[], size_t dstOffset)
{
    for (size_t i = 0; i < srcLength; i++)
    {
        dst[dstOffset + i] = src[i];
    }
}
void SmartHomeServerClient::writeSerial16Bytes(uint8_t dst[], size_t dstOffset)
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
