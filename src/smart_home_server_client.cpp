#include <Arduino.h>
#include <WiFiNINA.h>
#include <ArduinoOTA.h>
#include <CRC32.h>
#include "smart_home_server_client.h"
#include "pwm.h"
#include "adc.h"
#include "contactor.h"
#include "sensor_current.h"
#include "sensor_voltage.h"
#include "utils.h"

SmartHomeServerClientClass SmartHomeServerClient;

SmartHomeServerClientClass::SmartHomeServerClientClass()
{
}

void SmartHomeServerClientClass::setup()
{

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

void SmartHomeServerClientClass::tick(bool sendNotify)
{

    // server charging timeout
    if (millis() - lastRequestFromServer > SERVER_CHARGING_TIMEOUT_IN_MS &&
        (Contactor.isOn() || PwmD3.getCurrentPwmDutyCycle_percent() != 100))
    {
        // no communication with server for too long, disabling off charging
        Log.log("Disabling charging due to missing server comunication");
        Contactor.switchOff();
        PwmD3.updateDutyCycle(100);
        return;
    }

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
        int read = readInto(requestInitialBytes, requestInitialBytesRead, 5 - requestInitialBytesRead);
        if (read > 0)
            requestInitialBytesRead += read;
        else
            break; // timeout is OK, we will come back here

        if (requestInitialBytesRead == 5)
        {
            int msgLen = toInt(requestInitialBytes, 1);
#ifdef DEBUG
            char logBuf[100];
            snprintf(logBuf, 100, "Got: requestType=%d msgLen=%d", requestInitialBytes[0], msgLen);
            Log.log(logBuf);
#endif
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
                int read = readInto(requestBodyBytes, requestBodyBytesRead, msgLen - requestBodyBytesRead);
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
    if (millis() - lastRequestFromServer > SERVER_CONNECTION_TIMEOUT_IN_MS)
    {
        // server timeout
        Log.log("Timeout while reading next command. Giving up connection");
        client.stop();
        return;
    }

    if (sendNotify)
    {
        unsigned char sendingBuffer[4 + 1];
        sendingBuffer[0] = NOTIFY_TYPE_DATA_CHANGED; // response type: charge-rate
        writeUint32(0, sendingBuffer, 1);            // msg leng

        // send
        client.write(sendingBuffer, 5);

        Log.log("Notify dataChanged sent");
    }
}

void SmartHomeServerClientClass::prepareReadingNextRequest()
{
    requestInitialBytesRead = 0;
    requestBodyBytesRead = 0;
    requestFirmwareCRC32BytesRead = 0;
    requestFirmwareBytesStored = 0;
    lastRequestFromServer = millis();
}

void SmartHomeServerClientClass::handleRequest(int requestType, int msgLen)
{

    if (requestType == REQUEST_TYPE_PING)
    {
        handlePing();
#ifdef DEBUG
        Log.log("Sendt pong response");
#endif
    }
    else if (requestType == REQUEST_TYPE_COLLECT_DATA)
    {
        handleCollectData();
#ifdef DEBUG
        Log.log("Sendt data response");
#endif
    }
    else if (requestType == REQUEST_TYPE_SET_PWM_PERCENT)
    {
        unsigned int pwmPercent = requestBodyBytes[0];

        PwmD3.updateDutyCycle(pwmPercent);

        unsigned char sendingBuffer[4 + 1];
        sendingBuffer[0] = RESPONSE_TYPE_SET_PWM_PERCENT; // response type: charge-rate
        writeUint32(0, sendingBuffer, 1);                 // msg leng

        // send
        client.write(sendingBuffer, 5);

        char buf[100];
        snprintf(buf, 100, "PWM percent updated to %u", pwmPercent);
        Log.log(buf);
    }
    else if (requestType == REQUEST_TYPE_SET_CONTACTOR_STATE)
    {
        unsigned int contactorState = requestBodyBytes[0];
        bool isCurrentlyOn = Contactor.isOn();

        char buf[100];
        if (contactorState == 1 && !isCurrentlyOn)
        {
            Contactor.switchOn();
            snprintf(buf, 100, "Switched contactor on");
        }
        else if (contactorState == 0 && isCurrentlyOn)
        {
            Contactor.switchOff();
            snprintf(buf, 100, "Switched contactor off");
        }
        else
        {
            snprintf(buf, 100, "Contactor already in state=%u", contactorState);
        }

        unsigned char sendingBuffer[4 + 1];
        sendingBuffer[0] = RESPONSE_TYPE_SET_CONTACTOR_STATE; // response type: charge-rate
        writeUint32(0, sendingBuffer, 1);                     // msg leng

        // send
        client.write(sendingBuffer, 5);

        Log.log(buf);
    }
    else
    {
        char buf[100];
        snprintf(buf, 100, "Unknown request type %d", requestType);
        Log.log(buf);
    }

    client.flush();
}

void SmartHomeServerClientClass::handleCollectData()
{

    /*
     * 
     * [field]            | size in bytes
     * contactor          | 1
     * pwmPercent         | 1
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
     */

    int sendingBufferSize = 1 +              // 1: response type
                            4 +              // 4: msgLen
                            (4 * 1) +        // 1'er
                            (10 * 4) +        // 4'er
                            Log.bytesLogged; // logging buffer;

    unsigned char sendingBuffer[sendingBufferSize];

    sendingBuffer[0] = RESPONSE_TYPE_COLLECT_DATA;            // response type
    writeUint32(sendingBufferSize - 1 - 4, sendingBuffer, 1); // sendingBufferSize - type - msgLen

    writeBool(Contactor.isOn(), sendingBuffer, 5);
    writeUint8(PwmD3.getCurrentPwmDutyCycle_percent(), sendingBuffer, 6);
    writeUint8(AdcManager.currentPilotVoltage, sendingBuffer, 7);
    writeUint8(AdcManager.currentProximityPilotAmps, sendingBuffer, 8);
    writeUint32(SensorVoltage.phase1Millivolts(), sendingBuffer, 9);
    writeUint32(SensorVoltage.phase2Millivolts(), sendingBuffer, 13);
    writeUint32(SensorVoltage.phase3Millivolts(), sendingBuffer, 17);
    writeUint32(SensorCurrent.phase1Milliamps(), sendingBuffer, 21);
    writeUint32(SensorCurrent.phase2Milliamps(), sendingBuffer, 25);
    writeUint32(SensorCurrent.phase3Milliamps(), sendingBuffer, 29);
    writeInt32(WiFi.RSSI(), sendingBuffer, 33);
    writeInt32(millis(), sendingBuffer, 37);
    writeUint32(AdcManager.currentPilotControlAdc, sendingBuffer, 41);
    writeUint32(AdcManager.currentProximityPilotAdc, sendingBuffer, 45);
    writeCharArray(Log.logBuffer, Log.bytesLogged, sendingBuffer, 49);
    Log.bytesLogged = 0; // clear local log buffer

    client.write(sendingBuffer, sendingBufferSize);
}

bool SmartHomeServerClientClass::handleFirmwareUpgrade(int msgLen)
{
    while (requestFirmwareCRC32BytesRead < 4)
    {
        int read = readInto(requestFirmwareCRC32Bytes, requestFirmwareCRC32BytesRead, 4 - requestFirmwareCRC32BytesRead);
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
        b = readSingle();
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

void SmartHomeServerClientClass::handlePing()
{
    // write pong response
    byte sendingBuffer[4 + 1];
    sendingBuffer[0] = RESPONSE_TYPE_PONG; // response type: pong
    writeUint32(0, sendingBuffer, 1);      // msg leng

    // responseType: pong
    client.write(sendingBuffer, 5);
}

void SmartHomeServerClientClass::connectToWifi()
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

int SmartHomeServerClientClass::readInto(byte dst[], int dstOffset, int length)
{
    int bytesRead = 0;
    int b;
    while (bytesRead < length)
    {
        b = readSingle();
        if (b < 0)
            break;

        dst[dstOffset + bytesRead++] = b;
    }

    return bytesRead;
}

int SmartHomeServerClientClass::readSingle()
{
    return client.read();
}