
#include "logger.h"

Logger::Logger() {}

void Logger::log(const char *input)
{

    size_t inputLen = strlen(input);

    size_t bufLen = inputLen + 12 + 1;
    char buf[bufLen];
    size_t msgLen = snprintf(buf, bufLen, "%lu: %s", millis(), input);

    Serial.println(buf);

    if (msgLen < 0 || msgLen > LOG_BUFFER_SIZE + 1)
    {
        return;
    }

    if ((msgLen + 1) >= (LOG_BUFFER_SIZE - bytesLogged))
    {
        Serial.println("Log buffer full, clearing it");
        bytesLogged = 0;
    }

    int i = 0;
    while (buf[i] != '\0')
    {
        logBuffer[bytesLogged++] = buf[i++];
    }
    logBuffer[bytesLogged++] = '\0';
}

Logger Log;
