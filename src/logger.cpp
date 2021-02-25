
#include "logger.h"

Logger::Logger() {}

void Logger::log(const char *msg)
{

    if ((bytesLogged + 1) >= LOG_BUFFER_SIZE)
    {
        return;
    }

    int i = 0;
    while (msg[i] != '\0' && (bytesLogged + 1) < LOG_BUFFER_SIZE)
    {
        logBuffer[bytesLogged++] = msg[i++];
    }
    logBuffer[bytesLogged++] = '\0';
}
