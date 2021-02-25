#ifndef _LOGGER_H
#define _LOGGER_H

#include <Arduino.h>

#define LOG_BUFFER_SIZE 1024

class Logger
{
private:
public:
    char logBuffer[LOG_BUFFER_SIZE];
    uint32_t bytesLogged = 0;

    Logger();
    void log(const char *msg);
};

#endif
