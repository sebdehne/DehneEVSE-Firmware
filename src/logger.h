#ifndef _LOGGER_H
#define _LOGGER_H

#include <Arduino.h>
#include "config.h"

class Logger
{
private:
public:
    char logBuffer[LOG_BUFFER_SIZE];
    unsigned long bytesLogged = 0;

    Logger();
    void log(const char *msg);
};

extern Logger Log;

#endif
