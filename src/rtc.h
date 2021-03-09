#ifndef _RTC_H
#define _RTC_H


class RTCClass
{
private:
    
public:
    RTCClass();
    
    unsigned long getEpoch2000Seconds();
};

extern RTCClass RTCUtils;


#endif