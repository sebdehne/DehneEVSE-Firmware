#include "utils.h"

#include <Arduino.h>

void writeSerial16Bytes(unsigned char dst[], int dstOffset)
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

void writeCharArray(char src[], int srcLength, unsigned char dst[], int dstOffset)
{
    for (int i = 0; i < srcLength; i++)
    {
        dst[dstOffset + i] = src[i];
    }
}

void writeUint32(unsigned int src, unsigned char dst[], int dstOffset)
{
    dst[dstOffset + 0] = (src >> 24) & 0xFF;
    dst[dstOffset + 1] = (src >> 16) & 0xFF;
    dst[dstOffset + 2] = (src >> 8) & 0xFF;
    dst[dstOffset + 3] = src & 0xFF;
}
void writeInt32(int src, unsigned char dst[], int dstOffset)
{
    dst[dstOffset + 0] = (src >> 24) & 0xFF;
    dst[dstOffset + 1] = (src >> 16) & 0xFF;
    dst[dstOffset + 2] = (src >> 8) & 0xFF;
    dst[dstOffset + 3] = src & 0xFF;
}
void writeUint8(unsigned char src, unsigned char dst[], int dstOffset)
{
    dst[dstOffset] = src;
}
void writeBool(bool src, unsigned char dst[], int dstOffset)
{
    if (src)
        dst[dstOffset] = 1;
    else
        dst[dstOffset] = 0;
}

unsigned int toUInt(unsigned char src[], int srcOffset)
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

int toInt(unsigned char src[], int srcOffset)
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
