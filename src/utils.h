#ifndef _UTILS_H
#define _UTILS_H

void writeSerial16Bytes(unsigned char dst[], int dstOffset);
void writeCharArray(char src[], int srcLength, unsigned char dst[], int dstOffset);

void writeUint32(unsigned int src, unsigned char dst[], int dstOffset);
void writeInt32(int src, unsigned char dst[], int dstOffset);
void writeUint8(unsigned char src, unsigned char dst[], int dstOffset);
void writeBool(bool src, unsigned char dst[], int dstOffset);

unsigned int toUInt(unsigned char src[], int srcOffset);
int toInt(unsigned char src[], int srcOffset);

#endif