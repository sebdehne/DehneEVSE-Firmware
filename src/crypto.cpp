#include "crypto.h"
#include <Arduino.h>

#include "utils.h"

CryptUtilClass CryptUtil;

CryptUtilClass::CryptUtilClass() {
    gcmaes256 = new GCM<AES256>();
}

int CryptUtilClass::encrypt(unsigned char plaintext[], const size_t plaintextLen, unsigned char *dstBuff, const size_t dstBuffLen, const unsigned long time)
{

    size_t totalLength = (plaintextLen + sizeof(tag) + sizeof(time));
    if (dstBuffLen < totalLength) {
        return -1;
    }

    gcmaes256->clear();
    gcmaes256->setKey(key, gcmaes256->keySize());

    writeUint32(time, iv, 0);
    gcmaes256->setIV(iv, sizeof(iv));
    gcmaes256->encrypt(dstBuff, plaintext, plaintextLen);
    gcmaes256->computeTag(tag, sizeof(tag));
    dstBuff += plaintextLen;
    memcpy(dstBuff, tag, sizeof(tag));
    dstBuff += sizeof(tag);
    writeUint32(time, dstBuff, 0);

    return totalLength;
}

bool CryptUtilClass::decrypt(unsigned char ciphertextAndTagAndTime[], const size_t ciphertextAndTagAndTimeLength, unsigned char dstBuff[], const size_t dstBuffLength, const unsigned long time)
{
    gcmaes256->clear();

    unsigned long receivedTime;
    size_t tagSize = sizeof(tag);
    size_t receivedTimeSize = sizeof(receivedTime);
    size_t ciphertextSize = ciphertextAndTagAndTimeLength - tagSize - receivedTimeSize;

    unsigned char *ptr = ciphertextAndTagAndTime;
    ptr += ciphertextSize;
    memcpy(tag, ptr, tagSize);
    receivedTime = toUInt(ptr, tagSize);

    gcmaes256->setKey(key, gcmaes256->keySize());

    writeUint32(receivedTime, iv, 0);
    gcmaes256->setIV(iv, sizeof(iv));
    gcmaes256->decrypt(dstBuff, ciphertextAndTagAndTime, ciphertextSize);

    if (!gcmaes256->checkTag(tag, tagSize))
    {
        Serial.println("Tag-validation failed");
        return false;
    }

    long delta = time - receivedTime;

    // allow 30 second clock-slew
    if (delta > 15 || delta < -15)
    {
        Serial.println("Time-validation failed");
        return false;
    }

    return true;
}
