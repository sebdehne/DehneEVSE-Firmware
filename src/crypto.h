#ifndef _CRYPTO_H
#define _CRYPTO_H

#include <Crypto.h>
#include <AES.h>
#include <GCM.h>
#include "secrets.h"

/*
 * AES-256 GCM
 */
class CryptUtilClass
{
private:

    const unsigned char key[32] = AES265_GCM_KEY;
    unsigned char iv[12] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    unsigned char tag[16];
    GCM<AES256> *gcmaes256 = 0;

public:
    CryptUtilClass();

    int encrypt(unsigned char plaintext[], const size_t plaintextLen, unsigned char *dstBuff, const size_t dstBuffLen, const unsigned long time);
    bool decrypt(unsigned char ciphertextAndTagAndTime[], const size_t ciphertextAndTagAndTimeLength, unsigned char dstBuff[], const size_t dstBuffLength, const unsigned long time);
};

extern CryptUtilClass CryptUtil;

#endif