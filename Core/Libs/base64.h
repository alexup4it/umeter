/*
 * base64 encoder
 *
 * Source: https://stackoverflow.com/questions/342409/how-do-i-base64-encode-decode-in-c
 * (19.10.2024)
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2024
 */

#ifndef BASE64_H_
#define BASE64_H_

#include <stddef.h>

void base64_encode(const unsigned char* in,
                   size_t inlen,
                   char* out,
                   size_t* outlen);

#endif /* BASE64_H_ */
