/*
 * base64 encoder
 *
 * Source: https://stackoverflow.com/questions/342409/how-do-i-base64-encode-decode-in-c
 * (19.10.2024)
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2024
 */

#include "base64.h"

#include <stdint.h>

static int mod_table[] = {0, 2, 1};
static char* encoding_table =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

void base64_encode(const unsigned char* in,
                   size_t inlen,
                   char* out,
                   size_t* outlen) {
    int i = 0;
    int j = 0;

    *outlen = 4 * ((inlen + 2) / 3);

    while (i < inlen) {
        uint32_t octet_a = i < inlen ? (unsigned char)in[i++] : 0;
        uint32_t octet_b = i < inlen ? (unsigned char)in[i++] : 0;
        uint32_t octet_c = i < inlen ? (unsigned char)in[i++] : 0;

        uint32_t triple = (octet_a << 0x10) + (octet_b << 0x08) + octet_c;

        out[j++] = encoding_table[(triple >> 3 * 6) & 0x3F];
        out[j++] = encoding_table[(triple >> 2 * 6) & 0x3F];
        out[j++] = encoding_table[(triple >> 1 * 6) & 0x3F];
        out[j++] = encoding_table[(triple >> 0 * 6) & 0x3F];
    }

    for (int k = 0; k < mod_table[inlen % 3]; k++) {
        out[*outlen - 1 - k] = '=';
    }
}
