/*
 * HMAC
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2025
 */

#include "hmac.h"

#include <string.h>

#include "base64.h"

void hmac_base64(const uint8_t* secret,
                 const char* data,
                 size_t len,
                 char* out) {
    uint8_t hmac[SHA256_HASH_SIZE];
    size_t enclen;

    hmac_sha256(secret, HMAC_SECRET_SIZE, data, len, hmac, SHA256_HASH_SIZE);

    base64_encode((unsigned char*)hmac, SHA256_HASH_SIZE, out, &enclen);
    out[enclen] = '\0';
}
