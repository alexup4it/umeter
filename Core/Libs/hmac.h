/*
 * HMAC
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2025
 */

#ifndef HMAC_H_
#define HMAC_H_

#include "hmac_sha256.h"
#include "sha256.h"

#define HMAC_SECRET_SIZE SHA256_HASH_SIZE
#define HMAC_BASE64_LEN  (4 * ((SHA256_HASH_SIZE + 2) / 3) + 1)

/*
 * @brief: HMAC SHA256 in base64 format
 * @param secret: secret key buffer, buffer length should be equal
 * HMAC_SECRET_SIZE
 * @param data: payload buffer
 * @param len: payload length
 * @param out: output buffer to store HMAC in base64 format, buffer length
 * should be equal HMAC_BASE64_LEN
 */
void hmac_base64(const uint8_t* secret,
                 const void* data,
                 size_t len,
                 char* out);

#endif /* HMAC_H_ */
