/*
 * Parameters (FLASH storage)
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2024-2026
 */

#ifndef PARAMS_H_
#define PARAMS_H_

#include "hmac.h"
#include "sim800l.h"

#if __has_include("gitcommit.h")
#    include "gitcommit.h"
#else
#    define GIT_COMMIT_HASH "-"
#endif

#define PARAMS_DEVICE_NAME "umeter-b1"

#define PARAMS_FW_B1 0
#define PARAMS_FW_B2 1
#define PARAMS_FW_B3 2
#define PARAMS_FW_B4 1

#define PARAMS_FW_VERSION                                              \
    (((uint32_t)PARAMS_FW_B1 << 24) | ((uint32_t)PARAMS_FW_B2 << 16) | \
     ((uint32_t)PARAMS_FW_B3 << 8) | ((uint32_t)PARAMS_FW_B4))

#define PARAMS_MAGIC_EMPTY 0xFFFFFFFF
#define PARAMS_MAGIC_VALID 0xAA55000D

#define PARAMS_APP_URL_SIZE 64
#define PARAMS_OTA_URL_SIZE 64
#define PARAMS_MCU_UID_SIZE 32

typedef struct {
    uint32_t magic;
    uint32_t id;
    /* Do not edit items above */

    char apn[SIM800L_APN_SIZE];
    char url_ota[PARAMS_OTA_URL_SIZE];
    char url_app[PARAMS_APP_URL_SIZE];
    char mcu_uid[PARAMS_MCU_UID_SIZE];
    uint8_t secret[HMAC_SECRET_SIZE];
    uint32_t period_upload;
    uint32_t period_sensors;
    uint32_t period_anemometer;
    uint32_t offset_angle;
} __attribute__((aligned(8))) params_t;

extern params_t params;

void params_get(params_t* p);
void params_set(params_t* p);
void params_init(void);

#endif /* PARAMS_H_ */
