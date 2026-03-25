/*
 * Firmware storage
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2024-2026
 */

#ifndef FWS_H_
#define FWS_H_

#include <stdint.h>

#include "stm32f4xx_hal.h"
#include "w25q_s.h"

#define FWS_HEADER_ADDR  0x00
#define FWS_HEADER_SIZE  W25Q_SECTOR_SIZE
#define FWS_PAYLOAD_ADDR (FWS_HEADER_ADDR + FWS_HEADER_SIZE)

#define FWS_WINBOND_MANUFACTURER_ID 0xEF

#define FWS_CHECKSUM_INIT 0x5A5A5A5A

#define FWS_BL_GIT_COMMIT_HASH_LEN 64

enum fws_status {
    FWS_STATUS_NO_FW,
    FWS_STATUS_SUCCESS,
    FWS_STATUS_ERR_NO_STORAGE,
    FWS_STATUS_ERR_FW_SIZE,
    FWS_STATUS_ERR_CHECKSUM_STORAGE,
    FWS_STATUS_ERR_CHECKSUM_LOADED,
    FWS_STATUS_ERR_ERASE,
};

/**
 * @brief: Firmware header
 * @param loaded: Firmware was already loaded (!= 0) or not (== 0)
 * @param size: Firmware size in bytes
 * @param checksum: Checksum
 * Sum of all firmware data (uint32_t) + FWS_CHECKSUM_INIT
 */
struct fws {
    uint32_t loaded;
    uint32_t size;
    uint32_t checksum;
};

/**
 * @brief: Bootloader status
 */
struct bl_params {
    uint32_t status;
    uint32_t : 32;
    uint8_t hash[FWS_BL_GIT_COMMIT_HASH_LEN];
};

#endif /* FWS_H_ */
