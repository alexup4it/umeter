/*
 * Parameters (FLASH storage)
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2024-2026
 */

#include <params.h>
#include <stdlib.h>
#include <string.h>

#include "stm32f4xx_hal.h"

params_t params;

extern const uint32_t* _storage;
//#define PARAMS_ADDRESS ((uint32_t) &_storage)
static uint32_t PARAMS_ADDRESS = ((uint32_t)&_storage);

/*
 * FLASH_SECTOR_0 - 16K - bootloader
 * FLASH_SECTOR_1 - 16K - storage
 * FLASH_SECTOR_2 ... FLASH_SECTOR_7 - 480K - application
 */
#define PARAMS_SECTOR     FLASH_SECTOR_1
#define PARAMS_NB_SECTORS 1

inline static void flash_get(uint32_t address, size_t size, uint8_t* buffer) {
    memcpy(buffer, (uint8_t*)address, size);
}

static void flash_set(uint32_t address, size_t size, uint8_t* buffer) {
    // Alignment: 4
    if (size & 0x03) {
        size = size + 0x04 - (size & 0x03);
    }

    HAL_FLASH_Unlock();
    while (size) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                          address,
                          *((uint32_t*)buffer));
        size -= 4;
        address += 4;
        buffer += 4;
    }
    HAL_FLASH_Lock();
}

static void flash_erase(uint32_t sector, uint32_t nb_sectors) {
    uint32_t page_error;
    FLASH_EraseInitTypeDef erase_init;
    erase_init.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase_init.Banks        = FLASH_BANK_1;
    erase_init.Sector       = sector;
    erase_init.NbSectors    = nb_sectors;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&erase_init, &page_error);
    HAL_FLASH_Lock();
}

void params_get(params_t* params) {
    flash_get(PARAMS_ADDRESS, sizeof(params_t), (uint8_t*)params);
}

void params_set(params_t* params) {
    flash_erase(PARAMS_SECTOR, PARAMS_NB_SECTORS);
    flash_set(PARAMS_ADDRESS, sizeof(params_t), (uint8_t*)params);
}

static void uid_string(char* uid) {
    char temp[16];

    utoa(HAL_GetUIDw0(), temp, 16);
    strcpy(uid, temp);
    utoa(HAL_GetUIDw1(), temp, 16);
    strcat(uid, temp);
    utoa(HAL_GetUIDw2(), temp, 16);
    strcat(uid, temp);
}

// Do not edit magic and id
static void set_default(params_t* params) {
    memset(params, 0, sizeof(params_t));
    strcpy(params->apn, "internet");
    strcpy(params->url_ota, "ota.proshutinsky.ru");
    strcpy(params->url_app, "app.proshutinsky.ru");
    uid_string(params->mcu_uid);
    memset(params->secret, 0xAB, sizeof(params->secret));
    params->period_upload     = 60 * 5;
    params->period_sensors    = 60;
    params->period_anemometer = 10;
    params->offset_angle      = 0;
}

void params_init(void) {
    uint32_t id;

    params_get(&params);

    // First initialization
    if (params.magic != PARAMS_MAGIC_VALID) {
        // Empty - set id to 0
        if (params.magic == PARAMS_MAGIC_EMPTY) {
            id = 0;
        }
        // Not empty - use saved id
        else {
            id = params.id;
        }

        set_default(&params);
        params.magic = PARAMS_MAGIC_VALID;
        params.id    = id;

        params_set(&params);
    }
}
