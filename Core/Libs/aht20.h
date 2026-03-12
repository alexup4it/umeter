/*
 * AHT20 humidity and temperature sensor
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2024-2026
 */

#ifndef AHT20_H_
#define AHT20_H_

#include "stm32f4xx_hal.h"

struct aht20 {
    I2C_HandleTypeDef* i2c;
};

void aht20_init(struct aht20* sen, I2C_HandleTypeDef* i2c);
int aht20_is_available(struct aht20* sen);
int aht20_read(struct aht20* sen, int32_t* temp, int32_t* hum);

#endif /* AHT20_H_ */
