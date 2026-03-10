/*
 * AS5600 contactless potentiometer
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2025-2026
 */

#ifndef AS5600_H_
#define AS5600_H_

#include "stm32f4xx_hal.h"

typedef void (*as5600_power_cb)(void);

struct as5600
{
	I2C_HandleTypeDef *i2c;
	as5600_power_cb power_on;
	as5600_power_cb power_off;
};

enum as5600_status
{
	AS5600_STATUS_MH = 0x08, // Magnet too strong
	AS5600_STATUS_ML = 0x10, // Magnet too weak
	AS5600_STATUS_MD = 0x20, // Magnet was detected
};


void as5600_init(struct as5600 *sen, I2C_HandleTypeDef *i2c,
				 as5600_power_cb power_on, as5600_power_cb power_off);
int as5600_is_available(struct as5600 *sen);
int as5600_status(struct as5600 *sen);
int32_t as5600_read(struct as5600 *sen);

#endif /* AS5600_H_ */
