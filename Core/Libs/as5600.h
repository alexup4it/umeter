/*
 * AS5600 contactless potentiometer
 */

#ifndef AS5600_H_
#define AS5600_H_

#include "stm32f4xx_hal.h"

struct as5600 {
    I2C_HandleTypeDef* i2c;
};

enum as5600_status {
    AS5600_STATUS_MH = 0x08,  // Magnet too strong
    AS5600_STATUS_ML = 0x10,  // Magnet too weak
    AS5600_STATUS_MD = 0x20,  // Magnet was detected
};

void as5600_init(struct as5600* self, I2C_HandleTypeDef* i2c);
int as5600_status(struct as5600* self);
int32_t as5600_read(struct as5600* self);

#endif /* AS5600_H_ */
